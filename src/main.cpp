#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <Wire.h>
#include <ISM330BXSensor.h>
#include <stdarg.h>
#include <MAX6675.h>

#define STACK_SIZE 256 // Increased stack size for tasks
#define MODBUS_BAUD 115200
#define MODBUS_CONFIG SERIAL_8N1
#define DEBUG_ENABLE 1 // Дебаг в серіал. Якщо поставити 0, то все пов'язане з дебагом не компілюється
#define DEBUG_MSG_SIZE 128


#if DEBUG_ENABLE
static char oneShotDebugMsg[DEBUG_MSG_SIZE];
static bool oneShotDebugMsgValid = false;

// Use this everywhere instead of sendDebugMessage(...)
void sendOneShotDebugMessage(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(oneShotDebugMsg, DEBUG_MSG_SIZE, format, args);
  va_end(args);
  oneShotDebugMsgValid = true;
}
#else
// no-op when debugging is disabled
inline void sendOneShotDebugMessage(const char*, ...) {}
#endif 

// Піни
const int16_t sensorPins[4] = {PC3, PC2, PC1, PC0}; // Інпути сенсорів
const int16_t outPins[4] = {PC13, PA0, PC15, PC14}; // Виводи для управління
const int16_t ledPins[4] = {PB12, PB13, PB14, PB15}; // Діоди індикації
const int16_t dePin = PA8;                          // Пін DE для RS-485
const int16_t addressPins[4] = {PC10, PC11, PC12, PD2}; // DIP світчі для адресації Modbus
const int thermoSCK = PA5;  // SCK pin
const int thermoCS = PA4;   // CS pin
const int thermoMISO = PA6;   // MISO pin

// Серіал для Modbus
HardwareSerial Serial1(USART1);


// Ініціалізація Modbus RTU як слейв, по Serial1, з DE піном
ModbusRTUSlave modbus(Serial1, dePin);

// I2C for IMU sensor - using PB7(SDA) and PB6(SCL)
TwoWire i2c(PB7, PB6);
ISM330BXSensor imu(&i2c, 0x6A); // Global IMU instance (SD0 to GND)

MAX6675 thermocouple(thermoSCK, thermoCS, thermoMISO);


// Modbus змінні
const uint8_t NUM_COILS = 5;
const uint8_t NUM_DISCRETE_INPUTS = 4;
const uint8_t NUM_HOLDING_REGISTERS = 5;
const uint8_t NUM_INPUT_REGISTERS = 6;

uint8_t modbusSlaveAddress = 3; // Адреса Modbus слейва
bool coils[NUM_COILS]; // Масив котушок
bool discreteInputs[NUM_DISCRETE_INPUTS]; // Масив дискретних входів
uint16_t holdingRegisters[NUM_HOLDING_REGISTERS]; // Масив регістрів
uint16_t inputRegisters[NUM_INPUT_REGISTERS];     // Масив регістрів вводу

bool address[4]; // Окремі біти адреси Modbus

// Сенсори та їх стани
int reloadSensor = 0; // Стан перезарядки
int lastReloadSensor = 0;
int reloadSignal = 0; // Сигнал перезарядки
int lastReloadSignal = 0;
int counterSensor = 0; // Стан сенсора лічильника
int lastCounterSensor = 0;
uint16_t bulletCounter = 0; // Лічильник куль

// IMU address and detection state
uint8_t imuAddress = 0;
bool imuFound = false;

// М'ютекс, щоб не було конфліктів при доступі до масивів Modbus між тасками
SemaphoreHandle_t modbusRegisterMutex; 

// Хендлери і декларації тасок
TaskHandle_t modbusTaskHandle;
TaskHandle_t sensorTaskHandle;
TaskHandle_t outputTaskHandle;
TaskHandle_t motorControlTaskHandle;
TaskHandle_t imuTaskHandle;
TaskHandle_t tempTaskHandle;

void modbusTask(void *pvParameters);
void sensorTask(void *pvParameters);
void outputTask(void *pvParameters);
void motorControlTask(void *pvParameters);
void imuTask(void *pvParameters);
void temperatureTask(void *pvParameters);


#if DEBUG_ENABLE
TaskHandle_t debugTaskHandle;
void debugTask(void *pvParameters);
#endif

void setup() {
  #if DEBUG_ENABLE
  Serial.begin(115200); // Основний серіал по юсб, для дебагу
  while(!Serial) { 
    delay(10);
  } 
  delay(100);
  #endif
  
  Serial1.begin(MODBUS_BAUD, MODBUS_CONFIG); // Modbus RTU
  Serial2.begin(115200, SERIAL_8N1); // Драйвер для моторів

  // Initialize the i2c object
  i2c.begin();
  i2c.setClock(400000);
  delay(500);

  #if DEBUG_ENABLE
  // I2C scanner - find the IMU
  Serial.println("Scanning I2C bus...");
  byte error, addressI2C;
  int deviceCount = 0;

  for(addressI2C = 1; addressI2C < 127; addressI2C++) {
    i2c.beginTransmission(addressI2C);
    error = i2c.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (addressI2C < 16) Serial.print("0");
      Serial.print(addressI2C, HEX);
      Serial.println();
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found! IMU may not be connected.");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }
  #endif

  // RS-485 driver enable pin
  pinMode(dePin, OUTPUT);

  // DIP перемикачі для адресації Modbus (4 біти), підтягнуті до VCC
  for (int i = 0; i < 4; i++) {
    pinMode(addressPins[i], INPUT_PULLUP);
  }
  
  // Сенсори, підтягнуті до VCC
  for (int i = 0; i < 4; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }
  
  // ВИводи для управління та індикації
  for (int i = 0; i < 4; i++) {
    pinMode(outPins[i], OUTPUT);
    pinMode(ledPins[i], OUTPUT);
  }
  
  // Задати адресу Modbus з DIP перемикачів (перевернуті, бо підтягнуті до VCC)
  for (int i = 0; i < 4; i++) {
    address[i] = !digitalRead(addressPins[i]);
    bitWrite(modbusSlaveAddress, i, address[i]);
  }

  // Конфігурація Modbus
  modbus.configureCoils(coils, NUM_COILS);
  modbus.configureDiscreteInputs(discreteInputs, NUM_DISCRETE_INPUTS);
  modbus.configureHoldingRegisters(holdingRegisters, NUM_HOLDING_REGISTERS);
  modbus.configureInputRegisters(inputRegisters, NUM_INPUT_REGISTERS);

  // Ініціалізація Modbus з адресою
  modbus.begin(modbusSlaveAddress, MODBUS_BAUD, MODBUS_CONFIG);
  
  // Створення м'ютексу для захисту масивів Modbus
  modbusRegisterMutex = xSemaphoreCreateMutex();
  
  // Створення тасок
  xTaskCreate(
    modbusTask,
    "ModbusTask",
    STACK_SIZE,
    NULL,
    2, // Найвища пріоритетність, бо Modbus повинен бути завжди активний
    &modbusTaskHandle
  );
  
  xTaskCreate(
    sensorTask,
    "SensorTask",
    STACK_SIZE,
    NULL,
    2, // Вища пріоритетність, читаємо сенсори
    &sensorTaskHandle
  );
  
  xTaskCreate(
    outputTask,
    "OutputTask",
    STACK_SIZE,
    NULL,
    1, // Низька пріоритетність, бо виводи не критичні
    &outputTaskHandle
  );
  
  xTaskCreate(
    motorControlTask,
    "MotorControlTask",
    STACK_SIZE,
    NULL,
    2, 
    &motorControlTaskHandle
  );

  xTaskCreate(
    imuTask,
    "IMU_Task",
    STACK_SIZE,
    NULL,  // Pass pointer to the sensor object
    2,  // Priority
    &imuTaskHandle
  );

  xTaskCreate(
    temperatureTask,
    "TempTask",
    STACK_SIZE,
    NULL,
    2,  // Lower priority
    &tempTaskHandle
  );


  #if DEBUG_ENABLE
  xTaskCreate(
    debugTask,
    "DebugTask",
    STACK_SIZE,
    NULL,
    1,  // Найнижча пріоритетність, бо дебаг не критичний
    &debugTaskHandle
  );
  #endif 
  // Запуск диспетчера FreeRTOS
  vTaskStartScheduler();
}

void loop() {
}



#if DEBUG_ENABLE
void debugTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500);
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    // Process regular status info
    if (xSemaphoreTake(modbusRegisterMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      Serial.println("------- DEBUG INFO -------");
      Serial.print("STATUS: MB Addr="); Serial.print(modbusSlaveAddress);
      Serial.print(" BulletCount="); Serial.println(bulletCounter);
      
      Serial.print("Reload Sensor="); Serial.print(reloadSensor); 
      Serial.print("/"); Serial.print(lastReloadSensor);
      Serial.print(" Reload Signal="); Serial.print(reloadSignal);
      Serial.print("/"); Serial.print(lastReloadSignal); 
      Serial.print(" Counter Sensor="); Serial.print(counterSensor);
      Serial.print("/"); Serial.println(lastCounterSensor);
      
      Serial.print("D INPUTS: ");
      for (int i = 0; i < NUM_DISCRETE_INPUTS; i++) {
        Serial.print(String(discreteInputs[i] ? "1, " : "0, "));
      }
      Serial.println();
      
      Serial.print("COILS: ");
      for (int i = 0; i < NUM_COILS; i++) {
        Serial.print(String(coils[i] ? "1, " : "0, "));
      }
      Serial.println();

      Serial.print("HOLDING REGISTERS: ");
      for (int i = 0; i < NUM_HOLDING_REGISTERS; i++) {
        Serial.print(holdingRegisters[i]);
        Serial.print(", ");
      }
      Serial.println();
      Serial.print("INPUT REGISTERS: ");
      for (int i = 0; i < NUM_INPUT_REGISTERS; i++) {
        Serial.print(inputRegisters[i]);
        Serial.print(", ");
      }
      Serial.println();
      
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    // Process custom messages from queue
    if (oneShotDebugMsgValid) {
        Serial.println(oneShotDebugMsg);
        oneShotDebugMsgValid = false;
    }
    
    Serial.println("-------------------------");
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
#endif

// Окрема таска для Modbus
void modbusTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5; // 5ms (200Hz)
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    modbus.poll();
    
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Таска на читання сенсорів, оновлення входів та лічильника
void sensorTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10; // 10ms (100Hz)
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
      // Читання всіх сенсорів і зразу запис в дискретні входи
      for (int i = 0; i < NUM_DISCRETE_INPUTS; i++) {
        discreteInputs[i] = digitalRead(sensorPins[i]);
      }
      
      // Сенсор підрахунку патронів
      counterSensor = discreteInputs[2];
      if (counterSensor != lastCounterSensor) {
        if (counterSensor == LOW) {
          bulletCounter++; // Falling edge
        }
      }
      lastCounterSensor = counterSensor;
      
      // Сенсор перезарядки
      reloadSensor = discreteInputs[0];
      if (reloadSensor != lastReloadSensor) {
        if (reloadSensor == LOW) {
          // Reset bullet counter on reload
          bulletCounter = 0;
        }
      }
      lastReloadSensor = reloadSensor;
      
      holdingRegisters[0] = bulletCounter;
      
      // If holding register is manually cleared, reset counter
      if (holdingRegisters[0] == 0) {
        bulletCounter = 0;
      }
      
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Таска для управління виходами
void outputTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20; // 20ms (50Hz)
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
      analogWrite(ledPins[1], holdingRegisters[0]);
      analogWrite(ledPins[2], holdingRegisters[1]);
      analogWrite(ledPins[3], holdingRegisters[2]);
      analogWrite(outPins[1], holdingRegisters[3]);
      digitalWrite(outPins[0], coils[0]); // M2 спуск
      digitalWrite(ledPins[0], coils[1]); // Перезарядка
      
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Комунікація з драйвером до моторів
void motorControlTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50; // 50ms (20Hz)
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
      // Сигнал перезарядки
      reloadSignal = coils[1];
      if (reloadSignal != lastReloadSignal) {
        if (reloadSignal == HIGH && reloadSensor == LOW) {
          xSemaphoreGive(modbusRegisterMutex); // Звільняємо м'ютекс, бо serial.print блокуючий
          
          Serial2.println("w axis0.requested_state 8");
          Serial2.println("v 0 -100");
          vTaskDelay(pdMS_TO_TICKS(3000)); // Чекаємо поки драйвер відригне
          
          if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) { // Забираємо м'ютекс назад
            lastReloadSignal = reloadSignal;
            xSemaphoreGive(modbusRegisterMutex);
          }
          continue;
        }
      }
      
      // Сенсор перезардки
      reloadSensor = discreteInputs[0];
      if (reloadSensor != lastReloadSensor) {
        if (reloadSensor == LOW) {
          xSemaphoreGive(modbusRegisterMutex); 
          
          Serial2.println("w axis0.requested_state 1");
          
          if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
            lastReloadSensor = reloadSensor;
            xSemaphoreGive(modbusRegisterMutex);
          }
          continue;
        }
      }
      
      lastReloadSignal = reloadSignal;
      lastReloadSensor = reloadSensor;
      
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// IMU sensor task for STEVAL-MKI245KA (ISM330BX)
void imuTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz sampling
    int32_t acceleration[3], angularRate[3], gravityVector[3];
    
    // Wait a bit for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    #if DEBUG_ENABLE
    Serial.println("Initializing ISM330BX IMU on STEVAL-MKI245KA board...");
    #endif
    
    // Initialize the sensor with custom ISM330BX driver
    if (imu.begin() != ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Error initializing ISM330BX");
        #endif
        delay(1000); // Wait and try again
        if (imu.begin() != ISM330BX_STATUS_OK) {
            #if DEBUG_ENABLE
            Serial.println("Second attempt failed, aborting IMU task");
            #endif
            vTaskDelete(NULL);
        }
    }
    
    imu.enableGyroscope();
    imu.enableAccelerometer();
    imu.enableSensorFusion(); // Enable sensor fusion for gravity vector
    
    #if DEBUG_ENABLE
    Serial.println("ISM330BX initialization complete!");
    #endif
    
    // Start the main processing loop
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;) {
        // Read sensor data when available
        if (imu.checkDataReady()) {
            imu.readGyroscope(angularRate);
            
            // Read gravity vector when available
            if (imu.checkGravityDataReady()) {
                imu.readGravityVector(gravityVector);
                #if DEBUG_ENABLE
                sendOneShotDebugMessage("Gravity Vector: X=%ld, Y=%ld, Z=%ld (mg)",
                                        gravityVector[0],
                                        gravityVector[1],
                                        gravityVector[2]);
                #endif
             if (coils[4]) {
                  // Capture current orientation as the new reference
                  if (imu.setGravityReference()) {
                      imu.applyGravityReference(gravityVector);

                      #if DEBUG_ENABLE
                      Serial.println("Gravity vector reset to zero");
                      #endif
                  } else {
                      #if DEBUG_ENABLE
                      Serial.println("Failed to set gravity reference");
                      #endif
                  }
                  coils[4] = false;
              }

            } else {
                #if DEBUG_ENABLE
                sendOneShotDebugMessage("Gravity data not ready");
                #endif
                
            }
                
            // Update Modbus registers with the data
            if (xSemaphoreTake(modbusRegisterMutex, pdMS_TO_TICKS(5)) == pdTRUE) {

                holdingRegisters[2] = gravityVector[0];
                holdingRegisters[3] = gravityVector[1];
                holdingRegisters[4] = gravityVector[2];
                
                
                xSemaphoreGive(modbusRegisterMutex);
            }
        }
        
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task to read temperature from MAX6675 thermocouple
void temperatureTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Read temperature once per second
  float temperature = 0.0;
  
  // Wait for the MAX6675 to stabilize
  vTaskDelay(pdMS_TO_TICKS(500));
  
  #if DEBUG_ENABLE
  Serial.println("Temperature sensor task started");
  #endif
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    // Read the temperature
    temperature = thermocouple.readCelsius();
    
    #if DEBUG_ENABLE
    sendOneShotDebugMessage("Temperature: %.2f°C", temperature);
    #endif
    
    // Update the Modbus holding register
    if (xSemaphoreTake(modbusRegisterMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      // Store temperature as an integer (e.g., 25.4°C becomes 254)
      holdingRegisters[1] = (uint16_t)(temperature * 10);
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}