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

#define STACK_SIZE 256 // Розмір стека для тасок FreeRTOS
#define MODBUS_BAUD 115200
#define MODBUS_CONFIG SERIAL_8N1 
#define DEBUG_ENABLE 1 // Дебаг в серіал. Якщо поставити 0, то все пов'язане з дебагом не компілюється

// Піни
const int16_t sensorPins[4] = {PC3, PC2, PC1, PC0}; // Інпути сенсорів (кінцевик, тд)
const int16_t outPins[4] = {PC13, PA0, PC15, PC14}; // Виводи для управління (поки шо просто леди)
const int16_t ledPins[4] = {PB12, PB13, PB14, PB15}; // Діоди індикації
const int16_t dePin = PA8;                          // Пін DE для RS-485
const int16_t DIP_Pins[4] = {PC10, PC11, PC12, PD2}; // DIP світчі на платі для адресації цього слейва Modbus 
const int thermoSCK = PA5;  // SCK pin термопари
const int thermoCS = PA4;   // CS pin термопари
const int thermoMISO = PA6;   // MISO pin термопари

// Серіал для Modbus 
HardwareSerial Serial1(USART1);

// Ініціалізація Modbus RTU як слейв, по Serial1, з DE піном
ModbusRTUSlave modbus(Serial1, dePin);

// I2C для IMU сенсора
TwoWire i2c(PB7, PB6);
ISM330BXSensor imu(&i2c, 0x6A); //  SD0 приєднаний до GND, щоб адреса була 0x6A

// Ініціалізація термопари MAX6675
MAX6675 thermocouple(thermoSCK, thermoCS, thermoMISO);

// Modbus змінні
const uint8_t NUM_COILS = 3; // 0 - M2 спуск, 1 - перезарядка, 2 - нуль. позиц. для гравітаційного вектора
const uint8_t NUM_DISCRETE_INPUTS = 4; 
const uint8_t NUM_HOLDING_REGISTERS = 5; // 0 - лічильник куль, 1 - температура, 2 - гравітаційний вектор X, 3 - Y, 4 - Z
const uint8_t NUM_INPUT_REGISTERS = 6;
bool coils[NUM_COILS]; // Масив котушок
bool discreteInputs[NUM_DISCRETE_INPUTS]; // Масив дискретних входів
uint16_t holdingRegisters[NUM_HOLDING_REGISTERS]; // Масив регістрів
uint16_t inputRegisters[NUM_INPUT_REGISTERS];     // Масив регістрів вводу
uint8_t modbusSlaveAddress = 3; // Адреса цього слейва за замовчуванням

bool DIP_modbus_swtiches[4]; // Окремі біти адреси Modbus

// Сенсори та їх стани
int reloadSensor = 0; // Стан перезарядки
int lastReloadSensor = 0; // Попередній стан, треба щоб детектити falling edge
int reloadSignal = 0; // Сигнал перезарядки
int lastReloadSignal = 0;
int counterSensor = 0; // Стан сенсора лічильника
int lastCounterSensor = 0;
uint16_t bulletCounter = 0; // Лічильник куль


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

// М'ютекс, щоб не було конфліктів при доступі до масивів Modbus між тасками
SemaphoreHandle_t modbusRegisterMutex; 

// Компілюємо дебаг таску тільки якщо DEBUG_ENABLE == 1
#if DEBUG_ENABLE
TaskHandle_t debugTaskHandle;
void debugTask(void *pvParameters);
#endif

void setup() {
  #if DEBUG_ENABLE
  Serial.begin(115200); // Основний серіал по юсб, для дебагу
  while(!Serial) { 
    delay(10); // Нічого не робимо, поки хтось не під'єднається до серіалу, щоб не пропустити ніяку важливу інфу
  } 
  delay(100);
  #endif
  
  Serial1.begin(MODBUS_BAUD, MODBUS_CONFIG); // Серіал на Modbus RTU
  Serial2.begin(115200, SERIAL_8N1); // Серіал для драйвера моторів

  // I2C для IMU
  i2c.begin();
  i2c.setClock(400000);
  delay(500); // Затримка для стабілізації I2C, бо може бути, що IMU ще не готовий

  // RS-485 driver enable pin
  pinMode(dePin, OUTPUT);

  // DIP перемикачі для адресації Modbus (4 біти), підтягнуті до VCC
  for (int i = 0; i < 4; i++) {
    pinMode(DIP_Pins[i], INPUT_PULLUP);
  }
  
  // Сенсори, підтягнуті до VCC
  for (int i = 0; i < 4; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }
  
  // Виводи для керування та індикації
  for (int i = 0; i < 4; i++) {
    pinMode(outPins[i], OUTPUT);
    pinMode(ledPins[i], OUTPUT);
  }
  
  // Задаємо адресу Modbus з DIP перемикачів (перевернуті, бо підтягнуті до VCC)
  for (int i = 0; i < 4; i++) {
    DIP_modbus_swtiches[i] = !digitalRead(DIP_Pins[i]);
    bitWrite(modbusSlaveAddress, i, DIP_modbus_swtiches[i]);
  }

  // Конфігурація Modbus
  modbus.configureCoils(coils, NUM_COILS);
  modbus.configureDiscreteInputs(discreteInputs, NUM_DISCRETE_INPUTS);
  modbus.configureHoldingRegisters(holdingRegisters, NUM_HOLDING_REGISTERS);
  modbus.configureInputRegisters(inputRegisters, NUM_INPUT_REGISTERS);

  // Ініціалізація Modbus з адресою
  modbus.begin(modbusSlaveAddress, MODBUS_BAUD, MODBUS_CONFIG);
  
  // Створення м'ютексу для захисту масивів Modbus, щоб не було конфліктів між тасками
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
    1, // Низька пріоритетність, бо виводи не критичні (поки що)
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
    NULL,
    2,
    &imuTaskHandle
  );

  xTaskCreate(
    temperatureTask,
    "TempTask",
    STACK_SIZE,
    NULL,
    2,
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


// Дебаг таска. Виводить в серіал інформацію про стан всіх змінних Modbus, сенсорів, лічильника куль і тд.
// Заради оптимізації - не компілюємо в продакшн (DEBUG_ENABLE = 0), бо вивід в серіал займає багацько часу
#if DEBUG_ENABLE
void debugTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // Раз в 500ms (2Hz), просто нема потреби частіше
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
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
        Serial.print((int16_t)holdingRegisters[i]);
        Serial.print(", ");
      }
      Serial.println();
      Serial.print("INPUT REGISTERS: ");
      for (int i = 0; i < NUM_INPUT_REGISTERS; i++) {
        Serial.print(inputRegisters[i]);
        Serial.print(", ");
      }
      Serial.println();
      Serial.print("Temperature: ");
      Serial.print(holdingRegisters[1]/10.0);
      Serial.print("°C");
      Serial.println();

      Serial.print("Gravity Vector: X=");
      Serial.print((int16_t)holdingRegisters[2]);
      Serial.print(", Y=");
      Serial.print((int16_t)holdingRegisters[3]);
      Serial.print(", Z=");
      Serial.print((int16_t)holdingRegisters[4]);
      Serial.println(" (mg)");

            
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    Serial.println("-------------------------");
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
#endif

// Окрема таска для Modbus
void modbusTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5;
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    modbus.poll();
    
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Таска на читання сенсорів, оновлення входів та лічильника. Зразу записує в Modbus.
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
          // При перезарядці скидаємо лічильник куль
          bulletCounter = 0;
        }
      }
      lastReloadSensor = reloadSensor;
      
      holdingRegisters[0] = bulletCounter;
      
      // Якщо регістр скинуто, то скидаємо й лічильник
      if (holdingRegisters[0] == 0) {
        bulletCounter = 0;
      }
      
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Таска для управління виводами
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

// Таска для комунікації з драйвером моторів
void motorControlTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10;
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    // М'ютекс для доступу до масивів Modbus, шоб нич не попердоляло
    if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
      // Сигнал перезарядки
      reloadSignal = coils[1];
      if (reloadSignal != lastReloadSignal) {
        if (reloadSignal == HIGH && reloadSensor == LOW) {
          xSemaphoreGive(modbusRegisterMutex); // Звільняємо м'ютекс
          
          Serial2.println("w axis0.requested_state 8");
          Serial2.println("v 0 -100");
          vTaskDelay(pdMS_TO_TICKS(3000)); // Чекаємо поки драйвер відригне
          
          // Забираємо м'ютекс назад, щоб записати стан сигналу перезарядки
          if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
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
          xSemaphoreGive(modbusRegisterMutex);  // Звільняємо м'ютекс як тільки знаємо, що перезарядка почалась
          
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

// Таска для IMU STEVAL-MKI245KA (на ISM330BX)
void imuTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    int32_t gravityVector[3];
    
    #if DEBUG_ENABLE
    Serial.println("[IMU] Initializing ISM330BX IMU on STEVAL-MKI245KA board...");
    #endif
    
    // Спроба Ініціалізації IMU
    if (imu.begin() != ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("[IMU] Error initializing ISM330BX");
        #endif
        // Спробувати ще через секунду (може бути шо i2c ше не стабілізувався, тому треба тріха подрихнути)
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        if (imu.begin() != ISM330BX_STATUS_OK) {
            #if DEBUG_ENABLE
            Serial.println("[IMU] Second attempt failed, aborting IMU task");
            #endif
            // Якщо не вдалося ініціалізувати IMU, то завершуємо таску
            vTaskDelete(NULL);
        }
    }
    
    imu.enableGyroscope();
    imu.enableAccelerometer();
    imu.enableSensorFusion(); // Для гравітаційного вектора
    
    #if DEBUG_ENABLE
    Serial.println("[IMU] ISM330BX initialization complete!");
    #endif
    

    xLastWakeTime = xTaskGetTickCount();    
    for(;;) {
        // Читаєм дані з IMU тільки коли є нові
        if (imu.checkDataReady()) {
            
            // Читаємо гравітаційний вектор тільки коли він готовий
            if (imu.checkGravityDataReady()) {
                imu.readGravityVector(gravityVector);
             if (coils[2]) {
                  // Якщо подали одиничку на цю катушку, значить треба поставити цей гравітаційний вектор як нульовий
                  if (imu.setGravityReference()) { // Скидаємо вектор гравітації
                      imu.applyGravityReference(gravityVector); // Зразу встановлюємо новий вектор (не чекаємо на наступний цикл)
                      #if DEBUG_ENABLE
                      Serial.println("[IMU] Gravity vector reset to zero");
                      #endif
                  } else {
                      #if DEBUG_ENABLE
                      Serial.println("[IMU] Failed to set gravity reference");
                      #endif
                  }
                  coils[2] = false;
              }

            } else {
                #if DEBUG_ENABLE
                Serial.println("[IMU] Gravity data not ready");
                #endif
                
            }
                
            // Оновлюємо регістри в Modbus
            if (xSemaphoreTake(modbusRegisterMutex, pdMS_TO_TICKS(5)) == pdTRUE) {

                holdingRegisters[2] = gravityVector[0];
                holdingRegisters[3] = gravityVector[1];
                holdingRegisters[4] = gravityVector[2];
                
                
                xSemaphoreGive(modbusRegisterMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Таска для термопари MAX6675
void temperatureTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(250); // 250ms (4Hz) Більше не треба,
  // бо термопара не так швидко реагує + тупо вмирає, якщо оптиувати частіше
  float temperature = 0.0;
  
  // Затримка для стабілізації термопари перед підключенням (відбувається тільки 1 раз)
  vTaskDelay(pdMS_TO_TICKS(500));
  
  #if DEBUG_ENABLE
  Serial.println("[TEMP] Initializing MAX6675 thermocouple...");
  #endif
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    // Читаємо температуру з термопари
    temperature = thermocouple.readCelsius();
    
    // Оновлюємо регістри в Modbus
    if (xSemaphoreTake(modbusRegisterMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      // Зберігаємо температуру в регістр як ціле число (наприклад, 25.4°C * 10 стає 254)
      holdingRegisters[1] = (uint16_t)(temperature * 10);

      xSemaphoreGive(modbusRegisterMutex);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}