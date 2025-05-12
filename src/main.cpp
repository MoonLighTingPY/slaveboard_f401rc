#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <Wire.h>
#include <ISM330DHCXSensor.h>

#define STACK_SIZE 512 // Increased stack size for tasks
#define DEBUG_ENABLE 1 // Дебаг в серіал. Якщо поставити 0, то все пов'язане з дебагом не компілюється

#define MODBUS_BAUD 115200
#define MODBUS_CONFIG SERIAL_8N1

// Піни
const int16_t sensorPins[4] = {PC3, PC2, PC1, PC0}; // Інпути сенсорів
const int16_t outPins[4] = {PC13, PA0, PC15, PC14}; // Виводи для управління
const int16_t ledPins[4] = {PB12, PB13, PB14, PB15}; // Діоди індикації
const int16_t dePin = PA8;                          // Пін DE для RS-485
const int16_t addressPins[4] = {PC10, PC11, PC12, PD2}; // DIP світчі для адресації Modbus

// Серіал для Modbus
HardwareSerial Serial1(USART1);


// Ініціалізація Modbus RTU як слейв, по Serial1, з DE піном
ModbusRTUSlave modbus(Serial1, dePin);

// I2C for IMU sensor - using PB7(SDA) and PB6(SCL)
TwoWire i2c(PB7, PB6);
// We'll use the ISM330DHCX driver but for ISM330BX sensor
ISM330DHCXSensor AccGyr(&i2c, 0x6B); // Default I2C address - we'll try to auto-detect

// Modbus змінні
const uint8_t NUM_COILS = 4;
const uint8_t NUM_DISCRETE_INPUTS = 4;
const uint8_t NUM_HOLDING_REGISTERS = 4;
const uint8_t NUM_INPUT_REGISTERS = 3;

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

void modbusTask(void *pvParameters);
void sensorTask(void *pvParameters);
void outputTask(void *pvParameters);
void motorControlTask(void *pvParameters);
void imuTask(void *pvParameters);

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
  i2c.setClock(400000); // 400kHz for faster communication

  #if DEBUG_ENABLE
  // I2C scanner - find the IMU
  Serial.println("Scanning I2C bus for IMU sensor...");
  byte error, addressI2C;
  int deviceCount = 0;

  for(addressI2C = 1; addressI2C < 127; addressI2C++) {
    i2c.beginTransmission(addressI2C);
    error = i2c.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (addressI2C < 16) Serial.print("0");
      Serial.print(addressI2C, HEX);
      
      // Check if this is our IMU (ISM330DHCX/ISM330BX has address 0x6A or 0x6B)
      if (addressI2C == 0x6A || addressI2C == 0x6B) {
        Serial.print(" (ISM330DHCX IMU)");
        imuAddress = addressI2C;
        imuFound = true;
      }
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
  #else
  // In production, try common IMU addresses
  i2c.beginTransmission(0x6A);
  if (i2c.endTransmission() == 0) {
    imuAddress = 0x6A;
    imuFound = true;
  } else {
    i2c.beginTransmission(0x6B);
    if (i2c.endTransmission() == 0) {
      imuAddress = 0x6B;
      imuFound = true;
    }
  }
  #endif

  // If IMU was found, update AccGyr with the found address
  if (imuFound && imuAddress != 0) {
    AccGyr = ISM330DHCXSensor(&i2c, imuAddress);
  }

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
    3, // Найвища пріоритетність, бо Modbus повинен бути завжди активний
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
    "IMUTask",
    STACK_SIZE * 2, // Needs more stack
    NULL,
    2,
    &imuTaskHandle
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
    if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
      Serial.println("STATUS: MB Addr=" + String(modbusSlaveAddress) + " BulletCount=" + String(bulletCounter));
      Serial.println("Reload Sensor=" + String(reloadSensor) + "/" + String(lastReloadSensor) + 
                    " Reload Signal=" + String(reloadSignal) + "/" + String(lastReloadSignal) + 
                    " Counter Sensor=" + String(counterSensor) + "/" + String(lastCounterSensor));
    
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
      Serial.println("----"); 

      xSemaphoreGive(modbusRegisterMutex);
    }
    
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
    #if DEBUG_ENABLE
    Serial.println("IMU task started");
    #endif
    
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz sampling
    int32_t acceleration[3], angularRate[3];
    uint8_t status = 0;
    
    // Wait a bit for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    #if DEBUG_ENABLE
    Serial.println("Initializing ISM330BX IMU on STEVAL-MKI245KA board...");
    #endif
    
    // ——————————————————————————————————————————————————
    // manual init for STEVAL-MKI245KA (ISM330BX)
    // ——————————————————————————————————————————————————
    
    // 1) sw-reset + wait
    AccGyr.WriteReg(ISM330DHCX_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 2) enable auto-increment + block data update
    //    CTRL3_C: IF_INC (bit2)=1, BDU (bit6)=1
    AccGyr.WriteReg(ISM330DHCX_CTRL3_C, (1<<6)|(1<<2));
    
    // 3) set accel to 104 Hz, ±2 g
    //    CTRL1_XL: ODR_XL[7:4]=0100b (104 Hz), FS_XL[3:2]=00b (±2 g)
    AccGyr.WriteReg(ISM330DHCX_CTRL1_XL, (0x4<<4)|(0x0<<2));
    
    // 4) set gyro to 104 Hz, ±2000 dps
    //    CTRL2_G: ODR_G[7:4]=0100b (104 Hz), FS_G[3:2]=11b (2000 dps)
    AccGyr.WriteReg(ISM330DHCX_CTRL2_G, (0x4<<4)|(0x3<<2));
    
    #if DEBUG_ENABLE
    // Read and verify the configuration
    uint8_t ctrl1_xl = 0, ctrl2_g = 0, ctrl3_c = 0;
    AccGyr.ReadReg(ISM330DHCX_CTRL1_XL, &ctrl1_xl);
    AccGyr.ReadReg(ISM330DHCX_CTRL2_G, &ctrl2_g);
    AccGyr.ReadReg(ISM330DHCX_CTRL3_C, &ctrl3_c);
    
    Serial.println("IMU manual setup done!");
    Serial.print("CTRL3_C: 0x"); Serial.println(ctrl3_c, HEX);
    Serial.print("CTRL1_XL: 0x"); Serial.println(ctrl1_xl, HEX);
    Serial.print("CTRL2_G: 0x"); Serial.println(ctrl2_g, HEX);
    #endif
    
    // Start the main processing loop
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;) {
        bool new_data = false;
        uint8_t status_reg = 0;
        
        // Read STATUS_REG to check if new data is available
        AccGyr.ReadReg(ISM330DHCX_STATUS_REG, &status_reg);
        
        bool acc_data_available = (status_reg & 0x01) != 0;  // XLDA bit
        bool gyro_data_available = (status_reg & 0x02) != 0; // GDA bit
        
        // Read accelerometer data if available
        if (acc_data_available) {
            uint8_t raw_data[6];
            // Read all 6 bytes in sequence (X,Y,Z low and high bytes)
            AccGyr.ReadReg(ISM330DHCX_OUTX_L_A, &raw_data[0]);
            AccGyr.ReadReg(ISM330DHCX_OUTX_H_A, &raw_data[1]);
            AccGyr.ReadReg(ISM330DHCX_OUTY_L_A, &raw_data[2]);
            AccGyr.ReadReg(ISM330DHCX_OUTY_H_A, &raw_data[3]);
            AccGyr.ReadReg(ISM330DHCX_OUTZ_L_A, &raw_data[4]);
            AccGyr.ReadReg(ISM330DHCX_OUTZ_H_A, &raw_data[5]);
            
            // Combine bytes and convert to mg (scale factor for 2g mode = 0.061 mg/LSB)
            int16_t acc_x = (raw_data[1] << 8) | raw_data[0];
            int16_t acc_y = (raw_data[3] << 8) | raw_data[2];
            int16_t acc_z = (raw_data[5] << 8) | raw_data[4];
            
            acceleration[0] = (int32_t)(acc_x * 0.061f);
            acceleration[1] = (int32_t)(acc_y * 0.061f);
            acceleration[2] = (int32_t)(acc_z * 0.061f);
            
            new_data = true;
        }
        
        // Read gyroscope data if available
        if (gyro_data_available) {
            uint8_t raw_data[6];
            // Read all 6 bytes in sequence (X,Y,Z low and high bytes)
            AccGyr.ReadReg(ISM330DHCX_OUTX_L_G, &raw_data[0]);
            AccGyr.ReadReg(ISM330DHCX_OUTX_H_G, &raw_data[1]);
            AccGyr.ReadReg(ISM330DHCX_OUTY_L_G, &raw_data[2]);
            AccGyr.ReadReg(ISM330DHCX_OUTY_H_G, &raw_data[3]);
            AccGyr.ReadReg(ISM330DHCX_OUTZ_L_G, &raw_data[4]);
            AccGyr.ReadReg(ISM330DHCX_OUTZ_H_G, &raw_data[5]);
            
            // Combine bytes and convert to mdps (scale factor for 2000dps mode = 70 mdps/LSB)
            int16_t gyro_x = (raw_data[1] << 8) | raw_data[0];
            int16_t gyro_y = (raw_data[3] << 8) | raw_data[2];
            int16_t gyro_z = (raw_data[5] << 8) | raw_data[4];
            
            angularRate[0] = (int32_t)(gyro_x * 70.0f);
            angularRate[1] = (int32_t)(gyro_y * 70.0f);
            angularRate[2] = (int32_t)(gyro_z * 70.0f);
            
            new_data = true;
        }
        
        // Only update Modbus registers and print debug if we have new data
        if (new_data && xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
            // Store in input registers for Modbus access
            // Use absolute values and limit to 16-bit range
            inputRegisters[0] = abs(acceleration[0]) > 32767 ? 32767 : abs(acceleration[0]);
            inputRegisters[1] = abs(acceleration[1]) > 32767 ? 32767 : abs(acceleration[1]);
            inputRegisters[2] = abs(acceleration[2]) > 32767 ? 32767 : abs(acceleration[2]);
            inputRegisters[3] = abs(angularRate[0]) > 32767 ? 32767 : abs(angularRate[0]);
            inputRegisters[4] = abs(angularRate[1]) > 32767 ? 32767 : abs(angularRate[1]);
            inputRegisters[5] = abs(angularRate[2]) > 32767 ? 32767 : abs(angularRate[2]);
            
            xSemaphoreGive(modbusRegisterMutex);
            
            #if DEBUG_ENABLE
            static uint32_t debug_counter = 0;
            if (debug_counter++ % 50 == 0) { // Print every 50 cycles to avoid flooding
                Serial.print("ACC status: "); Serial.print(acc_data_available);
                Serial.print(", GYRO status: "); Serial.print(gyro_data_available);
                Serial.print(" | Accel X: "); Serial.print(acceleration[0]);
                Serial.print(" Y: "); Serial.print(acceleration[1]);
                Serial.print(" Z: "); Serial.print(acceleration[2]);
                Serial.print(" | Gyro X: "); Serial.print(angularRate[0]);
                Serial.print(" Y: "); Serial.print(angularRate[1]);
                Serial.print(" Z: "); Serial.println(angularRate[2]);
            }
            #endif
        }
        
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}