#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#define STACK_SIZE 256 // Розмір стека для тасок FreeRTOS
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

// Modbus змінні
const uint8_t NUM_COILS = 4;
const uint8_t NUM_DISCRETE_INPUTS = 4;
const uint8_t NUM_HOLDING_REGISTERS = 4;
const uint8_t NUM_INPUT_REGISTERS = 4;

uint8_t modbusAddress = 3; // Адреса Modbus слейва
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

// М'ютекс, щоб не було конфліктів при доступі до масивів Modbus між тасками
SemaphoreHandle_t modbusRegisterMutex; 

// Хендлери і декларації тасок
TaskHandle_t modbusTaskHandle;
TaskHandle_t sensorTaskHandle;
TaskHandle_t outputTaskHandle;
TaskHandle_t motorControlTaskHandle;

void modbusTask(void *pvParameters);
void sensorTask(void *pvParameters);
void outputTask(void *pvParameters);
void motorControlTask(void *pvParameters);

#if DEBUG_ENABLE
TaskHandle_t debugTaskHandle;
void debugTask(void *pvParameters);
#endif

void setup() {
  #if DEBUG_ENABLE
  Serial.begin(115200); // Основний серіал по юсб, для дебагу
  #endif
  Serial1.begin(MODBUS_BAUD, MODBUS_CONFIG); // Modbus RTU
  Serial2.begin(115200, SERIAL_8N1); // Драйвер для моторів
  
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
    bitWrite(modbusAddress, i, address[i]);
  }

  // Конфігурація Modbus
  modbus.configureCoils(coils, NUM_COILS);
  modbus.configureDiscreteInputs(discreteInputs, NUM_DISCRETE_INPUTS);
  modbus.configureHoldingRegisters(holdingRegisters, NUM_HOLDING_REGISTERS);
  modbus.configureInputRegisters(inputRegisters, NUM_INPUT_REGISTERS);

  // Ініціалізація Modbus з адресою
  modbus.begin(modbusAddress, MODBUS_BAUD, MODBUS_CONFIG);
  
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
    1, // Низька пріоритетність, бо виводи не критичні (TODO: вивести М2 постріл на окрему таску з вищим пріоритетом)
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

  #if DEBUG_ENABLE
  xTaskCreate(
    debugTask,
    "DebugTask",
    STACK_SIZE,
    NULL,
    1,  // Найнижча пріоритетність, бо дебаг не критичний
    NULL
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
      Serial.println("STATUS: MB Addr=" + String(modbusAddress) + " BulletCount=" + String(bulletCounter));
      Serial.println("Reload Sensor=" + String(reloadSensor) + "/" + String(lastReloadSensor) + 
                    " Reload Signal=" + String(reloadSignal) + "/" + String(lastReloadSignal) + 
                    " Counter Sensor=" + String(counterSensor) + "/" + String(lastCounterSensor));
    
      Serial.print("D INPUTS: ");
      for (int i = 0; i < NUM_DISCRETE_INPUTS; i++) {
        Serial.print(String(discreteInputs[i] ? "1, " : "0, "));
      }
      Serial.println();
      
      Serial.print("COILS:  ");
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
      

      counterSensor = discreteInputs[2];
      if (counterSensor != lastCounterSensor) {
        if (counterSensor == LOW) {
          bulletCounter++; // Falling edge
        }
      }
      lastCounterSensor = counterSensor;

      holdingRegisters[0] = bulletCounter;
      
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