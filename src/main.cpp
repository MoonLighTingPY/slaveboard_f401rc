#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

// Configuration constants
#define MODBUS_BAUD 115200
#define MODBUS_CONFIG SERIAL_8N1
#define STACK_SIZE 256 // Stack size for tasks


// Configure hardware serial for Modbus communication
HardwareSerial Serial1(USART1);

// Pin definitions with meaningful names
const int16_t sensorPins[4] = {PC3, PC2, PC1, PC0};    // Input sensors
const int16_t outPins[4] = {PC13, PA0, PC15, PC14};    // Output control pins
const int16_t ledPins[4] = {PB12, PB13, PB14, PB15};   // LED indicators
const int16_t dePin = PA8;                             // RS-485 driver enable pin
const int16_t addressPins[4] = {PC10, PC11, PC12, PD2}; // Modbus address DIP switches

// Initialize Modbus RTU slave with serial port and driver enable pin
ModbusRTUSlave modbus(Serial1, dePin);

// Modbus register configuration
const uint8_t NUM_COILS = 4;              // Number of coils (digital outputs)
const uint8_t NUM_DISCRETE_INPUTS = 4;    // Number of discrete inputs (digital inputs)
const uint8_t NUM_HOLDING_REGISTERS = 4;  // Number of holding registers (writable registers)
const uint8_t NUM_INPUT_REGISTERS = 4;    // Number of input registers (read-only registers)

// Modbus variables
uint8_t modbusAddress = 3;                // Default Modbus device address
bool coils[NUM_COILS];                    // Storage for coil states
bool discreteInputs[NUM_DISCRETE_INPUTS]; // Storage for discrete input states
uint16_t holdingRegisters[NUM_HOLDING_REGISTERS]; // Storage for holding register values
uint16_t inputRegisters[NUM_INPUT_REGISTERS];     // Storage for input register values

// Address configuration
bool address[4]; // Individual address bits from DIP switches

// Sensor and control state tracking
int reloadSensor = 0;       // Current state of reload sensor
int lastReloadSensor = 0;   // Previous state of reload sensor
int reloadSignal = 0;       // Current state of reload signal
int lastReloadSignal = 0;   // Previous state of reload signal
uint16_t bulletCounter = 0; // Counter for bullets fired
int counterSensor = 0;      // Current state of counter sensor
int lastCounterSensor = 0;  // Previous state of counter sensor

// Mutex for protecting shared resources
SemaphoreHandle_t modbusRegisterMutex;

// Task handles
TaskHandle_t modbusTaskHandle;
TaskHandle_t sensorTaskHandle;
TaskHandle_t outputTaskHandle;
TaskHandle_t motorControlTaskHandle;

// Task functions declaration
void modbusTask(void *pvParameters);
void sensorTask(void *pvParameters);
void outputTask(void *pvParameters);
void motorControlTask(void *pvParameters);

void setup() {
  // Initialize serial communications
  Serial.begin(115200);             // Debug serial
  Serial1.begin(MODBUS_BAUD, MODBUS_CONFIG); // Modbus RTU communication
  Serial2.begin(115200, SERIAL_8N1); // Motor controller communication
  
  // Configure RS-485 driver enable pin
  pinMode(dePin, OUTPUT);

  // Configure DIP switch inputs for Modbus addressing (with pull-ups)
  for (int i = 0; i < 4; i++) {
    pinMode(addressPins[i], INPUT_PULLUP);
  }
  
  // Configure sensor inputs (with pull-ups)
  for (int i = 0; i < 4; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }
  
  // Configure output pins
  for (int i = 0; i < 4; i++) {
    pinMode(outPins[i], OUTPUT);
    pinMode(ledPins[i], OUTPUT);
  }

  // Read address DIP switches (inverted because of pull-ups)
  for (int i = 0; i < 4; i++) {
    address[i] = !digitalRead(addressPins[i]);
    bitWrite(modbusAddress, i, address[i]);
  }

  // Configure Modbus registers
  modbus.configureCoils(coils, NUM_COILS);
  modbus.configureDiscreteInputs(discreteInputs, NUM_DISCRETE_INPUTS);
  modbus.configureHoldingRegisters(holdingRegisters, NUM_HOLDING_REGISTERS);
  modbus.configureInputRegisters(inputRegisters, NUM_INPUT_REGISTERS);

  // Initialize Modbus communication
  modbus.begin(modbusAddress, MODBUS_BAUD, MODBUS_CONFIG);
  
  // Create a mutex to protect shared resources
  modbusRegisterMutex = xSemaphoreCreateMutex();
  
  // Create tasks
  xTaskCreate(
    modbusTask,           // Function that implements the task
    "ModbusTask",         // Text name for the task
    STACK_SIZE,           // Stack size in words, not bytes
    NULL,                 // Parameter passed into the task
    3,                    // Priority (higher number = higher priority)
    &modbusTaskHandle     // Task handle
  );
  
  xTaskCreate(
    sensorTask,
    "SensorTask",
    STACK_SIZE,
    NULL,
    2,
    &sensorTaskHandle
  );
  
  xTaskCreate(
    outputTask,
    "OutputTask",
    STACK_SIZE,
    NULL,
    1,
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
  
  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty, everything is handled by FreeRTOS tasks
}

// Task to handle Modbus communications
void modbusTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5; // 5ms (200Hz)
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    // Process Modbus communication
    modbus.poll();
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task to read sensors and update discrete inputs
void sensorTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10; // 10ms (100Hz)
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
      // Read all sensor inputs into discrete inputs array
      for (int i = 0; i < NUM_DISCRETE_INPUTS; i++) {
        discreteInputs[i] = digitalRead(sensorPins[i]);
      }
      
      // Detect bullet fired via counter sensor
      counterSensor = discreteInputs[2];
      if (counterSensor != lastCounterSensor) {
        if (counterSensor == LOW) {
          bulletCounter++; // Increment counter on falling edge
        }
      }
      lastCounterSensor = counterSensor;
      
      // Update holding register with count
      holdingRegisters[0] = bulletCounter;
      
      // Bullet counter management
      if (holdingRegisters[0] == 0) {
        bulletCounter = 0; // Reset counter if register is cleared
      }
      
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task to update outputs based on Modbus register values
void outputTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20; // 20ms (50Hz)
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
      // Update outputs based on Modbus register values
      analogWrite(ledPins[1], holdingRegisters[0]);
      analogWrite(ledPins[2], holdingRegisters[1]);
      analogWrite(ledPins[3], holdingRegisters[2]);
      analogWrite(outPins[1], holdingRegisters[3]);
      digitalWrite(outPins[0], coils[0]); // M2 trigger control
      digitalWrite(ledPins[0], coils[1]); // Reload indicator LED
      
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task to handle motor control communications
void motorControlTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50; // 50ms (20Hz)
  
  xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
      // Handle reload signal from Modbus control
      reloadSignal = coils[1];
      if (reloadSignal != lastReloadSignal) {
        if (reloadSignal == HIGH && reloadSensor == LOW) {
          xSemaphoreGive(modbusRegisterMutex); // Release mutex before long operations
          
          // Send commands to motor controller for reload operation
          Serial2.println("w axis0.requested_state 8");
          Serial2.println("v 0 -100");
          vTaskDelay(pdMS_TO_TICKS(3000)); // Use FreeRTOS delay instead of blocking delay()
          
          if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
            lastReloadSignal = reloadSignal;
            xSemaphoreGive(modbusRegisterMutex);
          }
          continue; // Skip the rest of this iteration
        }
      }
      
      // Monitor reload sensor state
      reloadSensor = discreteInputs[0];
      if (reloadSensor != lastReloadSensor) {
        if (reloadSensor == LOW) {
          xSemaphoreGive(modbusRegisterMutex); // Release mutex before I/O operation
          
          // Reset motor state when reload completes
          Serial2.println("w axis0.requested_state 1");
          
          if (xSemaphoreTake(modbusRegisterMutex, portMAX_DELAY) == pdTRUE) {
            lastReloadSensor = reloadSensor;
            xSemaphoreGive(modbusRegisterMutex);
          }
          continue; // Skip the rest of this iteration
        }
      }
      
      // Update state tracking variables
      lastReloadSignal = reloadSignal;
      lastReloadSensor = reloadSensor;
      
      xSemaphoreGive(modbusRegisterMutex);
    }
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}