#include <Arduino.h>
#include <ModbusRTUSlave.h>

// Configuration constants
#define MODBUS_BAUD 115200
#define MODBUS_CONFIG SERIAL_8N1

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
}

void loop() {
  // Read all sensor inputs into discrete inputs array
  for (int i = 0; i < NUM_DISCRETE_INPUTS; i++) {
    discreteInputs[i] = digitalRead(sensorPins[i]);
  }

  // Process Modbus communication
  modbus.poll();

  // Update outputs based on Modbus register values
  analogWrite(ledPins[1], holdingRegisters[0]);
  analogWrite(ledPins[2], holdingRegisters[1]);
  analogWrite(ledPins[3], holdingRegisters[2]);
  analogWrite(outPins[1], holdingRegisters[3]);
  digitalWrite(outPins[0], coils[0]); // M2 trigger control
  digitalWrite(ledPins[0], coils[1]); // Reload indicator LED


  // Handle reload signal from Modbus control
  reloadSignal = coils[1];
  if (reloadSignal != lastReloadSignal) {
    if (reloadSignal == HIGH && reloadSensor == LOW) {
      // Send commands to motor controller for reload operation
      Serial2.println("w axis0.requested_state 8");
      Serial2.println("v 0 -100");
      delay(3000);
    }
  }
  lastReloadSignal = reloadSignal;

  // Monitor reload sensor state
  reloadSensor = discreteInputs[0];
  if (reloadSensor != lastReloadSensor) {
    if (reloadSensor == LOW) {
      // Reset motor state when reload completes
      Serial2.println("w axis0.requested_state 1");
    }
  }
  lastReloadSensor = reloadSensor;

  // Bullet counter management
  if (holdingRegisters[0] == 0) {
    bulletCounter = 0; // Reset counter if register is cleared
  }

  // Detect bullet fired via counter sensor
  counterSensor = discreteInputs[2];
  if (counterSensor != lastCounterSensor) {
    if (counterSensor == LOW) {
      bulletCounter++; // Increment counter on falling edge
    }
  }
  lastCounterSensor = counterSensor;
  holdingRegisters[0] = bulletCounter; // Update holding register with count
} // end loop