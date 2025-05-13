// lib/ISM330BXSensor/ISM330BXSensor.cpp
#include "ISM330BXSensor.h"
#include <Arduino.h>

// Constructor
ISM330BXSensor::ISM330BXSensor(TwoWire *i2c, uint8_t address) {
  _i2c = i2c;
  _address = address;
}

// Initialize the sensor
ISM330BXStatusTypeDef ISM330BXSensor::begin() {
  uint8_t whoAmI = 0;
  int attempts = 0;
  const int maxAttempts = 5;
  
  // Give I2C bus time to settle
  delay(50);
  
  Serial.println("Starting ISM330BX initialization...");
  
  // Try multiple times to read WHO_AM_I
  while (attempts < maxAttempts) {
    Serial.print("Attempt ");
    Serial.print(attempts + 1);
    Serial.println(" to read WHO_AM_I...");
    
    if (readRegDirect(ISM330BX_WHO_AM_I, &whoAmI) == ISM330BX_STATUS_OK) {
      Serial.print("WHO_AM_I value: 0x");
      Serial.println(whoAmI, HEX);
      
      // Successfully read WHO_AM_I
      if (whoAmI == ISM330BX_WHO_AM_I_EXPECTED) {
        break;
      } else {
        Serial.println("WHO_AM_I mismatch, expected 0x71");
      }
    } else {
      Serial.println("Failed to read WHO_AM_I");
    }
    
    attempts++;
    delay(50); // Wait before retry
  }
  
  if (attempts >= maxAttempts) {
    Serial.println("Failed to communicate with ISM330BX after multiple attempts");
    return ISM330BX_STATUS_ERROR;
  }
  
  // Set BDU (Block Data Update)
  Serial.println("Setting BDU...");
  uint8_t ctrl3 = 0x04; // BDU bit
  if (writeRegDirect(ISM330BX_CTRL3_C, ctrl3) != ISM330BX_STATUS_OK) {
    Serial.println("Failed to set BDU");
    return ISM330BX_STATUS_ERROR;
  }
  
  Serial.println("ISM330BX basic initialization successful");
  return ISM330BX_STATUS_OK;
}

// Enable accelerometer
ISM330BXStatusTypeDef ISM330BXSensor::enableAccelerometer() {
  // Set ODR (104 Hz) and FS (±4g)
  uint8_t ctrl1_xl = 0x48; // 0x40 (104Hz) | 0x08 (±4g)
  ISM330BXStatusTypeDef result = writeRegDirect(ISM330BX_CTRL1_XL, ctrl1_xl);
  if (result == ISM330BX_STATUS_OK) {
    Serial.println("Accelerometer enabled successfully");
  } else {
    Serial.println("Failed to enable accelerometer");
  }
  return result;
}

// Enable gyroscope
ISM330BXStatusTypeDef ISM330BXSensor::enableGyroscope() {
  // Set ODR (104 Hz) and FS (2000dps)
  uint8_t ctrl2_g = 0x4C; // 0x40 (104Hz) | 0x0C (2000dps)
  ISM330BXStatusTypeDef result = writeRegDirect(ISM330BX_CTRL2_G, ctrl2_g);
  if (result == ISM330BX_STATUS_OK) {
    Serial.println("Gyroscope enabled successfully");
  } else {
    Serial.println("Failed to enable gyroscope");
  }
  return result;
}

// Check if new data is available
bool ISM330BXSensor::checkDataReady() {
  uint8_t status = 0;
  
  if (readRegDirect(ISM330BX_STATUS_REG, &status) != ISM330BX_STATUS_OK) {
    return false;
  }
  
  return (status & 0x03) != 0; // Check XLDA or GDA bits
}

// Read accelerometer data
ISM330BXStatusTypeDef ISM330BXSensor::readAcceleration(int32_t *acceleration) {
  uint8_t data[6];
  
  // Read all 6 registers in sequence
  ISM330BXStatusTypeDef result = readRegDirect(ISM330BX_OUTZ_L_A, data, 6);
  if (result != ISM330BX_STATUS_OK) {
    return result;
  }
  
  // Combine high and low bytes and apply scale factor (0.061 mg/LSB for ±4g range)
  // Note: Register order is Z, Y, X according to the datasheet
  int16_t rawZ = (int16_t)((data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
  int16_t rawX = (int16_t)((data[5] << 8) | data[4]);
  
  acceleration[0] = (int32_t)(rawX * 0.061f);
  acceleration[1] = (int32_t)(rawY * 0.061f);
  acceleration[2] = (int32_t)(rawZ * 0.061f);
  
  return ISM330BX_STATUS_OK;
}

// Read gyroscope data
ISM330BXStatusTypeDef ISM330BXSensor::readGyroscope(int32_t *angularRate) {
  uint8_t data[6];
  
  // Read all 6 registers in sequence
  ISM330BXStatusTypeDef result = readRegDirect(ISM330BX_OUTX_L_G, data, 6);
  if (result != ISM330BX_STATUS_OK) {
    return result;
  }
  
  // Combine high and low bytes and apply scale factor (70 mdps/LSB for 2000dps range)
  int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
  int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);
  
  angularRate[0] = (int32_t)(rawX * 70.0f);
  angularRate[1] = (int32_t)(rawY * 70.0f);
  angularRate[2] = (int32_t)(rawZ * 70.0f);
  
  return ISM330BX_STATUS_OK;
}

// Direct I2C register read (1 byte)
ISM330BXStatusTypeDef ISM330BXSensor::readRegDirect(uint8_t reg, uint8_t *data) {
  return readRegDirect(reg, data, 1);
}

// Direct I2C register read (multiple bytes)
ISM330BXStatusTypeDef ISM330BXSensor::readRegDirect(uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t attempts = 0;
  uint8_t maxAttempts = 3;
  bool success = false;
  
  while (attempts < maxAttempts && !success) {
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    uint8_t endResult = _i2c->endTransmission(true); // Full STOP
    
    if (endResult != 0) {
      Serial.print("I2C address write failed with error ");
      Serial.println(endResult);
      attempts++;
      delay(5);
      continue;
    }
    
    delay(1); // Small delay
    
    // Separate transaction for reading
    uint8_t bytesReceived = _i2c->requestFrom(_address, len);
    if (bytesReceived != len) {
      Serial.print("Requested ");
      Serial.print(len);
      Serial.print(" bytes, but received ");
      Serial.println(bytesReceived);
      attempts++;
      delay(5);
      continue;
    }
    
    for (uint8_t i = 0; i < len; i++) {
      data[i] = _i2c->read();
    }
    
    success = true;
  }
  
  return success ? ISM330BX_STATUS_OK : ISM330BX_STATUS_ERROR;
}

// Direct I2C register write
ISM330BXStatusTypeDef ISM330BXSensor::writeRegDirect(uint8_t reg, uint8_t data) {
  uint8_t attempts = 0;
  uint8_t maxAttempts = 3;
  bool success = false;
  
  while (attempts < maxAttempts && !success) {
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    _i2c->write(data);
    uint8_t endResult = _i2c->endTransmission();
    
    if (endResult != 0) {
      Serial.print("I2C write failed with error ");
      Serial.println(endResult);
      attempts++;
      delay(5);
      continue;
    }
    
    success = true;
  }
  
  return success ? ISM330BX_STATUS_OK : ISM330BX_STATUS_ERROR;
}

// Keep these for backward compatibility
ISM330BXStatusTypeDef ISM330BXSensor::readReg(uint8_t reg, uint8_t *data) {
  return readRegDirect(reg, data);
}

ISM330BXStatusTypeDef ISM330BXSensor::writeReg(uint8_t reg, uint8_t data) {
  return writeRegDirect(reg, data);
}