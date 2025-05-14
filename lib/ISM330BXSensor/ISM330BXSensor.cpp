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
// In ISM330BXSensor.cpp - enableGyroscope() function
ISM330BXStatusTypeDef ISM330BXSensor::enableGyroscope() {
  // Try a stronger ODR and range setting
  uint8_t ctrl2_g = 0x50 | 0x0C; // 0x50 (208Hz) | 0x0C (2000dps)
  
  // Read current value first
  uint8_t current_value = 0;
  if (readRegDirect(ISM330BX_CTRL2_G, &current_value) == ISM330BX_STATUS_OK) {
    Serial.print("Current CTRL2_G value: 0x"); Serial.println(current_value, HEX);
  }
  
  ISM330BXStatusTypeDef result = writeRegDirect(ISM330BX_CTRL2_G, ctrl2_g);
  if (result == ISM330BX_STATUS_OK) {
    Serial.println("Gyroscope enabled with higher settings");
    
    // Verify the write
    uint8_t verify_value = 0;
    if (readRegDirect(ISM330BX_CTRL2_G, &verify_value) == ISM330BX_STATUS_OK) {
      Serial.print("Verified CTRL2_G value: 0x"); Serial.println(verify_value, HEX);
      if (verify_value != ctrl2_g) {
        Serial.println("WARNING: Gyroscope settings weren't applied correctly!");
      }
    }
    
    // Enable HP filter for gyroscope - might help with zero drift
    uint8_t ctrl7 = 0x00 | 0x40; // Set high-pass filter
    writeRegDirect(ISM330BX_CTRL7, 0x40);
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

bool ISM330BXSensor::checkGyroDataReady() {
  uint8_t status = 0;
  
  if (readRegDirect(ISM330BX_STATUS_REG, &status) != ISM330BX_STATUS_OK) {
    return false;
  }
  
  Serial.print("Status register: 0x"); Serial.println(status, HEX);
  return (status & 0x02) != 0; // Check only GDA bit (bit 1)
}

// Read gyroscope data
ISM330BXStatusTypeDef ISM330BXSensor::readGyroscope(int32_t *angularRate) {
  uint8_t data[6];
  
  // Read all 6 registers in sequence
  ISM330BXStatusTypeDef result = readRegDirect(ISM330BX_OUTX_L_G, data, 6);
  if (result != ISM330BX_STATUS_OK) {
    Serial.println("Failed to read gyroscope data");
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


ISM330BXStatusTypeDef ISM330BXSensor::enableSensorFusion() {
  Serial.println("Enabling sensor fusion for gravity vector...");
  
  // Check current status first
  uint8_t current_value = 0;
  if (readRegDirect(ISM330BX_EMB_FUNC_EN_A, &current_value) == ISM330BX_STATUS_OK) {
    Serial.print("Current EMB_FUNC_EN_A value: 0x");
    Serial.println(current_value, HEX);
  }
  
  // Enable sensor fusion by setting SFLP_GAME_EN bit
  ISM330BXStatusTypeDef result = writeRegDirect(ISM330BX_EMB_FUNC_EN_A, current_value | ISM330BX_SFLP_GAME_EN);
  
  if (result == ISM330BX_STATUS_OK) {
    Serial.println("Sensor fusion enabled successfully");
    
    // Initialize sensor fusion
    result = writeRegDirect(ISM330BX_EMB_FUNC_INIT_A, ISM330BX_SFLP_GAME_INIT);
    if (result == ISM330BX_STATUS_OK) {
      Serial.println("Sensor fusion initialized successfully");
    } else {
      Serial.println("Failed to initialize sensor fusion");
    }
  } else {
    Serial.println("Failed to enable sensor fusion");
  }
  
  return result;
}

// Check if gravity vector data is ready
bool ISM330BXSensor::checkGravityDataReady() {
  // We can use the same status register as for accelerometer data
  uint8_t status = 0;
  
  if (readRegDirect(ISM330BX_STATUS_REG, &status) != ISM330BX_STATUS_OK) {
    return false;
  }
  
  // XLDA bit (bit 0) indicates accelerometer data ready
  // Since gravity is derived from accelerometer, we can use this bit
  return (status & 0x01) != 0;
}

// Read quaternion data
ISM330BXStatusTypeDef ISM330BXSensor::readQuaternion(float *quat) {
  uint8_t data[6];
  auto result = readRegDirect(ISM330BX_FIFO_DATA_OUT_TAG, data, 6);
  if (result != ISM330BX_STATUS_OK) {
    Serial.println("Failed to read quaternion data");
    return result;
  }

  uint16_t *rawQuat = (uint16_t*)data;
  
  // Convert from half-precision to float
  // Using simplified conversion for Arduino
  float sumsq = 0;
  
  for (int i = 0; i < 3; i++) {
    // Simple half-precision to float conversion
    uint16_t h = rawQuat[i];
    uint16_t h_exp = (h & 0x7c00u);
    uint32_t f_sgn = ((uint32_t)h & 0x8000u) << 16;
    uint32_t result = 0;
    
    if (h_exp == 0x0000u) { // Zero or subnormal
      uint16_t h_sig = (h & 0x03ffu);
      if (h_sig == 0) {
        result = f_sgn;
      } else {
        h_sig <<= 1;
        while ((h_sig & 0x0400u) == 0) {
          h_sig <<= 1;
          h_exp++;
        }
        uint32_t f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
        uint32_t f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
        result = f_sgn + f_exp + f_sig;
      }
    } else if (h_exp == 0x7c00u) { // Infinity or NaN
      result = f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
    } else { // Normalized
      result = f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
    }
    
    // Convert the bit pattern to float
    float* resultFloat = (float*)&result;
    quat[i] = *resultFloat;
    sumsq += quat[i] * quat[i];
  }
  
  // Calculate the fourth component
  if (sumsq > 1.0f) {
    // Normalize if sum of squares exceeds 1
    float n = sqrt(sumsq);
    quat[0] /= n;
    quat[1] /= n;
    quat[2] /= n;
    sumsq = 1.0f;
  }
  
  // Calculate w component
  quat[3] = sqrt(1.0f - sumsq);
  
  return ISM330BX_STATUS_OK;
}

// Set the current orientation as reference (zero) position
// Set the current orientation as reference (zero) position
bool ISM330BXSensor::setGravityReference() {
  int32_t gravityVector[3];
  
  // Read the current gravity vector
  if (readGravityVector(gravityVector) != ISM330BX_STATUS_OK) {
    Serial.println("Failed to read gravity vector for reference");
    return false;
  }
  
  // Calculate rotation needed to transform current gravity to [0,0,1000]
  float currentNorm = sqrt(gravityVector[0]*gravityVector[0] + 
                           gravityVector[1]*gravityVector[1] + 
                           gravityVector[2]*gravityVector[2]);
  
  if (currentNorm < 10.0f) { // Prevent division by zero or very small values
    Serial.println("Gravity vector magnitude too small");
    return false;
  }
  
  // Normalize the gravity vector
  float gx = gravityVector[0] / currentNorm;
  float gy = gravityVector[1] / currentNorm;
  float gz = gravityVector[2] / currentNorm;
  
  // Default gravity vector (unit vector in Z direction)
  float defaultGx = 0.0f;
  float defaultGy = 0.0f;
  float defaultGz = 1.0f;
  
  // Calculate rotation axis and angle using cross product
  float crossX = gy * defaultGz - gz * defaultGy;
  float crossY = gz * defaultGx - gx * defaultGz;
  float crossZ = gx * defaultGy - gy * defaultGx;
  
  float crossMag = sqrt(crossX*crossX + crossY*crossY + crossZ*crossZ);
  
  // If vectors are nearly parallel, use simpler method
  if (crossMag < 0.001f) {
    if (gz > 0.999f) { // Already aligned with Z
      _referenceQuat[0] = 0.0f;
      _referenceQuat[1] = 0.0f;
      _referenceQuat[2] = 0.0f;
      _referenceQuat[3] = 1.0f;
    } else { // Anti-parallel, rotate 180° around X
      _referenceQuat[0] = 1.0f;
      _referenceQuat[1] = 0.0f;
      _referenceQuat[2] = 0.0f;
      _referenceQuat[3] = 0.0f;
    }
  } else {
    // Normalize the cross product
    crossX /= crossMag;
    crossY /= crossMag;
    crossZ /= crossMag;
    
    // Calculate the rotation angle (dot product)
    float dotProduct = gx * defaultGx + gy * defaultGy + gz * defaultGz;
    float angle = acos(constrain(dotProduct, -1.0f, 1.0f));
    
    // Create quaternion from axis-angle
    float halfAngle = angle * 0.5f;
    float sinHalfAngle = sin(halfAngle);
    
    _referenceQuat[0] = crossX * sinHalfAngle;
    _referenceQuat[1] = crossY * sinHalfAngle;
    _referenceQuat[2] = crossZ * sinHalfAngle;
    _referenceQuat[3] = cos(halfAngle);
  }
  
  _hasReferenceQuat = true;
  Serial.println("Gravity reference set successfully");
  return true;
}


// Apply the gravity reference to transform the gravity vector
bool ISM330BXSensor::applyGravityReference(int32_t *vec) {
  if (!_hasReferenceQuat) {
    return false;
  }
  
  // Convert integer vector to float
  float vecFloat[3];
  for (int i = 0; i < 3; i++) {
    vecFloat[i] = (float)vec[i];
  }
  
  // Apply rotation using reference quaternion
  float vecRotated[3];
  quaternionRotate(_referenceQuat, vecFloat, vecRotated);
  
  // Convert back to integer
  for (int i = 0; i < 3; i++) {
    vec[i] = (int32_t)round(vecRotated[i]);
  }
  
  return true;
}

// Calculate the inverse of a quaternion
void ISM330BXSensor::quaternionInverse(const float *q, float *qInv) {
  // For unit quaternions, inverse equals conjugate
  float norm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  
  if (fabs(norm - 1.0f) > 0.01f) {
    // Not a unit quaternion, need to normalize
    float invNorm = 1.0f / norm;
    qInv[0] = -q[0] * invNorm;
    qInv[1] = -q[1] * invNorm;
    qInv[2] = -q[2] * invNorm;
    qInv[3] = q[3] * invNorm;
  } else {
    // Unit quaternion, just take conjugate
    qInv[0] = -q[0];
    qInv[1] = -q[1];
    qInv[2] = -q[2];
    qInv[3] = q[3];
  }
}

// Rotate a vector by a quaternion
void ISM330BXSensor::quaternionRotate(const float *q, const float *v, float *vOut) {
  // qv = [0, v]
  float qv[4] = {v[0], v[1], v[2], 0.0f};
  float temp[4];
  
  // temp = q * qv
  temp[0] = q[3]*qv[0] + q[1]*qv[2] - q[2]*qv[1];
  temp[1] = q[3]*qv[1] + q[2]*qv[0] - q[0]*qv[2];
  temp[2] = q[3]*qv[2] + q[0]*qv[1] - q[1]*qv[0];
  temp[3] = -q[0]*qv[0] - q[1]*qv[1] - q[2]*qv[2];
  
  // qinv = conjugate of q (since q is normalized)
  float qinv[4] = {-q[0], -q[1], -q[2], q[3]};
  
  // vOut = temp * qinv (only vector part)
  vOut[0] = temp[0]*qinv[3] + temp[3]*qinv[0] + temp[1]*qinv[2] - temp[2]*qinv[1];
  vOut[1] = temp[1]*qinv[3] + temp[3]*qinv[1] + temp[2]*qinv[0] - temp[0]*qinv[2];
  vOut[2] = temp[2]*qinv[3] + temp[3]*qinv[2] + temp[0]*qinv[1] - temp[1]*qinv[0];
}

// Read gravity vector data
// Read gravity vector data with reference correction
ISM330BXStatusTypeDef ISM330BXSensor::readGravityVector(int32_t *gravityVector) {
  uint8_t data[6];
  auto result = readRegDirect(ISM330BX_UI_OUTZ_L_A_DualC, data, 6);
  if (result != ISM330BX_STATUS_OK) {
    Serial.println("Failed to read gravity vector data");
    return result;
  }
  int16_t rawZ = (int16_t)((data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
  int16_t rawX = (int16_t)((data[5] << 8) | data[4]);

  const float scale = 0.488f;       // mg per LSB
  gravityVector[0] = (int32_t)round(rawX * scale);
  gravityVector[1] = (int32_t)round(rawY * scale);
  gravityVector[2] = (int32_t)round(rawZ * scale);
  
  // Apply reference quaternion if set
  if (_hasReferenceQuat) {
    applyGravityReference(gravityVector);
  }

  return ISM330BX_STATUS_OK;
}

void ISM330BXSensor::setGravityZero() {
  setGravityReference();
}