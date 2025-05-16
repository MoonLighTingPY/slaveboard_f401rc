#include "ISM330BXSensor.h"
#include <Arduino.h>

// Коменти написав англійською, бо збираюсь запушити це на свій гіт

//  Constructor
ISM330BXSensor::ISM330BXSensor(TwoWire *i2c, uint8_t address) {
  _i2c = i2c;
  _address = address;
}

// Initialization of the ISM330BX sensor
ISM330BXStatusTypeDef ISM330BXSensor::begin() {
  uint8_t whoAmI = 0;
  int attempts = 0;
  const int maxAttempts = 5;
  
  // A lil delay to wait for I2C to settle
  delay(50);
  
  Serial.println("Starting ISM330BX initialization...");
  
  // Try to read WHO_AM_I register multiple times
  while (attempts < maxAttempts) {
    Serial.print("Attempt ");
    Serial.print(attempts + 1);
    Serial.println(" to read WHO_AM_I...");
    
    if (readRegDirect(ISM330BX_WHO_AM_I, &whoAmI) == ISM330BX_STATUS_OK) {
      Serial.print("WHO_AM_I value: 0x");
      Serial.println(whoAmI, HEX);
      
      // Successful read, check if it matches expected value
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
  
  // Setting the mode as BDU (Block Data Update) to prevent data from being updated while reading
  // This is important for ensuring data integrity during read operations
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
  uint8_t ctrl2_g = 0x50 | 0x0C; // 0x50 (208Hz) | 0x0C (2000dps)
  
  // Read current value first
  uint8_t current_value = 0;
  if (readRegDirect(ISM330BX_CTRL2_G, &current_value) == ISM330BX_STATUS_OK) {
    Serial.print("Current CTRL2_G value: 0x"); Serial.println(current_value, HEX);
  }
  // Set ODR and FS
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
  
  // Read all 6 registers in sequence from low to high b
  ISM330BXStatusTypeDef result = readRegDirect(ISM330BX_OUTZ_L_A, data, 6);
  if (result != ISM330BX_STATUS_OK) {
    return result;
  }
  
  // Combine high and low bytes and apply scale factor (0.061 mg/LSB for ±4g range)
  // !! Register order is Z, Y, X according to the datasheet
  int16_t rawZ = (int16_t)((data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
  int16_t rawX = (int16_t)((data[5] << 8) | data[4]);
  
  // Apply scale factor to convert to mg
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
  uint8_t maxAttempts = 5;  // Increase max attempts
  bool success = false;
  
  // Attempts to read the register, so i2c is restarted if needed.
  while (attempts < maxAttempts && !success) {
    // Clear any previous error state
    if (attempts > 0) {
      _i2c->end();
      delay(10);
      _i2c->begin();
      delay(10);
    }
    
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    uint8_t endResult = _i2c->endTransmission(false);  // No STOP condition, as we want to read right after
    
    if (endResult != 0) {
      Serial.print("I2C address write failed with error ");
      Serial.println(endResult);
      attempts++;
      delay(10 * attempts);  // Exponential backoff
      continue;
    }
    
    // Immediate read after write
    uint8_t bytesReceived = _i2c->requestFrom(_address, len, (bool)true);  // STOP after read, true for release bus
    if (bytesReceived != len) {
      Serial.print("Requested ");
      Serial.print(len);
      Serial.print(" bytes, but received ");
      Serial.println(bytesReceived);
      attempts++;
      delay(10 * attempts);
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
bool ISM330BXSensor::setGravityReference() {
  int32_t rawGravityVector[3];
  
  // Read the current gravity vector
  if (readRawGravityVector(rawGravityVector) != ISM330BX_STATUS_OK) {
    Serial.println("Failed to read gravity vector for reference");
    return false;
  }
  
  // Print current gravity values for debugging
   Serial.print("Current raw gravity before reset: X=");
  Serial.print(rawGravityVector[0]);
  Serial.print(", Y=");
  Serial.print(rawGravityVector[1]);
  Serial.print(", Z=");
  Serial.println(rawGravityVector[2]);
  
  // Calculate magnitude of gravity vector
  float currentNorm = sqrt(rawGravityVector[0]*rawGravityVector[0] + 
                          rawGravityVector[1]*rawGravityVector[1] + 
                          rawGravityVector[2]*rawGravityVector[2]);
  
  if (currentNorm < 100.0f) { // Prevent division by zero or very small values
    Serial.println("Gravity vector magnitude too small");
    return false;
  }
  
  // Direction we want gravity to point (typically +Z)
  float targetX = 0.0f;
  float targetY = 0.0f;
  float targetZ = 1.0f; // +Z direction
  
  // Current gravity direction (normalized)
  float gx = rawGravityVector[0] / currentNorm;
  float gy = rawGravityVector[1] / currentNorm;
  float gz = rawGravityVector[2] / currentNorm;
  
  
  // Check if current direction is already very close to target
    // Check if current direction is already very close to target
  float dotProduct = gx * targetX + gy * targetY + gz * targetZ;
  if (dotProduct > 0.999f) {
    // Already pointing in the right direction, use identity quaternion
    _referenceQuat[0] = 0.0f;
    _referenceQuat[1] = 0.0f;
    _referenceQuat[2] = 0.0f;
    _referenceQuat[3] = 1.0f;
    _hasReferenceQuat = true;
    Serial.println("Already aligned with target, using identity quaternion");
    return true;
  }
  
  // For perfectly anti-parallel case (dotProduct close to -1)
  if (dotProduct < -0.999f) {
    // Rotate 180° around X axis
    _referenceQuat[0] = 1.0f;
    _referenceQuat[1] = 0.0f;
    _referenceQuat[2] = 0.0f;
    _referenceQuat[3] = 0.0f;
    _hasReferenceQuat = true;
    Serial.println("Anti-parallel to target, using 180° X-axis rotation");
    return true;
  }
  
  // Calculate rotation axis (cross product)
  float crossX = gy * targetZ - gz * targetY;
  float crossY = gz * targetX - gx * targetZ;
  float crossZ = gx * targetY - gy * targetX;
  
  // Normalize rotation axis
  float crossMag = sqrt(crossX*crossX + crossY*crossY + crossZ*crossZ);
  if (crossMag < 0.001f) {
    Serial.println("Cross product too small, can't determine rotation axis");
    return false;
  }
  
  crossX /= crossMag;
  crossY /= crossMag;
  crossZ /= crossMag;
  
  // Calculate rotation angle and build quaternion
  float angle = acos(constrain(dotProduct, -1.0f, 1.0f));
  float halfAngle = angle * 0.5f;
  float sinHalfAngle = sin(halfAngle);
  
  _referenceQuat[0] = crossX * sinHalfAngle;
  _referenceQuat[1] = crossY * sinHalfAngle;
  _referenceQuat[2] = crossZ * sinHalfAngle;
  _referenceQuat[3] = cos(halfAngle);
  
  // Ensure quaternion is normalized
  float qNorm = sqrt(_referenceQuat[0]*_referenceQuat[0] + 
                    _referenceQuat[1]*_referenceQuat[1] + 
                    _referenceQuat[2]*_referenceQuat[2] + 
                    _referenceQuat[3]*_referenceQuat[3]);
  
  _referenceQuat[0] /= qNorm;
  _referenceQuat[1] /= qNorm;
  _referenceQuat[2] /= qNorm;
  _referenceQuat[3] /= qNorm;
  
  // Print quaternion for debugging
  Serial.print("Quaternion: X=");
  Serial.print(_referenceQuat[0], 6);
  Serial.print(", Y=");
  Serial.print(_referenceQuat[1], 6);
  Serial.print(", Z=");
  Serial.print(_referenceQuat[2], 6);
  Serial.print(", W=");
  Serial.println(_referenceQuat[3], 6);
  
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

// Improved quaternion rotation function
void ISM330BXSensor::quaternionRotate(const float *q, const float *v, float *vOut) {
  // More numerically stable quaternion rotation implementation
  
  // Calculate quaternion rotation: q * v * q^(-1)
  // Using optimized formula to avoid full quaternion multiplication
  
  float qw = q[3];
  float qx = q[0];
  float qy = q[1];
  float qz = q[2];
  float vx = v[0];
  float vy = v[1];
  float vz = v[2];
  
  // Pre-compute some common terms
  float qw2 = qw * qw;
  float qx2 = qx * qx;
  float qy2 = qy * qy;
  float qz2 = qz * qz;
  
  float qwx = qw * qx;
  float qwy = qw * qy;
  float qwz = qw * qz;
  float qxy = qx * qy;
  float qxz = qx * qz;
  float qyz = qy * qz;
  
  // Optimized rotation calculation
  vOut[0] = vx * (qw2 + qx2 - qy2 - qz2) + 
            vy * 2 * (qxy - qwz) + 
            vz * 2 * (qxz + qwy);
            
  vOut[1] = vx * 2 * (qxy + qwz) + 
            vy * (qw2 - qx2 + qy2 - qz2) + 
            vz * 2 * (qyz - qwx);
            
  vOut[2] = vx * 2 * (qxz - qwy) + 
            vy * 2 * (qyz + qwx) + 
            vz * (qw2 - qx2 - qy2 + qz2);
}


ISM330BXStatusTypeDef ISM330BXSensor::readRawGravityVector(int32_t *gravityVector) {
  uint8_t data[6];
  auto result = readRegDirect(ISM330BX_UI_OUTZ_L_A_DualC, data, 6);
  if (result != ISM330BX_STATUS_OK) {
    Serial.println("Failed to read raw gravity vector data");
    return result;
  }
  
  int16_t rawZ = (int16_t)((data[1] << 8) | data[0]);
  int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
  int16_t rawX = (int16_t)((data[5] << 8) | data[4]);

  const float scale = 0.488f; // mg per LSB
  gravityVector[0] = (int32_t)round(rawX * scale);
  gravityVector[1] = (int32_t)round(rawY * scale);
  gravityVector[2] = (int32_t)round(rawZ * scale);
  
  return ISM330BX_STATUS_OK;
}

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

  const float scale = 0.488f; // mg per LSB
  gravityVector[0] = (int32_t)round(rawX * scale);
  gravityVector[1] = (int32_t)round(rawY * scale);
  gravityVector[2] = (int32_t)round(rawZ * scale);
  
  // Apply reference quaternion if set
  if (_hasReferenceQuat) {
    // Store original magnitude
    float originalMag = sqrt(gravityVector[0]*gravityVector[0] + 
                            gravityVector[1]*gravityVector[1] + 
                            gravityVector[2]*gravityVector[2]);
                            
    // Apply rotation
    float vecFloat[3] = {(float)gravityVector[0], (float)gravityVector[1], (float)gravityVector[2]};
    float vecRotated[3];
    quaternionRotate(_referenceQuat, vecFloat, vecRotated);
    
    // Preserve original magnitude
    float rotatedMag = sqrt(vecRotated[0]*vecRotated[0] + 
                            vecRotated[1]*vecRotated[1] + 
                            vecRotated[2]*vecRotated[2]);
    
    if (rotatedMag > 0.0001f) {
      float scaleFactor = originalMag / rotatedMag;
      for (int i = 0; i < 3; i++) {
        vecRotated[i] *= scaleFactor;
        gravityVector[i] = (int32_t)round(vecRotated[i]);
      }
    }
  }

  return ISM330BX_STATUS_OK;
}