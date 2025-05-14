#ifndef ISM330BX_SENSOR_H
#define ISM330BX_SENSOR_H

#include <Wire.h>

// ISM330BX Register map
#define ISM330BX_WHO_AM_I          0x0F  // Device identification register
#define ISM330BX_CTRL1_XL          0x10  // Accelerometer control register
#define ISM330BX_CTRL2_G           0x11  // Gyroscope control register
#define ISM330BX_CTRL3_C           0x12  // Control register 3
#define ISM330BX_CTRL7             0x16  // Control register 7
#define ISM330BX_CTRL6_G           0x15 // Gyroscope control register 6 (for Full Scale)
#define ISM330BX_CTRL8_XL          0x17  // Control register 8
#define ISM330BX_FUNC_CFG_ACCESS   0x01
#define ISM330BX_STATUS_REG        0x1E  // Status data register
#define ISM330BX_OUTX_L_G          0x22  // Gyroscope X-axis low byte
#define ISM330BX_OUTX_H_G          0x23  // Gyroscope X-axis high byte
#define ISM330BX_OUTY_L_G          0x24  // Gyroscope Y-axis low byte
#define ISM330BX_OUTY_H_G          0x25  // Gyroscope Y-axis high byte
#define ISM330BX_OUTZ_L_G          0x26  // Gyroscope Z-axis low byte
#define ISM330BX_OUTZ_H_G          0x27  // Gyroscope Z-axis high byte
#define ISM330BX_OUTZ_L_A          0x28  // Acceleration Z-axis low byte
#define ISM330BX_OUTZ_H_A          0x29  // Acceleration Z-axis high byte
#define ISM330BX_OUTY_L_A          0x2A  // Acceleration Y-axis low byte
#define ISM330BX_OUTY_H_A          0x2B  // Acceleration Y-axis high byte
#define ISM330BX_OUTX_L_A          0x2C  // Acceleration X-axis low byte
#define ISM330BX_OUTX_H_A          0x2D  // Acceleration X-axis high byte

// Gravity vector output registers (UI_OUTX/Y/Z for DualC mode)
#define ISM330BX_UI_OUTZ_L_A_DualC 0x34  // Gravity Z-axis low byte
#define ISM330BX_UI_OUTZ_H_A_DualC 0x35  // Gravity Z-axis high byte
#define ISM330BX_UI_OUTY_L_A_DualC 0x36  // Gravity Y-axis low byte
#define ISM330BX_UI_OUTY_H_A_DualC 0x37  // Gravity Y-axis high byte
#define ISM330BX_UI_OUTX_L_A_DualC 0x38  // Gravity X-axis low byte
#define ISM330BX_UI_OUTX_H_A_DualC 0x39  // Gravity X-axis high byte

#define ISM330BX_FIFO_DATA_OUT_TAG 0x78
#define ISM330BX_FIFO_CTRL1        0x07
#define ISM330BX_FIFO_CTRL2        0x08
#define ISM330BX_FIFO_CTRL3        0x09
#define ISM330BX_FIFO_CTRL4        0x0A
#define ISM330BX_FIFO_STATUS1      0x1B
#define ISM330BX_FIFO_STATUS2      0x1C
#define ISM330BX_FIFO_DATA_OUT_BYTE_0 0x79
#define ISM330BX_FIFO_DATA_OUT_BYTE_1 0x7A  // FIFO data output register
#define ISM330BX_FIFO_DATA_OUT_BYTE_2 0x7B  // FIFO data output register
#define ISM330BX_FIFO_DATA_OUT_BYTE_3 0x7C  // FIFO data output register
#define ISM330BX_FIFO_DATA_OUT_BYTE_4 0x7D  // FIFO data output register
#define ISM330BX_FIFO_DATA_OUT_BYTE_5 0x7E  // FIFO data output register


#define ISM330BX_EMB_FUNC_EN_A     0x04  // Embedded functions enable register A
#define ISM330BX_EMB_FUNC_INIT_A   0x66  // Embedded functions initialization register A
#define ISM330BX_EMB_FUNC_FIFO_EN_A 0x44
#define ISM330BX_EMB_FUNC_FIFO_EN_B 0x45

#define ISM330BX_SFLP_GAME_EN      0x01  // Bit 0 of EMB_FUNC_EN_A register
#define ISM330BX_SFLP_GAME_INIT    0x01  // Bit 0 of EMB_FUNC_INIT_A register
#define ISM330BX_SFLP_ODR            0x5E   // SFLP low-power output data rate
#define ISM330BX_MLC_FIFO_EN           (1 << 0)
#define ISM330BX_STEP_COUNTER_FIFO_EN  (1 << 1)
#define ISM330BX_SFLP_GBIAS_FIFO_EN    (1 << 2)
#define ISM330BX_SFLP_GRAVITY_FIFO_EN  (1 << 3)
#define ISM330BX_SFLP_GAME_FIFO_EN     (1 << 4)
#define ISM330BX_SFLP_GYROSCOPE_BIAS_TAG        0x03
#define ISM330BX_SFLP_GRAVITY_VECTOR_TAG        0x06
#define ISM330BX_SFLP_GAME_ROTATION_VECTOR_TAG  0x13


#define ISM330BX_WHO_AM_I_EXPECTED 0x71

// Status
typedef enum {
  ISM330BX_STATUS_OK = 0,
  ISM330BX_STATUS_ERROR
} ISM330BXStatusTypeDef;

class ISM330BXSensor {
  public:
    ISM330BXSensor(TwoWire *i2c, uint8_t address = 0x6A);

    ISM330BXStatusTypeDef begin();
    ISM330BXStatusTypeDef enableAccelerometer();
    ISM330BXStatusTypeDef enableGyroscope();
    ISM330BXStatusTypeDef enableSensorFusion();

    ISM330BXStatusTypeDef readAcceleration(int32_t *acceleration);
    ISM330BXStatusTypeDef readGyroscope(int32_t *angularRate);
    ISM330BXStatusTypeDef readGravityVector(int32_t *gravityVector);
    ISM330BXStatusTypeDef readRawGravityVector(int32_t *gravityVector);

    ISM330BXStatusTypeDef readReg(uint8_t reg, uint8_t *data);
    ISM330BXStatusTypeDef writeReg(uint8_t reg, uint8_t data);

    bool checkDataReady();
    bool checkGyroDataReady();
    bool checkGravityDataReady();

    void setGravityZero(); // legacy, does nothing
    bool setGravityReference(); // new
    bool applyGravityReference(int32_t *vec);

  private:
    TwoWire *_i2c;
    uint8_t _address;

    float _referenceQuat[4] = {1, 0, 0, 0};
    bool _hasReferenceQuat = false;

    ISM330BXStatusTypeDef readQuaternion(float *quat);
    void quaternionInverse(const float *q, float *qInv);
    void quaternionRotate(const float *q, const float *v, float *vOut);

    ISM330BXStatusTypeDef readRegDirect(uint8_t reg, uint8_t *data);
    ISM330BXStatusTypeDef readRegDirect(uint8_t reg, uint8_t *data, uint8_t len);
    ISM330BXStatusTypeDef writeRegDirect(uint8_t reg, uint8_t data);
};

#endif
