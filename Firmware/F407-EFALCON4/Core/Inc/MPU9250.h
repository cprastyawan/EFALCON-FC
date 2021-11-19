/*
MPU9250.h
BananaPi Bit Team
juwan@banana-pi.com (Juwan·C)
wanghao@banana-pi.com (Hulk Wang)
We modified it based on https://github.com/bolderflight/MPU9250
The main purpose is to adapt the MPU9250 driver library of BPI-BIT (espressif32).
Copyright (c) 2017 BananaPi Bit Team
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MPU9250_h
#define MPU9250_h

#include <stm32f4xx_hal.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "IMU.h"

#define ACTIVATE_SPI(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define DEACTIVATE_SPI(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define delay(x) HAL_Delay((uint32_t)x)
//#define RAD_TO_DEG 57.29578

typedef enum {
  GYRO_RANGE_250DPS,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
}GyroRange;

typedef enum {
  ACCEL_RANGE_2G,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G
}AccelRange;

typedef enum {
  DLPF_BANDWIDTH_184HZ,
  DLPF_BANDWIDTH_92HZ,
  DLPF_BANDWIDTH_41HZ,
  DLPF_BANDWIDTH_20HZ,
  DLPF_BANDWIDTH_10HZ,
  DLPF_BANDWIDTH_5HZ
}DlpfBandwidth;

typedef enum{
  LP_ACCEL_ODR_0_24HZ = 0,
    LP_ACCEL_ODR_0_49HZ = 1,
    LP_ACCEL_ODR_0_98HZ = 2,
    LP_ACCEL_ODR_1_95HZ = 3,
    LP_ACCEL_ODR_3_91HZ = 4,
    LP_ACCEL_ODR_7_81HZ = 5,
    LP_ACCEL_ODR_15_63HZ = 6,
    LP_ACCEL_ODR_31_25HZ = 7,
    LP_ACCEL_ODR_62_50HZ = 8,
    LP_ACCEL_ODR_125HZ = 9,
    LP_ACCEL_ODR_250HZ = 10,
    LP_ACCEL_ODR_500HZ = 11
}LpAccelOdr;

typedef struct {
  SPI_HandleTypeDef * _spi;
  GPIO_TypeDef * _csPort;
  uint16_t _csPin;
  bool _useSPI;
  bool _useSPIHS;
  // track success of interacting with sensor
  int _status;
  // buffer for reading from sensor
  uint8_t _buffer[21];

  // data counts
  int16_t _axcounts, _aycounts, _azcounts;
  int16_t _gxcounts, _gycounts, _gzcounts;
  int16_t _hxcounts, _hycounts, _hzcounts;
  int16_t _tcounts;
  // data buffer
  float _ax, _ay, _az;
  float _gx, _gy, _gz;
  float _hx, _hy, _hz;
  float _t;
  // wake on motion
  uint8_t _womThreshold;
  // scale factors
  float _accelScale;
  float _gyroScale;
  float _magScaleX, _magScaleY, _magScaleZ;

  // configuration
  AccelRange _accelRange;
  GyroRange _gyroRange;
  DlpfBandwidth _bandwidth;
  uint8_t _srd;
  // gyro bias estimation
  size_t _numSamples;
  double _gxbD, _gybD, _gzbD;
  float _gxb, _gyb, _gzb;
  // accel bias and scale factor estimation
  double _axbD, _aybD, _azbD;
  float _axmax, _aymax, _azmax;
  float _axmin, _aymin, _azmin;
  float _axb, _ayb, _azb;
  float _axs;
  float _ays;
  float _azs;
  // magnetometer bias and scale factor estimation
  uint16_t _maxCounts;
  float _deltaThresh;
  uint8_t _coeff;
  uint16_t _counter;
  float _framedelta, _delta;
  float _hxfilt, _hyfilt, _hzfilt;
  float _hxmax, _hymax, _hzmax;
  float _hxmin, _hymin, _hzmin;
  float _hxb, _hyb, _hzb;
  float _hxs;
  float _hys;
  float _hzs;
  float _avgs;

}MPU9250_t;

typedef struct {
	MPU9250_t * mpu9250;
	bool _enFifoAccel, _enFifoGyro, _enFifoMag, _enFifoTemp;
	size_t _fifoSize, _fifoFrameSize;
	float _axFifo[85], _ayFifo[85], _azFifo[85];
	size_t _aSize;
	float _gxFifo[85], _gyFifo[85], _gzFifo[85];
	size_t _gSize;
	float _hxFifo[73], _hyFifo[73], _hzFifo[73];
	size_t _hSize;
	float _tFifo[256];
	size_t _tSize;
}MPU9250FIFO_t;


void MPU9250_init(MPU9250_t *mpu9250, SPI_HandleTypeDef *hspi, GPIO_TypeDef * csPort, uint16_t csPin);
int MPU9250_begin(MPU9250_t *mpu9250);
int MPU9250_setAccelRange(MPU9250_t *mpu9250, AccelRange range);
int MPU9250_setGyroRange(MPU9250_t *mpu9250, GyroRange range);
int MPU9250_setDlpfBandwidth(MPU9250_t *mpu9250, DlpfBandwidth bandwidth);
int MPU9250_setSrd(MPU9250_t *mpu9250, uint8_t srd);
int MPU9250_enableDataReadyInterrupt(MPU9250_t *mpu9250);
int MPU9250_disableDataReadyInterrupt(MPU9250_t *mpu9250);
int MPU9250_enableWakeOnMotion(MPU9250_t *mpu9250, float womThresh_mg, LpAccelOdr odr);
int MPU9250_readSensor(MPU9250_t *mpu9250);
float MPU9250_getAccelX_mss(MPU9250_t *mpu9250);
float MPU9250_getAccelY_mss(MPU9250_t *mpu9250);
float MPU9250_getAccelZ_mss(MPU9250_t *mpu9250);
float MPU9250_getGyroX_rads(MPU9250_t *mpu9250);
float MPU9250_getGyroY_rads(MPU9250_t *mpu9250);
float MPU9250_getGyroZ_rads(MPU9250_t *mpu9250);
float MPU9250_getMagX_uT(MPU9250_t *mpu9250);
float MPU9250_getMagY_uT(MPU9250_t *mpu9250);
float MPU9250_getMagZ_uT(MPU9250_t *mpu9250);
float MPU9250_getTemperature_C(MPU9250_t *mpu9250);

float MPU9250_getAccelX_g(MPU9250_t *mpu9250);
float MPU9250_getAccelY_g(MPU9250_t *mpu9250);
float MPU9250_getAccelZ_g(MPU9250_t *mpu9250);

float MPU9250_getGyroX_degs(MPU9250_t *mpu9250);
float MPU9250_getGyroY_degs(MPU9250_t *mpu9250);
float MPU9250_getGyroZ_degs(MPU9250_t *mpu9250);

int MPU9250_calibrateGyro(MPU9250_t *mpu9250);
float MPU9250_getGyroBiasX_rads(MPU9250_t *mpu9250);
float MPU9250_getGyroBiasY_rads(MPU9250_t *mpu9250);
float MPU9250_getGyroBiasZ_rads(MPU9250_t *mpu9250);
void MPU9250_setGyroBiasX_rads(MPU9250_t *mpu9250, float bias);
void MPU9250_setGyroBiasY_rads(MPU9250_t *mpu9250, float bias);
void MPU9250_setGyroBiasZ_rads(MPU9250_t *mpu9250, float bias);
int MPU9250_calibrateAccel(MPU9250_t *mpu9250);
float MPU9250_getAccelBiasX_mss(MPU9250_t *mpu9250);
float MPU9250_getAccelScaleFactorX(MPU9250_t *mpu9250);
float MPU9250_getAccelBiasY_mss(MPU9250_t *mpu9250);
float MPU9250_getAccelScaleFactorY(MPU9250_t *mpu9250);
float MPU9250_getAccelBiasZ_mss(MPU9250_t *mpu9250);
float MPU9250_getAccelScaleFactorZ(MPU9250_t *mpu9250);
void MPU9250_setAccelCalX(MPU9250_t *mpu9250, float bias, float scaleFactor);
void MPU9250_setAccelCalY(MPU9250_t *mpu9250, float bias, float scaleFactor);
void MPU9250_setAccelCalZ(MPU9250_t *mpu9250, float bias, float scaleFactor);
int MPU9250_calibrateMag(MPU9250_t *mpu9250);
float MPU9250_getMagBiasX_uT(MPU9250_t *mpu9250);
float MPU9250_getMagScaleFactorX(MPU9250_t *mpu9250);
float MPU9250_getMagBiasY_uT(MPU9250_t *mpu9250);
float MPU9250_getMagScaleFactorY(MPU9250_t *mpu9250);
float MPU9250_getMagBiasZ_uT(MPU9250_t *mpu9250);
float MPU9250_getMagScaleFactorZ(MPU9250_t *mpu9250);
void MPU9250_setMagCalX(MPU9250_t *mpu9250, float bias, float scaleFactor);
void MPU9250_setMagCalY(MPU9250_t *mpu9250, float bias, float scaleFactor);
void MPU9250_setMagCalZ(MPU9250_t *mpu9250, float bias, float scaleFactor);

void MPU9250_SelfTest(MPU9250_t * mpu9250, float * destination);
bool MPU9250_getDrdyStatus(MPU9250_t * mpu9250);

// spi
#define SPI_READ 0x80
#define SPI_LS_CLOCK 1000000 // 1 MHz
#define SPI_HS_CLOCK 15000000 // 15 MHz



// MPU9250 registers
#define ACCEL_OUT  0x3B
#define GYRO_OUT  0x43
#define TEMP_OUT  0x41
#define EXT_SENS_DATA_00  0x49
#define ACCEL_CONFIG  0x1C
#define ACCEL_FS_SEL_2G  0x00
#define ACCEL_FS_SEL_4G  0x08
#define ACCEL_FS_SEL_8G  0x10
#define ACCEL_FS_SEL_16G  0x18
#define GYRO_CONFIG  0x1B
#define GYRO_FS_SEL_250DPS  0x00
#define GYRO_FS_SEL_500DPS  0x08
#define GYRO_FS_SEL_1000DPS  0x10
#define GYRO_FS_SEL_2000DPS  0x18
#define ACCEL_CONFIG2  0x1D
#define ACCEL_DLPF_184  0x01
#define ACCEL_DLPF_92  0x02
#define ACCEL_DLPF_41  0x03
#define ACCEL_DLPF_20  0x04
#define ACCEL_DLPF_10  0x05
#define ACCEL_DLPF_5  0x06
#define CONFIG  0x1A
#define GYRO_DLPF_184  0x01
#define GYRO_DLPF_92  0x02
#define GYRO_DLPF_41  0x03
#define GYRO_DLPF_20  0x04
#define GYRO_DLPF_10  0x05
#define GYRO_DLPF_5  0x06
#define SMPDIV  0x19
#define INT_PIN_CFG  0x37
#define INT_ENABLE  0x38
#define INT_DISABLE  0x00
#define INT_PULSE_50US  0x00
#define INT_STATUS 0x3A
#define INT_WOM_EN  0x40
#define INT_RAW_RDY_EN  0x01
#define PWR_MGMNT_1  0x6B
#define PWR_CYCLE  0x20
#define PWR_RESET  0x80
#define CLOCK_SEL_PLL  0x01
#define PWR_MGMNT_2  0x6C
#define SEN_ENABLE  0x00
#define DIS_GYRO  0x07
#define USER_CTRL  0x6A
#define I2C_MST_EN  0x20
#define I2C_MST_CLK  0x0D
#define I2C_MST_CTRL  0x24
#define I2C_SLV0_ADDR  0x25
#define I2C_SLV0_REG  0x26
#define I2C_SLV0_DO  0x63
#define I2C_SLV0_CTRL  0x27
#define I2C_SLV0_EN  0x80
#define I2C_READ_FLAG  0x80
#define MOT_DETECT_CTRL  0x69
#define ACCEL_INTEL_EN  0x80
#define ACCEL_INTEL_MODE  0x40
#define LP_ACCEL_ODR  0x1E
#define WOM_THR  0x1F
#define WHO_AM_I  0x75
#define FIFO_EN  0x23
#define FIFO_TEMP  0x80
#define FIFO_GYRO  0x70
#define FIFO_ACCEL  0x08
#define FIFO_MAG  0x01
#define FIFO_COUNT  0x72
#define FIFO_READ  0x74
// AK8963 registers
#define AK8963_I2C_ADDR  0x0C
#define AK8963_HXL  0x03
#define AK8963_CNTL1  0x0A
#define AK8963_PWR_DOWN  0x00
#define AK8963_CNT_MEAS1  0x12
#define AK8963_CNT_MEAS2  0x16
#define AK8963_FUSE_ROM  0x0F
#define AK8963_CNTL2  0x0B
#define AK8963_RESET  0x01
#define AK8963_ASA  0x10
#define AK8963_WHO_AM_I  0x00

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

int MPU9250_enableFifo(MPU9250FIFO_t * mpu9250fifo, bool accel, bool gyro, bool mag, bool temp);
int MPU9250_readFifo(MPU9250FIFO_t * mpu9250fifo);
void MPU9250_getFifoAccelX_mss(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoAccelY_mss(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoAccelZ_mss(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoGyroX_rads(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoGyroY_rads(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoGyroZ_rads(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoMagX_uT(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoMagY_uT(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoMagZ_uT(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);
void MPU9250_getFifoTemperature_C(MPU9250FIFO_t * mpu9250fifo, size_t * size, float * data);


#endif
