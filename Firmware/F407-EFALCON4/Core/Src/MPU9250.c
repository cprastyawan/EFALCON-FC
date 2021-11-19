#include "MPU9250.h"
#include <stdio.h>

// transformation matrix
/* transform the accel and gyro axes to match the magnetometer axes */
const int16_t tX[3] = { 0, 1, 0 };
const int16_t tY[3] = { 1, 0, 0 };
const int16_t tZ[3] = { 0, 0, -1 };
const float _tempScale = 333.87f;
const float _tempOffset = 21.0f;

// constants
const float G = 9.807f;
const float _d2r = 3.14159265359f / 180.0f;

// private functions
static int writeRegister(MPU9250_t *mpu9250, uint8_t subAddress, uint8_t data) {
	ACTIVATE_SPI(mpu9250->_csPort, mpu9250->_csPin);
	uint8_t data_write[2] = { subAddress, data };
	HAL_SPI_Transmit(mpu9250->_spi, data_write, 2, HAL_MAX_DELAY);
	DEACTIVATE_SPI(mpu9250->_csPort, mpu9250->_csPin);
	return 1;
}

static int readRegisters(MPU9250_t *mpu9250, uint8_t subAddress, uint8_t count,
		uint8_t *dest) {
	ACTIVATE_SPI(mpu9250->_csPort, mpu9250->_csPin);
	uint8_t data_register = subAddress | SPI_READ;
	//HAL_SPI_TransmitReceive(mpu9250->_spi, subAddress | SPI_READ, dest, count, HAL_MAX_DELAY);
	HAL_SPI_Transmit(mpu9250->_spi, &data_register, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(mpu9250->_spi, dest, count, HAL_MAX_DELAY);
	DEACTIVATE_SPI(mpu9250->_csPort, mpu9250->_csPin);
	return 1;
}

static int writeAK8963Register(MPU9250_t *mpu9250, uint8_t subAddress,
		uint8_t data) {
	// set slave 0 to the AK8963 and set for write
	if (writeRegister(mpu9250, I2C_SLV0_ADDR, AK8963_I2C_ADDR) < 0) {
		return -1;
	}
	// set the register to the desired AK8963 sub address
	if (writeRegister(mpu9250, I2C_SLV0_REG, subAddress) < 0) {
		return -2;
	}
	// store the data for write
	if (writeRegister(mpu9250, I2C_SLV0_DO, data) < 0) {
		return -3;
	}
	// enable I2C and send 1 byte
	if (writeRegister(mpu9250, I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t) 1) < 0) {
		return -4;
	}
	return 1;
}

static int readAK8963Registers(MPU9250_t *mpu9250, uint8_t subAddress,
		uint8_t count, uint8_t *dest) {
	// set slave 0 to the AK8963 and set for read
	if (writeRegister(mpu9250, I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG)
			< 0) {
		return -1;
	}
	// set the register to the desired AK8963 sub address
	if (writeRegister(mpu9250, I2C_SLV0_REG, subAddress) < 0) {
		return -2;
	}
	// enable I2C and request the bytes
	if (writeRegister(mpu9250, I2C_SLV0_CTRL, I2C_SLV0_EN | count) < 0) {
		return -3;
	}
	delay(1); // takes some time for these registers to fill
	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	mpu9250->_status = readRegisters(mpu9250, EXT_SENS_DATA_00, count, dest);
	return mpu9250->_status;
}

static int whoAmI(MPU9250_t *mpu9250) {
	if (readRegisters(mpu9250, WHO_AM_I, 1, mpu9250->_buffer) < 0) {
		return -1;
	}
	// return the register value
	return mpu9250->_buffer[0];
}

static int whoAmIAK8963(MPU9250_t *mpu9250) {
	// read the WHO AM I register
	if (readAK8963Registers(mpu9250, AK8963_WHO_AM_I, 1, mpu9250->_buffer)
			< 0) {
		return -1;
	}
	// return the register value
	return mpu9250->_buffer[0];
}

void MPU9250_init(MPU9250_t *mpu9250, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *csPort, uint16_t csPin) {
	mpu9250->_spi = hspi;
	mpu9250->_csPort = csPort;
	mpu9250->_csPin = csPin;

	mpu9250->_numSamples = 100;

	mpu9250->_axs = 1.0f;
	mpu9250->_ays = 1.0f;
	mpu9250->_azs = 1.0f;

	mpu9250->_maxCounts = 1000;
	mpu9250->_deltaThresh = 0.3f;
	mpu9250->_coeff = 8;

	mpu9250->_hxs = 1.0f;
	mpu9250->_hys = 1.0f;
	mpu9250->_hzs = 1.0f;
}

int MPU9250_begin(MPU9250_t *mpu9250) {
	int statusCode = 1;
	DEACTIVATE_SPI(mpu9250->_csPort, mpu9250->_csPin);
	// select clock source to gyro
	if (writeRegister(mpu9250, PWR_MGMNT_1, CLOCK_SEL_PLL) < 0) {
		statusCode = -1;
		goto Done;
	}
	// enable I2C master mode
	if (writeRegister(mpu9250, USER_CTRL, I2C_MST_EN) < 0) {
		statusCode = -2;
		goto Done;
	}
	// set the I2C bus speed to 400 kHz
	if (writeRegister(mpu9250, I2C_MST_CTRL, I2C_MST_CLK) < 0) {
		statusCode = -3;
		goto Done;
	}
	// set AK8963 to Power Down
	writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(mpu9250, PWR_MGMNT_1, PWR_RESET);
	// wait for MPU-9250 to come back up
	delay(1);
	// reset the AK8963
	writeAK8963Register(mpu9250, AK8963_CNTL2, AK8963_RESET);
	// select clock source to gyro
	if (writeRegister(mpu9250, PWR_MGMNT_1, CLOCK_SEL_PLL) < 0) {
		statusCode = -4;
		goto Done;
	}
	int whoami = whoAmI(mpu9250);
	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	if ((whoami != 113) && (whoami != 115)) {
		statusCode = -5;
		goto Done;
	}
	// enable accelerometer and gyro
	if (writeRegister(mpu9250, PWR_MGMNT_2, SEN_ENABLE) < 0) {
		statusCode = -6;
		goto Done;
	}
	// setting accel range to 16G as default
	if (writeRegister(mpu9250, ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0) {
		statusCode = -7;
		goto Done;
	}
	mpu9250->_accelScale = G * 16.0f / 32767.5f; // setting the accel scale to 16G
	mpu9250->_accelRange = ACCEL_RANGE_16G;
	// setting the gyro range to 2000DPS as default
	if (writeRegister(mpu9250, GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0) {
		statusCode = -8;
		goto Done;
	}
	mpu9250->_gyroScale = 2000.0f / 32767.5f * _d2r; // setting the gyro scale to 2000DPS
	mpu9250->_gyroRange = GYRO_RANGE_2000DPS;
	// setting bandwidth to 184Hz as default
	if (writeRegister(mpu9250, ACCEL_CONFIG2, ACCEL_DLPF_184) < 0) {
		statusCode = -9;
		goto Done;
	}
	if (writeRegister(mpu9250, CONFIG, GYRO_DLPF_184) < 0) { // setting gyro bandwidth to 184Hz
		statusCode = -10;
		goto Done;
	}
	mpu9250->_bandwidth = DLPF_BANDWIDTH_184HZ;
	// setting the sample rate divider to 0 as default
	if (writeRegister(mpu9250, SMPDIV, 0x00) < 0) {
		statusCode = -11;
		goto Done;
	}
	mpu9250->_srd = 0;
	// enable I2C master mode
	if (writeRegister(mpu9250, USER_CTRL, I2C_MST_EN) < 0) {
		statusCode = -12;
		goto Done;
	}
	// set the I2C bus speed to 400 kHz
	if (writeRegister(mpu9250, I2C_MST_CTRL, I2C_MST_CLK) < 0) {
		statusCode = -13;
		goto Done;
	}
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	delay(100);
	int whoamiak8963 = whoAmIAK8963(mpu9250);
	if (whoamiak8963 != 72) {
		printf("Magnetometer Error WhoAmI: %d\r\n", whoamiak8963);
		statusCode = -14;
		goto Done;
	}
	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	if (writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
		statusCode = -15;
		goto Done;
	}
	delay(100); // long wait between AK8963 mode changes
	// set AK8963 to FUSE ROM access
	if (writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_FUSE_ROM) < 0) {
		statusCode = -16;
		goto Done;
	}
	delay(100); // long wait between AK8963 mode changes
	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(mpu9250, AK8963_ASA, 3, mpu9250->_buffer);
	mpu9250->_magScaleX = ((((float) mpu9250->_buffer[0]) - 128.0f) / (256.0f)
			+ 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	mpu9250->_magScaleY = ((((float) mpu9250->_buffer[1]) - 128.0f) / (256.0f)
			+ 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	mpu9250->_magScaleZ = ((((float) mpu9250->_buffer[2]) - 128.0f) / (256.0f)
			+ 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	// set AK8963 to Power Down
	if (writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
		statusCode = -17;
		goto Done;
	}
	delay(100); // long wait between AK8963 mode changes
	// set AK8963 to 16 bit resolution, 100 Hz update rate
	if (writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_CNT_MEAS2) < 0) {
		statusCode = -18;
		goto Done;
	}
	delay(100); // long wait between AK8963 mode changes
	// select clock source to gyro
	if (writeRegister(mpu9250, PWR_MGMNT_1, CLOCK_SEL_PLL) < 0) {
		statusCode = -19;
		goto Done;
	}
	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(mpu9250, AK8963_HXL, 7, mpu9250->_buffer);
	// estimate gyro bias
	if (MPU9250_calibrateGyro(mpu9250) < 0) {
		statusCode = -20;
		goto Done;
	}
	// successful init, return 1
	return 1;

	Done:
	printf("MPU9250 Begin error: %d\r\n", statusCode);
	return statusCode;
}

int MPU9250_setAccelRange(MPU9250_t *mpu9250, AccelRange range) {
	switch (range) {
	case ACCEL_RANGE_2G: {
		// setting the accel range to 2G
		if (writeRegister(mpu9250, ACCEL_CONFIG, ACCEL_FS_SEL_2G) < 0) {
			return -1;
		}
		mpu9250->_accelScale = G * 2.0f / 32767.5f; // setting the accel scale to 2G
		break;
	}
	case ACCEL_RANGE_4G: {
		// setting the accel range to 4G
		if (writeRegister(mpu9250, ACCEL_CONFIG, ACCEL_FS_SEL_4G) < 0) {
			return -1;
		}
		mpu9250->_accelScale = G * 4.0f / 32767.5f; // setting the accel scale to 4G
		break;
	}
	case ACCEL_RANGE_8G: {
		// setting the accel range to 8G
		if (writeRegister(mpu9250, ACCEL_CONFIG, ACCEL_FS_SEL_8G) < 0) {
			return -1;
		}
		mpu9250->_accelScale = G * 8.0f / 32767.5f; // setting the accel scale to 8G
		break;
	}
	case ACCEL_RANGE_16G: {
		// setting the accel range to 16G
		if (writeRegister(mpu9250, ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0) {
			return -1;
		}
		mpu9250->_accelScale = G * 16.0f / 32767.5f; // setting the accel scale to 16G
		break;
	}
	}
	mpu9250->_accelRange = range;
	return 1;
}

int MPU9250_setGyroRange(MPU9250_t *mpu9250, GyroRange range) {
	switch (range) {
	case GYRO_RANGE_250DPS: {
		// setting the gyro range to 250DPS
		if (writeRegister(mpu9250, GYRO_CONFIG, GYRO_FS_SEL_250DPS) < 0) {
			return -1;
		}
		mpu9250->_gyroScale = 250.0f / 32767.5f * _d2r; // setting the gyro scale to 250DPS
		break;
	}
	case GYRO_RANGE_500DPS: {
		// setting the gyro range to 500DPS
		if (writeRegister(mpu9250, GYRO_CONFIG, GYRO_FS_SEL_500DPS) < 0) {
			return -1;
		}
		mpu9250->_gyroScale = 500.0f / 32767.5f * _d2r; // setting the gyro scale to 500DPS
		break;
	}
	case GYRO_RANGE_1000DPS: {
		// setting the gyro range to 1000DPS
		if (writeRegister(mpu9250, GYRO_CONFIG, GYRO_FS_SEL_1000DPS) < 0) {
			return -1;
		}
		mpu9250->_gyroScale = 1000.0f / 32767.5f * _d2r; // setting the gyro scale to 1000DPS
		break;
	}
	case GYRO_RANGE_2000DPS: {
		// setting the gyro range to 2000DPS
		if (writeRegister(mpu9250, GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0) {
			return -1;
		}
		mpu9250->_gyroScale = 2000.0f / 32767.5f * _d2r; // setting the gyro scale to 2000DPS
		break;
	}
	}
	mpu9250->_gyroRange = range;
	return 1;
}

int MPU9250_setDlpfBandwidth(MPU9250_t *mpu9250, DlpfBandwidth bandwidth) {
	switch (bandwidth) {
	case DLPF_BANDWIDTH_184HZ: {
		if (writeRegister(mpu9250, ACCEL_CONFIG2, ACCEL_DLPF_184) < 0) { // setting accel bandwidth to 184Hz
			return -1;
		}
		if (writeRegister(mpu9250, CONFIG, GYRO_DLPF_184) < 0) { // setting gyro bandwidth to 184Hz
			return -2;
		}
		break;
	}
	case DLPF_BANDWIDTH_92HZ: {
		if (writeRegister(mpu9250, ACCEL_CONFIG2, ACCEL_DLPF_92) < 0) { // setting accel bandwidth to 92Hz
			return -1;
		}
		if (writeRegister(mpu9250, CONFIG, GYRO_DLPF_92) < 0) { // setting gyro bandwidth to 92Hz
			return -2;
		}
		break;
	}
	case DLPF_BANDWIDTH_41HZ: {
		if (writeRegister(mpu9250, ACCEL_CONFIG2, ACCEL_DLPF_41) < 0) { // setting accel bandwidth to 41Hz
			return -1;
		}
		if (writeRegister(mpu9250, CONFIG, GYRO_DLPF_41) < 0) { // setting gyro bandwidth to 41Hz
			return -2;
		}
		break;
	}
	case DLPF_BANDWIDTH_20HZ: {
		if (writeRegister(mpu9250, ACCEL_CONFIG2, ACCEL_DLPF_20) < 0) { // setting accel bandwidth to 20Hz
			return -1;
		}
		if (writeRegister(mpu9250, CONFIG, GYRO_DLPF_20) < 0) { // setting gyro bandwidth to 20Hz
			return -2;
		}
		break;
	}
	case DLPF_BANDWIDTH_10HZ: {
		if (writeRegister(mpu9250, ACCEL_CONFIG2, ACCEL_DLPF_10) < 0) { // setting accel bandwidth to 10Hz
			return -1;
		}
		if (writeRegister(mpu9250, CONFIG, GYRO_DLPF_10) < 0) { // setting gyro bandwidth to 10Hz
			return -2;
		}
		break;
	}
	case DLPF_BANDWIDTH_5HZ: {
		if (writeRegister(mpu9250, ACCEL_CONFIG2, ACCEL_DLPF_5) < 0) { // setting accel bandwidth to 5Hz
			return -1;
		}
		if (writeRegister(mpu9250, CONFIG, GYRO_DLPF_5) < 0) { // setting gyro bandwidth to 5Hz
			return -2;
		}
		break;
	}
	}
	mpu9250->_bandwidth = bandwidth;
	return 1;
}

int MPU9250_setSrd(MPU9250_t *mpu9250, uint8_t srd) {
	if (writeRegister(mpu9250, SMPDIV, 19) < 0) { // setting the sample rate divider
		return -1;
	}
	if (srd > 9) {
		// set AK8963 to Power Down
		if (writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
			return -2;
		}
		delay(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 8 Hz update rate
		if (writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_CNT_MEAS1) < 0) {
			return -3;
		}
		delay(100); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(mpu9250, AK8963_HXL, 7, mpu9250->_buffer);
	} else {
		// set AK8963 to Power Down
		if (writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_PWR_DOWN) < 0) {
			return -2;
		}
		delay(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		if (writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_CNT_MEAS2) < 0) {
			return -3;
		}
		delay(100); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(mpu9250, AK8963_HXL, 7, mpu9250->_buffer);
	}
	/* setting the sample rate divider */
	if (writeRegister(mpu9250, SMPDIV, srd) < 0) { // setting the sample rate divider
		return -4;
	}
	mpu9250->_srd = srd;
	return 1;
}

int MPU9250_enableDataReadyInterrupt(MPU9250_t *mpu9250) {
	if (writeRegister(mpu9250, INT_PIN_CFG, INT_PULSE_50US) < 0) { // setup interrupt, 50 us pulse
		return -1;
	}
	if (writeRegister(mpu9250, INT_ENABLE, INT_RAW_RDY_EN) < 0) { // set to data ready
		return -2;
	}
	return 1;
}

int MPU9250_disableDataReadyInterrupt(MPU9250_t *mpu9250) {
	if (writeRegister(mpu9250, INT_ENABLE, INT_DISABLE) < 0) { // disable interrupt
		return -1;
	}
	return 1;
}

int MPU9250_enableWakeOnMotion(MPU9250_t *mpu9250, float womThresh_mg,
		LpAccelOdr odr) {
	writeAK8963Register(mpu9250, AK8963_CNTL1, AK8963_PWR_DOWN);
	// reset the MPU9250
	writeRegister(mpu9250, PWR_MGMNT_1, PWR_RESET);
	// wait for MPU-9250 to come back up
	delay(1);
	if (writeRegister(mpu9250, PWR_MGMNT_1, 0x00) < 0) { // cycle 0, sleep 0, standby 0
		return -1;
	}
	if (writeRegister(mpu9250, PWR_MGMNT_2, DIS_GYRO) < 0) { // disable gyro measurements
		return -2;
	}
	if (writeRegister(mpu9250, ACCEL_CONFIG2, ACCEL_DLPF_184) < 0) { // setting accel bandwidth to 184Hz
		return -3;
	}
	if (writeRegister(mpu9250, INT_ENABLE, INT_WOM_EN) < 0) { // enabling interrupt to wake on motion
		return -4;
	}
	if (writeRegister(mpu9250, MOT_DETECT_CTRL,
			(ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0) { // enabling accel hardware intelligence
		return -5;
	}
	mpu9250->_womThreshold = map(womThresh_mg, 0, 1020, 0, 255);
	if (writeRegister(mpu9250, WOM_THR, mpu9250->_womThreshold) < 0) { // setting wake on motion threshold
		return -6;
	}
	if (writeRegister(mpu9250, LP_ACCEL_ODR, (uint8_t) odr) < 0) { // set frequency of wakeup
		return -7;
	}
	if (writeRegister(mpu9250, PWR_MGMNT_1, PWR_CYCLE) < 0) { // switch to accel low power mode
		return -8;
	}
	return 1;
}

int MPU9250_readSensor(MPU9250_t *mpu9250) {
	// grab the data from the MPU9250
	if (readRegisters(mpu9250, ACCEL_OUT, 21, mpu9250->_buffer) < 0) {
		return -1;
	}
	// combine into 16 bit values
	mpu9250->_axcounts = (((int16_t) mpu9250->_buffer[0]) << 8)
			| mpu9250->_buffer[1];
	mpu9250->_aycounts = (((int16_t) mpu9250->_buffer[2]) << 8)
			| mpu9250->_buffer[3];
	mpu9250->_azcounts = (((int16_t) mpu9250->_buffer[4]) << 8)
			| mpu9250->_buffer[5];
	mpu9250->_tcounts = (((int16_t) mpu9250->_buffer[6]) << 8)
			| mpu9250->_buffer[7];
	mpu9250->_gxcounts = (((int16_t) mpu9250->_buffer[8]) << 8)
			| mpu9250->_buffer[9];
	mpu9250->_gycounts = (((int16_t) mpu9250->_buffer[10]) << 8)
			| mpu9250->_buffer[11];
	mpu9250->_gzcounts = (((int16_t) mpu9250->_buffer[12]) << 8)
			| mpu9250->_buffer[13];
	mpu9250->_hxcounts = (((int16_t) mpu9250->_buffer[15]) << 8)
			| mpu9250->_buffer[14];
	mpu9250->_hycounts = (((int16_t) mpu9250->_buffer[17]) << 8)
			| mpu9250->_buffer[16];
	mpu9250->_hzcounts = (((int16_t) mpu9250->_buffer[19]) << 8)
			| mpu9250->_buffer[18];
	// transform and convert to float values
	mpu9250->_ax = (((float) (tX[0] * mpu9250->_axcounts
			+ tX[1] * mpu9250->_aycounts + tX[2] * mpu9250->_azcounts)
			* mpu9250->_accelScale) - mpu9250->_axb) * mpu9250->_axs;
	mpu9250->_ay = (((float) (tY[0] * mpu9250->_axcounts
			+ tY[1] * mpu9250->_aycounts + tY[2] * mpu9250->_azcounts)
			* mpu9250->_accelScale) - mpu9250->_ayb) * mpu9250->_ays;
	mpu9250->_az = (((float) (tZ[0] * mpu9250->_axcounts
			+ tZ[1] * mpu9250->_aycounts + tZ[2] * mpu9250->_azcounts)
			* mpu9250->_accelScale) - mpu9250->_azb) * mpu9250->_azs;
	mpu9250->_gx = ((float) (tX[0] * mpu9250->_gxcounts
			+ tX[1] * mpu9250->_gycounts + tX[2] * mpu9250->_gzcounts)
			* mpu9250->_gyroScale) - mpu9250->_gxb;
	mpu9250->_gy = ((float) (tY[0] * mpu9250->_gxcounts
			+ tY[1] * mpu9250->_gycounts + tY[2] * mpu9250->_gzcounts)
			* mpu9250->_gyroScale) - mpu9250->_gyb;
	mpu9250->_gz = ((float) (tZ[0] * mpu9250->_gxcounts
			+ tZ[1] * mpu9250->_gycounts + tZ[2] * mpu9250->_gzcounts)
			* mpu9250->_gyroScale) - mpu9250->_gzb;
	mpu9250->_hx = (((float) (mpu9250->_hxcounts) * mpu9250->_magScaleX)
			- mpu9250->_hxb) * mpu9250->_hxs;
	mpu9250->_hy = (((float) (mpu9250->_hycounts) * mpu9250->_magScaleY)
			- mpu9250->_hyb) * mpu9250->_hys;
	mpu9250->_hz = (((float) (mpu9250->_hzcounts) * mpu9250->_magScaleZ)
			- mpu9250->_hzb) * mpu9250->_hzs;
	mpu9250->_t = ((((float) mpu9250->_tcounts) - _tempOffset) / _tempScale)
			+ _tempOffset;
	return 1;
}

float MPU9250_getAccelX_mss(MPU9250_t *mpu9250) {
	return mpu9250->_ax;
}

float MPU9250_getAccelY_mss(MPU9250_t *mpu9250) {
	return mpu9250->_ay;
}

float MPU9250_getAccelZ_mss(MPU9250_t *mpu9250) {
	return mpu9250->_az;
}

float MPU9250_getGyroX_rads(MPU9250_t *mpu9250) {
	return mpu9250->_gx;
}

float MPU9250_getGyroY_rads(MPU9250_t *mpu9250) {
	return mpu9250->_gy;
}

float MPU9250_getGyroZ_rads(MPU9250_t *mpu9250) {
	return mpu9250->_gz;
}

float MPU9250_getGyroX_degs(MPU9250_t *mpu9250) {
	return mpu9250->_gx * RAD_TO_DEG;
}

float MPU9250_getGyroY_degs(MPU9250_t *mpu9250) {
	return mpu9250->_gy * RAD_TO_DEG;
}

float MPU9250_getGyroZ_degs(MPU9250_t *mpu9250) {
	return mpu9250->_gz * RAD_TO_DEG;
}

float MPU9250_getMagX_uT(MPU9250_t *mpu9250) {
	return mpu9250->_hx;
}

float MPU9250_getMagY_uT(MPU9250_t *mpu9250) {
	return mpu9250->_hy;
}

float MPU9250_getMagZ_uT(MPU9250_t *mpu9250) {
	return mpu9250->_hz;
}

float MPU9250_getTemperature_C(MPU9250_t *mpu9250) {
	return mpu9250->_t;
}

int MPU9250_calibrateGyro(MPU9250_t *mpu9250) {
	if (MPU9250_setGyroRange(mpu9250, GYRO_RANGE_250DPS) < 0) {
		return -1;
	}
	if (MPU9250_setDlpfBandwidth(mpu9250, DLPF_BANDWIDTH_20HZ) < 0) {
		return -2;
	}
	if (MPU9250_setSrd(mpu9250, 19) < 0) {
		return -3;
	}

	// take samples and find bias
	mpu9250->_gxbD = 0;
	mpu9250->_gybD = 0;
	mpu9250->_gzbD = 0;
	for (size_t i = 0; i < mpu9250->_numSamples; i++) {
		MPU9250_readSensor(mpu9250);
		mpu9250->_gxbD += (MPU9250_getGyroX_rads(mpu9250) + mpu9250->_gxb)
				/ ((double) mpu9250->_numSamples);
		mpu9250->_gybD += (MPU9250_getGyroY_rads(mpu9250) + mpu9250->_gyb)
				/ ((double) mpu9250->_numSamples);
		mpu9250->_gzbD += (MPU9250_getGyroZ_rads(mpu9250) + mpu9250->_gzb)
				/ ((double) mpu9250->_numSamples);
		delay(20);
	}
	mpu9250->_gxb = (float) mpu9250->_gxbD;
	mpu9250->_gyb = (float) mpu9250->_gybD;
	mpu9250->_gzb = (float) mpu9250->_gzbD;

	// set the range, bandwidth, and srd back to what they were
	if (MPU9250_setGyroRange(mpu9250, mpu9250->_gyroRange) < 0) {
		return -4;
	}
	if (MPU9250_setDlpfBandwidth(mpu9250, mpu9250->_bandwidth) < 0) {
		return -5;
	}
	if (MPU9250_setSrd(mpu9250, mpu9250->_srd) < 0) {
		return -6;
	}
	return 1;
}

float MPU9250_getGyroBiasX_rads(MPU9250_t *mpu9250) {
	return mpu9250->_gxb;
}

float MPU9250_getGyroBiasY_rads(MPU9250_t *mpu9250) {
	return mpu9250->_gyb;
}

float MPU9250_getGyroBiasZ_rads(MPU9250_t *mpu9250) {
	return mpu9250->_gzb;
}

void MPU9250_setGyroBiasX_rads(MPU9250_t *mpu9250, float bias) {
	mpu9250->_gxb = bias;
}

void MPU9250_setGyroBiasY_rads(MPU9250_t *mpu9250, float bias) {
	mpu9250->_gyb = bias;
}

void MPU9250_setGyroBiasZ_rads(MPU9250_t *mpu9250, float bias) {
	mpu9250->_gzb = bias;
}

int MPU9250_calibrateAccel(MPU9250_t *mpu9250) {
	// set the range, bandwidth, and srd
	if (MPU9250_setAccelRange(mpu9250, ACCEL_RANGE_2G) < 0) {
		return -1;
	}
	if (MPU9250_setDlpfBandwidth(mpu9250, DLPF_BANDWIDTH_20HZ) < 0) {
		return -2;
	}
	if (MPU9250_setSrd(mpu9250, 19) < 0) {
		return -3;
	}

	// take samples and find min / max
	mpu9250->_axbD = 0;
	mpu9250->_aybD = 0;
	mpu9250->_azbD = 0;
	for (size_t i = 0; i < mpu9250->_numSamples; i++) {
		MPU9250_readSensor(mpu9250);
		mpu9250->_axbD += (MPU9250_getAccelX_mss(mpu9250) / mpu9250->_axs
				+ mpu9250->_axb) / ((double) mpu9250->_numSamples);
		mpu9250->_aybD += (MPU9250_getAccelY_mss(mpu9250) / mpu9250->_ays
				+ mpu9250->_ayb) / ((double) mpu9250->_numSamples);
		mpu9250->_azbD += (MPU9250_getAccelZ_mss(mpu9250) / mpu9250->_azs
				+ mpu9250->_azb) / ((double) mpu9250->_numSamples);
		delay(20);
	}
	if (mpu9250->_axbD > 9.0f) {
		mpu9250->_axmax = (float) mpu9250->_axbD;
	}
	if (mpu9250->_aybD > 9.0f) {
		mpu9250->_aymax = (float) mpu9250->_aybD;
	}
	if (mpu9250->_azbD > 9.0f) {
		mpu9250->_azmax = (float) mpu9250->_azbD;
	}
	if (mpu9250->_axbD < -9.0f) {
		mpu9250->_axmin = (float) mpu9250->_axbD;
	}
	if (mpu9250->_aybD < -9.0f) {
		mpu9250->_aymin = (float) mpu9250->_aybD;
	}
	if (mpu9250->_azbD < -9.0f) {
		mpu9250->_azmin = (float) mpu9250->_azbD;
	}

	// find bias and scale factor
	if ((fabs(mpu9250->_axmin) > 9.0f) && (fabs(mpu9250->_axmax) > 9.0f)) {
		mpu9250->_axb = (mpu9250->_axmin + mpu9250->_axmax) / 2.0f;
		mpu9250->_axs = G
				/ ((fabs(mpu9250->_axmin) + fabs(mpu9250->_axmax)) / 2.0f);
	}
	if ((fabs(mpu9250->_aymin) > 9.0f) && (fabs(mpu9250->_aymax) > 9.0f)) {
		mpu9250->_ayb = (mpu9250->_aymin + mpu9250->_aymax) / 2.0f;
		mpu9250->_ays = G
				/ ((fabs(mpu9250->_aymin) + fabs(mpu9250->_aymax)) / 2.0f);
	}
	if ((fabs(mpu9250->_azmin) > 9.0f) && (fabs(mpu9250->_azmax) > 9.0f)) {
		mpu9250->_azb = (mpu9250->_azmin + mpu9250->_azmax) / 2.0f;
		mpu9250->_azs = G
				/ ((fabs(mpu9250->_azmin) + fabs(mpu9250->_azmax)) / 2.0f);
	}

	// set the range, bandwidth, and srd back to what they were
	if (MPU9250_setAccelRange(mpu9250, mpu9250->_accelRange) < 0) {
		return -4;
	}
	if (MPU9250_setDlpfBandwidth(mpu9250, mpu9250->_bandwidth) < 0) {
		return -5;
	}
	if (MPU9250_setSrd(mpu9250, mpu9250->_srd) < 0) {
		return -6;
	}
	return 1;
}

float MPU9250_getAccelBiasX_mss(MPU9250_t *mpu9250) {
	return mpu9250->_axb;
}

float MPU9250_getAccelScaleFactorX(MPU9250_t *mpu9250) {
	return mpu9250->_axs;
}

float MPU9250_getAccelBiasY_mss(MPU9250_t *mpu9250) {
	return mpu9250->_ayb;
}

float MPU9250_getAccelScaleFactorY(MPU9250_t *mpu9250) {
	return mpu9250->_ays;
}

float MPU9250_getAccelBiasZ_mss(MPU9250_t *mpu9250) {
	return mpu9250->_azb;
}

float MPU9250_getAccelScaleFactorZ(MPU9250_t *mpu9250) {
	return mpu9250->_azs;
}

void MPU9250_setAccelCalX(MPU9250_t *mpu9250, float bias, float scaleFactor) {
	mpu9250->_axb = bias;
	mpu9250->_axs = scaleFactor;
}

void MPU9250_setAccelCalY(MPU9250_t *mpu9250, float bias, float scaleFactor) {
	mpu9250->_ayb = bias;
	mpu9250->_ays = scaleFactor;
}

void MPU9250_setAccelCalZ(MPU9250_t *mpu9250, float bias, float scaleFactor) {
	mpu9250->_ayb = bias;
	mpu9250->_ays = scaleFactor;
}

int MPU9250_calibrateMag(MPU9250_t *mpu9250) {
	// set the srd
	if (MPU9250_setSrd(mpu9250, 19) < 0) {
		return -1;
	}
	// get a starting set of data
	MPU9250_readSensor(mpu9250);
	mpu9250->_hxmax = MPU9250_getMagX_uT(mpu9250);
	mpu9250->_hxmin = MPU9250_getMagX_uT(mpu9250);
	mpu9250->_hymax = MPU9250_getMagY_uT(mpu9250);
	mpu9250->_hymin = MPU9250_getMagY_uT(mpu9250);
	mpu9250->_hzmax = MPU9250_getMagZ_uT(mpu9250);
	mpu9250->_hzmin = MPU9250_getMagZ_uT(mpu9250);

	// collect data to find max / min in each channel
	mpu9250->_counter = 0;
	while (mpu9250->_counter < mpu9250->_maxCounts) {
		mpu9250->_delta = 0.0f;
		mpu9250->_framedelta = 0.0f;
		MPU9250_readSensor(mpu9250);
		mpu9250->_hxfilt = (mpu9250->_hxfilt * ((float) mpu9250->_coeff - 1)
				+ (MPU9250_getMagX_uT(mpu9250) / mpu9250->_hxs + mpu9250->_hxb))
				/ ((float) mpu9250->_coeff);
		mpu9250->_hyfilt = (mpu9250->_hyfilt * ((float) mpu9250->_coeff - 1)
				+ (MPU9250_getMagY_uT(mpu9250) / mpu9250->_hys + mpu9250->_hyb))
				/ ((float) mpu9250->_coeff);
		mpu9250->_hzfilt = (mpu9250->_hzfilt * ((float) mpu9250->_coeff - 1)
				+ (MPU9250_getMagZ_uT(mpu9250) / mpu9250->_hzs + mpu9250->_hzb))
				/ ((float) mpu9250->_coeff);
		if (mpu9250->_hxfilt > mpu9250->_hxmax) {
			mpu9250->_delta = mpu9250->_hxfilt - mpu9250->_hxmax;
			mpu9250->_hxmax = mpu9250->_hxfilt;
		}
		if (mpu9250->_delta > mpu9250->_framedelta) {
			mpu9250->_framedelta = mpu9250->_delta;
		}
		if (mpu9250->_hyfilt > mpu9250->_hymax) {
			mpu9250->_delta = mpu9250->_hyfilt - mpu9250->_hymax;
			mpu9250->_hymax = mpu9250->_hyfilt;
		}
		if (mpu9250->_delta > mpu9250->_framedelta) {
			mpu9250->_framedelta = mpu9250->_delta;
		}
		if (mpu9250->_hzfilt > mpu9250->_hzmax) {
			mpu9250->_delta = mpu9250->_hzfilt - mpu9250->_hzmax;
			mpu9250->_hzmax = mpu9250->_hzfilt;
		}
		if (mpu9250->_delta > mpu9250->_framedelta) {
			mpu9250->_framedelta = mpu9250->_delta;
		}
		if (mpu9250->_hxfilt < mpu9250->_hxmin) {
			mpu9250->_delta = fabs(mpu9250->_hxfilt - mpu9250->_hxmin);
			mpu9250->_hxmin = mpu9250->_hxfilt;
		}
		if (mpu9250->_delta > mpu9250->_framedelta) {
			mpu9250->_framedelta = mpu9250->_delta;
		}
		if (mpu9250->_hyfilt < mpu9250->_hymin) {
			mpu9250->_delta = fabs(mpu9250->_hyfilt - mpu9250->_hymin);
			mpu9250->_hymin = mpu9250->_hyfilt;
		}
		if (mpu9250->_delta > mpu9250->_framedelta) {
			mpu9250->_framedelta = mpu9250->_delta;
		}
		if (mpu9250->_hzfilt < mpu9250->_hzmin) {
			mpu9250->_delta = fabs(mpu9250->_hzfilt - mpu9250->_hzmin);
			mpu9250->_hzmin = mpu9250->_hzfilt;
		}
		if (mpu9250->_delta > mpu9250->_framedelta) {
			mpu9250->_framedelta = mpu9250->_delta;
		}
		if (mpu9250->_framedelta > mpu9250->_deltaThresh) {
			mpu9250->_counter = 0;
		} else {
			mpu9250->_counter++;
		}
		delay(20);
	}

	// find the magnetometer bias
	mpu9250->_hxb = (mpu9250->_hxmax + mpu9250->_hxmin) / 2.0f;
	mpu9250->_hyb = (mpu9250->_hymax + mpu9250->_hymin) / 2.0f;
	mpu9250->_hzb = (mpu9250->_hzmax + mpu9250->_hzmin) / 2.0f;

	// find the magnetometer scale factor
	mpu9250->_hxs = (mpu9250->_hxmax - mpu9250->_hxmin) / 2.0f;
	mpu9250->_hys = (mpu9250->_hymax - mpu9250->_hymin) / 2.0f;
	mpu9250->_hzs = (mpu9250->_hzmax - mpu9250->_hzmin) / 2.0f;
	mpu9250->_avgs = (mpu9250->_hxs + mpu9250->_hys + mpu9250->_hzs) / 3.0f;
	mpu9250->_hxs = mpu9250->_avgs / mpu9250->_hxs;
	mpu9250->_hys = mpu9250->_avgs / mpu9250->_hys;
	mpu9250->_hzs = mpu9250->_avgs / mpu9250->_hzs;

	// set the srd back to what it was
	if (MPU9250_setSrd(mpu9250, mpu9250->_srd) < 0) {
		return -2;
	}
	return 1;
}

void MPU9250_SelfTest(MPU9250_t *mpu9250, float *destination) {
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] =
			{ 0 };
	float factoryTrim[6];
	uint8_t FS = GYRO_RANGE_250DPS;

	writeRegister(mpu9250, SMPDIV, 0x00); // Set gyro sample rate to 1 kHz
	writeRegister(mpu9250, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeRegister(mpu9250, GYRO_CONFIG, FS << 3); // Set full scale range for the gyro to 250 dps
	writeRegister(mpu9250, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeRegister(mpu9250, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

		readRegisters(mpu9250, ACCEL_OUT, 6, &rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		readRegisters(mpu9250, GYRO_OUT, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	// Get average of 200 values and store as average current readings
	for (int ii = 0; ii < 3; ii++) {
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	// Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeRegister(mpu9250, ACCEL_CONFIG, 0xE0);
	// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	writeRegister(mpu9250, GYRO_CONFIG, 0xE0);
	delay(25); // Delay a while to let the device stabilize

	// Get average self-test values of gyro and acclerometer
	for (int ii = 0; ii < 200; ii++) {
		// Read the six raw data registers into data array
		readRegisters(mpu9250, ACCEL_OUT, 6, &rawData[0]);
		// Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);
		aSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		// Read the six raw data registers sequentially into data array
		readRegisters(mpu9250, GYRO_OUT, 6, &rawData[0]);
		// Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);
		gSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	// Get average of 200 values and store as average self-test readings
	for (int ii = 0; ii < 3; ii++) {
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeRegister(mpu9250, ACCEL_CONFIG, 0x00);
	writeRegister(mpu9250, GYRO_CONFIG, 0x00);
	delay(25); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	// X-axis accel self-test results
	readRegisters(mpu9250, SELF_TEST_X_ACCEL, 1, &selfTest[0]);
	// Y-axis accel self-test results
	readRegisters(mpu9250, SELF_TEST_Y_ACCEL, 1, &selfTest[1]);
	// Z-axis accel self-test results
	readRegisters(mpu9250, SELF_TEST_Z_ACCEL, 1, &selfTest[2]);
	// X-axis gyro self-test results
	readRegisters(mpu9250, SELF_TEST_X_GYRO, 1, &selfTest[3]);
	// Y-axis gyro self-test results
	readRegisters(mpu9250, SELF_TEST_Y_GYRO, 1, &selfTest[4]);
	// Z-axis gyro self-test results
	readRegisters(mpu9250, SELF_TEST_Z_GYRO, 1, &selfTest[5]);

	// Retrieve factory self-test value from self-test code reads
	// FT[Xa] factory trim calculation
	factoryTrim[0] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[0] - 1.0)));
	// FT[Ya] factory trim calculation
	factoryTrim[1] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[1] - 1.0)));
	// FT[Za] factory trim calculation
	factoryTrim[2] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[2] - 1.0)));
	// FT[Xg] factory trim calculation
	factoryTrim[3] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[3] - 1.0)));
	// FT[Yg] factory trim calculation
	factoryTrim[4] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[4] - 1.0)));
	// FT[Zg] factory trim calculation
	factoryTrim[5] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[5] - 1.0)));

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
	// of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		// Report percent differences
		destination[i] = 100.0 * ((float) (aSTAvg[i] - aAvg[i]))
				/ factoryTrim[i] - 100.;
		// Report percent differences
		destination[i + 3] = 100.0 * ((float) (gSTAvg[i] - gAvg[i]))
				/ factoryTrim[i + 3] - 100.;
	}
}

bool MPU9250_getDrdyStatus(MPU9250_t *mpu9250) {
	uint8_t data;
	readRegisters(mpu9250, INT_STATUS, 1, &data);

	if (data & 0x01)
		return true;
	else
		return false;
}

float MPU9250_getMagBiasX_uT(MPU9250_t *mpu9250) {
	return mpu9250->_hxb;
}

float MPU9250_getMagScaleFactorX(MPU9250_t *mpu9250) {
	return mpu9250->_hxs;
}

float MPU9250_getMagBiasY_uT(MPU9250_t *mpu9250) {
	return mpu9250->_hyb;
}

float MPU9250_getMagScaleFactorY(MPU9250_t *mpu9250) {
	return mpu9250->_hys;
}

float MPU9250_getMagBiasZ_uT(MPU9250_t *mpu9250) {
	return mpu9250->_hzb;
}

float MPU9250_getMagScaleFactorZ(MPU9250_t *mpu9250) {
	return mpu9250->_hzs;
}

void MPU9250_setMagCalX(MPU9250_t *mpu9250, float bias, float scaleFactor) {
	mpu9250->_hxb = bias;
	mpu9250->_hxs = scaleFactor;
}

void MPU9250_setMagCalY(MPU9250_t *mpu9250, float bias, float scaleFactor) {
	mpu9250->_hyb = bias;
	mpu9250->_hys = scaleFactor;
}

void MPU9250_setMagCalZ(MPU9250_t *mpu9250, float bias, float scaleFactor) {
	mpu9250->_hzb = bias;
	mpu9250->_hzs = scaleFactor;
}

int MPU9250_enableFifo(MPU9250FIFO_t *mpu9250fifo, bool accel, bool gyro,
		bool mag, bool temp) {
	if (writeRegister(mpu9250fifo->mpu9250, USER_CTRL, (0x40 | I2C_MST_EN))
			< 0) {
		return -1;
	}
	if (writeRegister(mpu9250fifo->mpu9250, FIFO_EN,
			(accel * FIFO_ACCEL) | (gyro * FIFO_GYRO) | (mag * FIFO_MAG)
					| (temp * FIFO_TEMP)) < 0) {
		return -2;
	}
	mpu9250fifo->_enFifoAccel = accel;
	mpu9250fifo->_enFifoGyro = gyro;
	mpu9250fifo->_enFifoMag = mag;
	mpu9250fifo->_enFifoTemp = temp;
	mpu9250fifo->_fifoFrameSize = accel * 6 + gyro * 6 + mag * 7 + temp * 2;
	return 1;
}

int MPU9250_readFifo(MPU9250FIFO_t *mpu9250fifo) {
	return 0;
}

float MPU9250_getAccelX_g(MPU9250_t *mpu9250) {
	return (MPU9250_getAccelX_mss(mpu9250) / G);
}

float MPU9250_getAccelY_g(MPU9250_t *mpu9250) {
	return (MPU9250_getAccelY_mss(mpu9250) / G);
}
float MPU9250_getAccelZ_g(MPU9250_t *mpu9250) {
	return (MPU9250_getAccelZ_mss(mpu9250) / G);
}

void MPU9250_getFifoAccelX_mss(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

void MPU9250_getFifoAccelY_mss(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

void MPU9250_getFifoAccelZ_mss(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

void MPU9250_getFifoGyroX_rads(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}
void MPU9250_getFifoGyroY_rads(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

void MPU9250_getFifoGyroZ_rads(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

void MPU9250_getFifoMagX_uT(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

void MPU9250_getFifoMagY_uT(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

void MPU9250_getFifoMagZ_uT(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

void MPU9250_getFifoTemperature_C(MPU9250FIFO_t *mpu9250fifo, size_t *size,
		float *data) {

}

