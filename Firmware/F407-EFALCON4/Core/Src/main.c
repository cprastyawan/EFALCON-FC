/* USER CODE BEGIN Header */
/* Tidak ada yang penting, semua akan berakhir, semua akan kembali
 * Hidup untuk orang orang yang tetap hidup
 * Ada yang ingin ditanyakan?
 * IG: @cprastyawan
 * */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "MPU9250.h"
#include "IMU.h"
#include "Fusion.h"
#include "bmp2.h"
#include "retarget.h"
#include "bmp2_app.h"
#include "Kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	FLY_MODE_OFF = 0b0001, FLY_MODE_ON = 0b0011, FLY_MODE_HOLD = 0b0111
} FLY_MODE_t;

typedef enum {
	LANDING_MODE_MANUAL = 0b0001, LANDING_MODE_AUTO = 0b0011
} LANDING_MODE_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MainBuf_SIZE 256
#define RxBuf_SIZE 128
#define SERIAL_DEBUG huart2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Remote_Task */
osThreadId_t Remote_TaskHandle;
const osThreadAttr_t Remote_Task_attributes = {
  .name = "Remote_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for PrintSerial_Tas */
osThreadId_t PrintSerial_TasHandle;
const osThreadAttr_t PrintSerial_Tas_attributes = {
  .name = "PrintSerial_Tas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BMP_Task */
osThreadId_t BMP_TaskHandle;
const osThreadAttr_t BMP_Task_attributes = {
  .name = "BMP_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_Controller */
osThreadId_t Task_ControllerHandle;
const osThreadAttr_t Task_Controller_attributes = {
  .name = "Task_Controller",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for SerialRecv_Task */
osThreadId_t SerialRecv_TaskHandle;
const osThreadAttr_t SerialRecv_Task_attributes = {
  .name = "SerialRecv_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPI1_Mutex */
osMutexId_t SPI1_MutexHandle;
const osMutexAttr_t SPI1_Mutex_attributes = {
  .name = "SPI1_Mutex"
};
/* Definitions for SerialDebug_Mutex */
osMutexId_t SerialDebug_MutexHandle;
const osMutexAttr_t SerialDebug_Mutex_attributes = {
  .name = "SerialDebug_Mutex"
};
/* Definitions for Fly_Event */
osEventFlagsId_t Fly_EventHandle;
const osEventFlagsAttr_t Fly_Event_attributes = {
  .name = "Fly_Event"
};
/* Definitions for Landing_Event */
osEventFlagsId_t Landing_EventHandle;
const osEventFlagsAttr_t Landing_Event_attributes = {
  .name = "Landing_Event"
};
/* Definitions for Calibration_Event */
osEventFlagsId_t Calibration_EventHandle;
const osEventFlagsAttr_t Calibration_Event_attributes = {
  .name = "Calibration_Event"
};
/* USER CODE BEGIN PV */
//Serial
uint8_t strBuffer[512];
uint16_t strSize = 0;
uint16_t USART2Length = 0;

//IMU
float MPU9250_selfTest[6];
MPU9250_t mpu9250;
float accX, accY, accZ;
float magX, magY, magZ;
float gyrX, gyrY, gyrZ;
float IMURateFreq = 0;
float quaternion[4];
float IMU_delta;
bool updateIMU = false;
bool updateRemote = true;
FusionAhrs fusionahrs;
uint32_t IMU_cntNow = 0;
uint32_t IMU_cntPrev = 0;
FusionVector3 accel;
FusionVector3 gyro;
FusionVector3 magnet;
static FusionEulerAngles eulerPV;
static FusionEulerAngles eulerCalibration;
float tmpRoll = 0.0;
FusionBias fusionBias;
FusionEulerAngles eulerPVArray[12];
uint8_t eulerPVCount = 0;
uint32_t eulerPVSampling = 5;
float pid_ts = 0.005;

HAL_StatusTypeDef halStatusTypeDef;

//Position
FusionVector3 linearAccel;
FusionVector3 linearVelocity;
FusionVector3 linearPosition;

//BMP280
struct bmp2_status bmp_status;
struct bmp2_data bmp_comp_data;
struct bmp2_dev bmp_dev;
struct bmp2_config bmp_conf, bmp_test;
int8_t bmp_rslt = BMP2_E_NULL_PTR;
float bmp_delta;
uint32_t bmp_cntNow = 0;
uint32_t bmp_cntPrev = 0;
float bmpRateFreq = 0;
double altitude = 0.0;
double estimated_altitude = 0.0;

//Remote
RC_t PPMRC;
volatile uint32_t PPMOutput[9];
uint32_t Fly_Mode_Flag = FLY_MODE_OFF;
uint32_t Landing_Mode_Flag = LANDING_MODE_MANUAL;

//Position Controller
volatile Velocity_t inputVel;

//MODE
FLY_MODE_t fly_mode = FLY_MODE_OFF;
LANDING_MODE_t flight_mode = LANDING_MODE_MANUAL;

//Control
PIDType_t PIDPitch, PIDRoll, PIDYaw;

extern float G;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
void Start_IMU_Task(void *argument);
void Start_Remote_Task(void *argument);
void Start_PrintSerial_Task(void *argument);
void Start_BMP_Task(void *argument);
void Start_Task_Controller(void *argument);
void Start_SerialReceive_Task(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t DWT_Init(void) {
	/* Disable TRC */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
	/* Enable TRC */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
	/* Disable clock cycle counter */
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	/* Enable  clock cycle counter */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
	/* Reset the clock cycle counter value */
	DWT->CYCCNT = 0;
	/* 3 NO OPERATION instructions */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	/* Check if clock cycle counter has started */
	if (DWT->CYCCNT) {
		return 0; /*clock cycle counter started*/
	} else {
		return 1; /*clock cycle counter not started*/
	}
}

__STATIC_INLINE uint32_t DWT_GetuS() {
	return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static int8_t get_data(uint32_t period, struct bmp2_config *conf,
		struct bmp2_dev *dev) {
	int8_t rslt = BMP2_E_NULL_PTR;
	int8_t idx = 1;
	struct bmp2_status status;
	struct bmp2_data comp_data;

	printf("Measurement delay : %lu us\r\n", (long unsigned int) period);

	while (idx <= 50) {
		rslt = bmp2_get_status(&status, dev);
		bmp2_error_codes_print_result("bmp2_get_status", rslt);

		if (status.measuring == BMP2_MEAS_DONE) {
			/* Delay between measurements */
			dev->delay_us(period, dev->intf_ptr);

			/* Read compensated data */
			rslt = bmp2_get_sensor_data(&comp_data, dev);
			bmp2_error_codes_print_result("bmp2_get_sensor_data", rslt);

#ifdef BMP2_64BIT_COMPENSATION
            comp_data.pressure = comp_data.pressure / 256;
            #endif

#ifdef BMP2_DOUBLE_COMPENSATION
			printf("Data[%d]:    Temperature: %f deg C	Pressure: %f Pa\r\n",
					idx, comp_data.temperature, comp_data.pressure);
#else
            printf("Data[%d]:    Temperature: %ld deg C	Pressure: %lu Pa\n", idx, (long int)comp_data.temperature,
                   (long unsigned int)comp_data.pressure);
            #endif

			idx++;
		}
	}

	return rslt;
}

void bmp280_setup(struct bmp2_config *conf, struct bmp2_dev *dev) {

}

void accelerometer_calibration_orientation(MPU9250_t *mpu9250,
		Orientation orientation) {
	switch (orientation) {
	case Front:
		while (mpu9250->_ax <= 1.0f * G) {
			MPU9250_readSensor(mpu9250);
			HAL_Delay(5);
		}
		HAL_Delay(1500);
		printf("Mengambil Data xmax\r\n");
		break;
	case Back:
		while (mpu9250->_ax >= -1.0f * G) {
			MPU9250_readSensor(mpu9250);
			HAL_Delay(5);
		}
		HAL_Delay(1500);
		printf("Mengambil Data xmin\r\n");
		break;
	case Left:
		while (mpu9250->_ay >= -1.0f * G) {
			MPU9250_readSensor(mpu9250);
			HAL_Delay(5);
		}
		HAL_Delay(1500);
		printf("Mengambil Data ymin\r\n");
		break;
	case Right:
		while (mpu9250->_ay <= 1.0f * G) {
			MPU9250_readSensor(mpu9250);
			HAL_Delay(5);
		}
		HAL_Delay(1500);
		printf("Mengambil Data ymax\r\n");
		break;
	case Top:
		while (mpu9250->_az >= -1.0f * G) {
			MPU9250_readSensor(mpu9250);
			HAL_Delay(5);
		}
		HAL_Delay(1500);
		printf("Mengambil Data zmin\r\n");
		break;
	case Bottom:
		while (!(mpu9250->_az >= 0.82) && mpu9250->_az <= 1.0f * G) {
			MPU9250_readSensor(mpu9250);
			HAL_Delay(5);
		}
		HAL_Delay(1500);
		printf("Mengambil Data zmax\r\n");
		break;
	default:
		break;
	}

	switch (orientation) {
	case Front:
	case Back:
		mpu9250->_axbD = 0;
		for (int i = 0; i < mpu9250->_numSamples; i++) {
			MPU9250_readSensor(mpu9250);
			mpu9250->_axbD += mpu9250->_ax;
			HAL_Delay(20);
		}
		break;

	case Left:
	case Right:
		mpu9250->_aybD = 0;
		for (int i = 0; i < mpu9250->_numSamples; i++) {
			MPU9250_readSensor(mpu9250);
			mpu9250->_aybD += mpu9250->_ay;
			HAL_Delay(20);
		}
		break;

	case Top:
	case Bottom:
		mpu9250->_azbD = 0;
		for (int i = 0; i < mpu9250->_numSamples; i++) {
			MPU9250_readSensor(mpu9250);
			mpu9250->_azbD += mpu9250->_az;
			HAL_Delay(20);
		}
		break;
	default:
		break;
	}
}

void accelerometer_calibration(MPU9250_t *mpu9250) {

	if (MPU9250_setAccelRange(mpu9250, ACCEL_RANGE_2G) < 0) {
		printf("Error\r\n");
	}
	if (MPU9250_setDlpfBandwidth(mpu9250, DLPF_BANDWIDTH_20HZ) < 0) {
		printf("Error\r\n");
	}
	if (MPU9250_setSrd(mpu9250, 19) < 0) {
		printf("Error\r\n");
	}

	for (int i = 0; i < 100; i++) {
		MPU9250_readSensor(mpu9250);
		HAL_Delay(5);
	}

	printf("Posisikan menghadap ke atas!\r\n");
	accelerometer_calibration_orientation(mpu9250, Top);

	mpu9250->_azmin = mpu9250->_azbD / mpu9250->_numSamples;
	printf("zmin: %f\r\n", mpu9250->_azmin);

	printf("Posisikan menghadap ke depan!\r\n");
	accelerometer_calibration_orientation(mpu9250, Front);

	mpu9250->_axmax = mpu9250->_axbD / mpu9250->_numSamples;
	printf("xmax: %f\r\n", mpu9250->_axmax);

	printf("Posisikan menghadap ke kiri!\r\n");
	accelerometer_calibration_orientation(mpu9250, Left);

	mpu9250->_aymin = mpu9250->_aybD / mpu9250->_numSamples;
	printf("ymin: %f\r\n", mpu9250->_aymin);

	printf("Posisikan menghadap ke belakang!\r\n");
	accelerometer_calibration_orientation(mpu9250, Back);

	mpu9250->_axmin = mpu9250->_axbD / mpu9250->_numSamples;
	printf("xmin: %f\r\n", mpu9250->_axmin);

	printf("Posisikan menghadap ke kanan!\r\n");
	accelerometer_calibration_orientation(mpu9250, Right);

	mpu9250->_aymax = mpu9250->_aybD / mpu9250->_numSamples;
	printf("ymax: %f\r\n", mpu9250->_aymax);

	printf("Posisikan menghadap ke bawah!\r\n");
	accelerometer_calibration_orientation(mpu9250, Bottom);

	mpu9250->_azmax = mpu9250->_azbD / mpu9250->_numSamples;
	printf("zmax: %f\r\n", mpu9250->_azmax);

	// set the range, bandwidth, and srd back to what they were
	if (MPU9250_setAccelRange(mpu9250, mpu9250->_accelRange) < 0) {
		printf("Error!\r\n");
	}
	if (MPU9250_setDlpfBandwidth(mpu9250, mpu9250->_bandwidth) < 0) {
		printf("Error!\r\n");
	}
	if (MPU9250_setSrd(mpu9250, mpu9250->_srd) < 0) {
		printf("Error!\r\n");
	}

	mpu9250->_axb = (mpu9250->_axmin + mpu9250->_axmax) / 2.0f;
	mpu9250->_axs = G / ((fabs(mpu9250->_axmin) + fabs(mpu9250->_axmax)) / 2.0f);

	mpu9250->_ayb = (mpu9250->_aymin + mpu9250->_aymax) / 2.0f;
	mpu9250->_ays = G / ((fabs(mpu9250->_aymin) + fabs(mpu9250->_aymax)) / 2.0f);

	mpu9250->_azb = (mpu9250->_azmin + mpu9250->_azmax) / 2.0f;
	mpu9250->_azs = G / ((fabs(mpu9250->_azmin) + fabs(mpu9250->_azmax)) / 2.0f);

	printf("axb: %f\taxs: %f\r\n", mpu9250->_axb, mpu9250->_axs);
	printf("ayb: %f\tays: %f\r\n", mpu9250->_ayb, mpu9250->_ays);
	printf("azb: %f\tazs: %f\r\n", mpu9250->_azb, mpu9250->_azs);

	HAL_Delay(3000);
}

static void PIDReset(PIDType_t *pidtype) {
	pidtype->sumIntegral = 0;
	pidtype->output = 0;

	TIM10->CCR1 = 1000;
	TIM2->CCR2 = 1000;
	TIM2->CCR3 = 1000;
	TIM2->CCR4 = 1000;
}

static void PIDInit(PIDType_t *pidtype, double kp, double ki, double kd,
		double timesampling) {
	PIDReset(pidtype);

	pidtype->kp = kp;
	pidtype->kd = kd;
	pidtype->ki = ki;

	pidtype->timesampling = timesampling;
}

static void PIDControl(PIDType_t *pidtype, float dataSensor, float setPoint) {
	pidtype->setPoint = constrain(setPoint, -30.0, 30.0);
	pidtype->error = pidtype->setPoint - dataSensor;

	if (pidtype->error >= 180.0)
		pidtype->error -= 360.0;
	else if (pidtype->error < -180.0)
		pidtype->error += 360.0;

	pidtype->sumIntegral += pidtype->error * pidtype->timesampling;
	pidtype->sumIntegral = constrain(pidtype->sumIntegral, -500.0, 500.0);

	pidtype->derivative = (pidtype->error - pidtype->preverror)
			/ pidtype->timesampling;
	pidtype->preverror = pidtype->error;

	pidtype->output = (pidtype->kp * pidtype->error)
			+ (pidtype->kd * pidtype->derivative)
			+ (pidtype->ki * pidtype->sumIntegral);
}

void OrientationUpdate() {
	osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);

	accX = MPU9250_getAccelX_g(&mpu9250);
	accY = MPU9250_getAccelY_g(&mpu9250);
	accZ = MPU9250_getAccelZ_g(&mpu9250);

	gyrX = MPU9250_getGyroX_rads(&mpu9250);
	gyrY = MPU9250_getGyroY_rads(&mpu9250);
	gyrZ = MPU9250_getGyroZ_rads(&mpu9250);

	magX = MPU9250_getMagX_uT(&mpu9250);
	magY = MPU9250_getMagY_uT(&mpu9250);
	magZ = MPU9250_getMagZ_uT(&mpu9250);

	accel.axis.x = accX;
	accel.axis.y = accY;
	accel.axis.z = accZ;

	gyro.axis.x = gyrX * RAD_TO_DEG;
	gyro.axis.y = gyrY * RAD_TO_DEG;
	gyro.axis.z = gyrZ * RAD_TO_DEG;

	magnet.axis.x = magX;
	magnet.axis.y = magY;
	magnet.axis.z = magZ;

	FusionBiasUpdate(&fusionBias, gyro);

	FusionAhrsUpdate(&fusionahrs, gyro, accel, magnet, IMU_delta);
	eulerPV = FusionQuaternionToEulerAngles(
			FusionAhrsGetQuaternion(&fusionahrs));

	eulerPV.angle.pitch = (-1.0 * eulerPV.angle.pitch);
	//eulerPV.angle.roll = -1.0 * eulerPV.angle.roll;

	if (eulerPV.angle.roll < 180.0 && eulerPV.angle.roll >= 0) {
		eulerPV.angle.roll = map(eulerPV.angle.roll, 0.0, 180.0, -180.0, 0.0);
	} else if (eulerPV.angle.roll > -180.0 && eulerPV.angle.roll <= 0.0) {
		eulerPV.angle.roll = map(eulerPV.angle.roll, -180.0, 0.0, 0.0, 180.0);
	}

	eulerPV.angle.roll -= eulerCalibration.angle.roll;
	eulerPV.angle.pitch -= eulerCalibration.angle.pitch;
	eulerPV.angle.yaw -= eulerCalibration.angle.yaw;

	//osThreadFlagsClear(0x1);
}

void OrientationCalibrationUpdate() {
	eulerCalibration.angle.roll = 0.0f;
	eulerCalibration.angle.pitch = 0.0f;
	eulerCalibration.angle.yaw = 0.0f;

	FusionEulerAngles eulerTotal;

	for (int i = 0; i < 1000; i++) {
		//osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);

		OrientationUpdate();

		eulerTotal.angle.roll += eulerPV.angle.roll;
		eulerTotal.angle.pitch += eulerPV.angle.pitch;
		eulerTotal.angle.yaw += eulerPV.angle.yaw;

		//osThreadFlagsClear(0x1);
	}
	eulerCalibration.angle.roll = eulerTotal.angle.roll / 1000.0;
	eulerCalibration.angle.pitch = eulerTotal.angle.pitch / 1000.0;
	eulerCalibration.angle.yaw = eulerTotal.angle.yaw / 1000.0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM12_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

	HAL_Delay(100);
	RetargetInit(&SERIAL_DEBUG);

	printf("Coba printf\r\n");

	printf("Mulai!\r\n");

	if (DWT_Init()) {
		printf("DWT gagal inisialisasi!\r\n");
		while (1)
			;
	}

	/*HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	 TIM11->CCR1 = 500;
	 HAL_Delay(3000);
	 TIM11->CCR1 = 0;*/

	//MPU9250_begin();
	/*uint8_t imu_whoami = MPU9250_WhoAmI();

	 strSize = sprintf((char*)strBuffer, "Hasil Who Am I MPU9250: %d\r\n", imu_whoami);
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);

	 if(imu_whoami != 0x71 && imu_whoami != 0x73){
	 strSize = sprintf((char*)strBuffer, "Gagal inisialiasasi IMU!\r\n");
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);
	 while(1);
	 } else {
	 MPU9250_SelfTest(MPU9250_selfTest);

	 strSize = sprintf((char*)strBuffer, "x-Axis self test: acceleration trim within: %f %% of factory value\r\n", MPU9250_selfTest[0]);
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);

	 strSize = sprintf((char*)strBuffer, "y-Axis self test: acceleration trim within: %f %% of factory value\r\n", MPU9250_selfTest[1]);
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);

	 strSize = sprintf((char*)strBuffer, "z-Axis self test: acceleration trim within: %f %% of factory value\r\n", MPU9250_selfTest[2]);
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);

	 strSize = sprintf((char*)strBuffer, "x-Axis self test: gyration trim within: %f %% of factory value\r\n", MPU9250_selfTest[3]);
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);

	 strSize = sprintf((char*)strBuffer, "y-Axis self test: gyration trim within: %f %% of factory value\r\n", MPU9250_selfTest[4]);
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);

	 strSize = sprintf((char*)strBuffer, "z-Axis self test: gyration trim within: %f %% of factory value\r\n", MPU9250_selfTest[5]);
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);
	 }

	 uint8_t magnet_whoami = MPU9250_AK8963WhoAmI();

	 strSize = sprintf((char*)strBuffer, "Hasil Who Am I AK8963: %d\r\n", magnet_whoami);
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);

	 if(magnet_whoami != 0x48){
	 strSize = sprintf((char*)strBuffer, "Gagal inisialiasasi Magnetometer!\r\n");
	 HAL_UART_Transmit(&SERIAL_DEBUG, strBuffer, strSize, HAL_MAX_DELAY);
	 while(1);
	 }*/
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of SPI1_Mutex */
  SPI1_MutexHandle = osMutexNew(&SPI1_Mutex_attributes);

  /* creation of SerialDebug_Mutex */
  SerialDebug_MutexHandle = osMutexNew(&SerialDebug_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(Start_IMU_Task, NULL, &IMU_Task_attributes);

  /* creation of Remote_Task */
  Remote_TaskHandle = osThreadNew(Start_Remote_Task, NULL, &Remote_Task_attributes);

  /* creation of PrintSerial_Tas */
  PrintSerial_TasHandle = osThreadNew(Start_PrintSerial_Task, NULL, &PrintSerial_Tas_attributes);

  /* creation of BMP_Task */
  BMP_TaskHandle = osThreadNew(Start_BMP_Task, NULL, &BMP_Task_attributes);

  /* creation of Task_Controller */
  Task_ControllerHandle = osThreadNew(Start_Task_Controller, NULL, &Task_Controller_attributes);

  /* creation of SerialRecv_Task */
  SerialRecv_TaskHandle = osThreadNew(Start_SerialReceive_Task, NULL, &SerialRecv_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of Fly_Event */
  Fly_EventHandle = osEventFlagsNew(&Fly_Event_attributes);

  /* creation of Landing_Event */
  Landing_EventHandle = osEventFlagsNew(&Landing_Event_attributes);

  /* creation of Calibration_Event */
  Calibration_EventHandle = osEventFlagsNew(&Calibration_Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM8_BRK_TIM12_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 8;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 164 - 1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 20000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 168 - 1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000 - 1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 84 - 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, US_TRIG_Pin|GPIO_PIN_3|GPIO_PIN_4|LORA_RST_Pin
                          |RTC_NC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BMP_CS_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LORA_DIO1_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LORA_DIO2_Pin|LORA_DIO3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : US_TRIG_Pin PE3 PE4 LORA_RST_Pin
                           LED1_Pin LED2_Pin LED3_Pin RTC_NC_Pin */
  GPIO_InitStruct.Pin = US_TRIG_Pin|GPIO_PIN_3|GPIO_PIN_4|LORA_RST_Pin
                          |LED1_Pin|LED2_Pin|LED3_Pin|RTC_NC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP_CS_Pin */
  GPIO_InitStruct.Pin = BMP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BMP_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO1_Pin PB15 */
  GPIO_InitStruct.Pin = LORA_DIO1_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO2_Pin LORA_DIO3_Pin */
  GPIO_InitStruct.Pin = LORA_DIO2_Pin|LORA_DIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_INT_Pin */
  GPIO_InitStruct.Pin = GPS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART2) {
		osThreadFlagsSet(SerialRecv_TaskHandle, 0x1);
		USART2Length = Size;
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	static uint8_t PPMCount;
	static uint32_t PWMCH2;

	if (htim->Instance == TIM12) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			static uint32_t oldVal;
			static uint32_t nowVal;
			static uint32_t deltaFalling;

			nowVal = TIM12->CCR1;
			if (nowVal >= oldVal)
				deltaFalling = nowVal - oldVal;
			oldVal = nowVal;

			if (nowVal >= PWMCH2 && (nowVal - PWMCH2 >= 4000)) {
				__HAL_TIM_SET_COUNTER(&htim12, 0);
				oldVal = 0;
				PPMCount = 0;
				osThreadFlagsSet(Remote_TaskHandle, 0x1);
			} else {
				PPMOutput[PPMCount] = deltaFalling;
				PPMCount++;
			}

		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			PWMCH2 = TIM12->CCR2;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_IMU_Task */
/**
 * @brief  Function implementing the IMU_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_IMU_Task */
void Start_IMU_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
	for (int i = 0; i < 3; i++) {
		linearAccel.array[i] = 0;
		linearVelocity.array[i] = 0;
		linearPosition.array[i] = 0;
	}
	osMutexAcquire(SPI1_MutexHandle, osWaitForever);
	MPU9250_init(&mpu9250, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin);
	int statusCode = MPU9250_begin(&mpu9250);
	if (statusCode < 0) {
		printf("IMU gagal inisialisasi\r\n");
		while (1)
			;
	}

	statusCode = MPU9250_enableDataReadyInterrupt(&mpu9250);
	if (statusCode < 0) {
		printf("Data ready Interrupt gagal! code: %d\r\n", statusCode);
		while (1)
			;
	}

	printf("Memulai kalibrasi accelerometer...\r\n");

	MPU9250_calibrateAccel(&mpu9250);
	//accelerometer_calibration(&mpu9250);

	/*axb: 0.151378	axs: 1.001281
	 ayb: 0.094687	ays: 1.002767
	 azb: -1.724494	azs: 0.992538*/

	/*if (MPU9250_setAccelRange(&mpu9250, ACCEL_RANGE_2G) < 0) {
		printf("Error\r\n");
	}
	if (MPU9250_setDlpfBandwidth(&mpu9250, DLPF_BANDWIDTH_184HZ) < 0) {
		printf("Error\r\n");
	}
	if (MPU9250_setSrd(&mpu9250, 0) < 0) {
		printf("Error\r\n");
	}*/

	for (int i = 0; i < 100; i++) {
		MPU9250_readSensor(&mpu9250);
		HAL_Delay(5);
	}

	//MPU9250_setAccelCalX(&mpu9250, 0.151378, 1.001281);
	//MPU9250_setAccelCalY(&mpu9250, 0.094687, 1.002767);
	//MPU9250_setAccelCalZ(&mpu9250, -1.724494, 0.992538);

	printf("Kalibrasi accelerometer sukses!\r\n");

	//printf("Memulai kalibrasi magnetometer...\r\n");
	//MPU9250_calibrateMag(&mpu9250);

	//printf("Kalibrasi magnetometer sukses!\r\n");

	printf("MPU9250 Self Test\r\n");

	//MPU9250_SelfTest(&mpu9250, MPU9250_selfTest);
	osMutexRelease(SPI1_MutexHandle);

	//mpu9250._azb = -1.4;
	//mpu9250._azs = 0.9906;

	FusionBiasInitialise(&fusionBias, 0.005f, 0.001f);
	FusionAhrsInitialise(&fusionahrs, 2.5f);

	HAL_TIM_Base_Start(&htim5);
	osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
	//osEventFlagsWait(IMUEventHandle, 0x1, osFlagsWaitAny, osWaitForever);
	//Calibration Orientation
	printf("Calibration IMU Orientation\r\n");

	for (int i = 0; i < 2000; i++)
		OrientationUpdate();

	//OrientationCalibrationUpdate();

	printf("Calibration IMU Orientation Done!\r\n");

	osThreadFlagsSet(PrintSerial_TasHandle, 0x1);
	//osThreadFlagsSet(Task_ControllerHandle, 0x1);

	//osEventFlagsSet(ControllerEventHandle, 0x1);
	//osEventFlagsSet(SerialEventHandle, 0x1);

	uint8_t eulerCount = 0;

	/* Infinite loop */
	for (;;) {
		//osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		OrientationUpdate();
		linearAccel = FusionAhrsGetLinearAcceleration(&fusionahrs);

		for (int i = 0; i < 3; i++) {
			linearVelocity.array[i] += linearAccel.array[i] * IMU_delta;
			linearPosition.array[i] += linearVelocity.array[i] * IMU_delta;
		}

		eulerCount++;
		if (eulerCount < 5) {
			eulerPVArray[eulerCount].angle.pitch = eulerPV.angle.pitch;
			eulerPVArray[eulerCount].angle.roll = eulerPV.angle.roll;
			eulerPVArray[eulerCount].angle.yaw = eulerPV.angle.yaw;
		} else {
			eulerCount = 0;
		}

		if(osEventFlagsGet(Calibration_EventHandle) == 0x1){
			OrientationCalibrationUpdate();
			osEventFlagsClear(Calibration_EventHandle, 0x1);
			osEventFlagsSet(Calibration_EventHandle, 0x2);
		}

		//osThreadFlagsClear(0x1);
		//osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Remote_Task */
/**
 * @brief Function implementing the Remote_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Remote_Task */
void Start_Remote_Task(void *argument)
{
  /* USER CODE BEGIN Start_Remote_Task */
	HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_2);
	/* Infinite loop */
	for (;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		PPMRC.Roll = constrain(PPMOutput[0], 1000, 2000);
		PPMRC.Pitch = constrain(PPMOutput[1], 1000, 2000);
		PPMRC.Throttle = constrain(PPMOutput[2], 1000, 2000);
		PPMRC.Yaw = constrain(PPMOutput[3], 1000, 2000);
		PPMRC.SWC = constrain(PPMOutput[4], 1000, 2000);
		PPMRC.SWB = constrain(PPMOutput[5], 1000, 2000);

		if (Fly_Mode_Flag != FLY_MODE_OFF && PPMRC.SWC >= 1000
				&& PPMRC.SWC <= 1100) {
			Fly_Mode_Flag = FLY_MODE_OFF;
			printf("FLY MODE OFF\r\n");
		} else if (Fly_Mode_Flag != FLY_MODE_ON && PPMRC.SWC >= 1450
				&& PPMRC.SWC <= 1550) {
			Fly_Mode_Flag = FLY_MODE_ON;
			osEventFlagsSet(Calibration_EventHandle, 0x4);

			printf("FLY MODE ON\r\n");
		} else if (Fly_Mode_Flag != FLY_MODE_HOLD && PPMRC.SWC >= 1900
				&& PPMRC.SWC <= 2000) {
			Fly_Mode_Flag = FLY_MODE_HOLD;
			printf("FLY MODE HOLD\r\n");
		}

		if (Landing_Mode_Flag != LANDING_MODE_MANUAL && PPMRC.SWB >= 1000
				&& PPMRC.SWB <= 1100) {
			Landing_Mode_Flag = LANDING_MODE_MANUAL;
			printf("LANDING MODE MANUAL\r\n");
		} else if (Landing_Mode_Flag != LANDING_MODE_AUTO && PPMRC.SWB >= 1900
				&& PPMRC.SWB <= 2000) {
			Landing_Mode_Flag = LANDING_MODE_AUTO;
			printf("LANDING MODE AUTO\r\n");
		}
		osThreadFlagsClear(0x1);

		//eulerInput.angle.pitch = (float)map((float)PPMRC.Pitch, 1000.0, 2000.0, -30.0, 30.0);
		//eulerInput.angle.roll = (float)map((float)PPMRC.Roll, 1000.0, 2000.0, -30.0, 30.0);
		//eulerInput.angle.yaw = (float)map((float)PPMRC.Yaw, 1000.0, 2000.0, -30.0, 30.0);

		/*inputVel.xV = (float) map((float )PPMRC.Roll, 1000.0, 2000.0, -10.0,
		 10.0);
		 inputVel.yV = (float) map((float )PPMRC.Pitch, 1000.0, 2000.0, -10.0,
		 10.0);*/

		//HAL_UART_Transmit_IT(&SERIAL_DEBUG, (uint8_t*)".\r\n", 4);
		//osEventFlagsWait(RemoteEventHandle, flag, osFlagsWaitAny, osWaitForever);
		//flag = osEventFlagsGet(RemoteEventHandle);
		//PPMOutput[flag] = prevCount;
		//__HAL_TIM_SET_COUNTER(&htim12, 0);
		//osEventFlagsClear(RemoteEventHandle, flag);
		//osDelay(10);
	}
  /* USER CODE END Start_Remote_Task */
}

/* USER CODE BEGIN Header_Start_PrintSerial_Task */
/**
 * @brief Function implementing the PrintSerial_Tas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_PrintSerial_Task */
void Start_PrintSerial_Task(void *argument)
{
  /* USER CODE BEGIN Start_PrintSerial_Task */
	//osEventFlagsWait(SerialEventHandle, 0x1, osFlagsWaitAny, osWaitForever);
	osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
	//printf("No,Yaw,Pitch,Roll,ax,ay,az,vx,vy,vz,px,py,pz\r\n");
	//static uint32_t i;
	/* Infinite loop */
	for (;;) {
		//strSize = sprintf((char*)strBuffer, "CH1: %lu\tCH2: %lu\tCH3: %lu\tCH4: %lu\tCH5: %lu\tCH6: %lu\r\n", PPMOutput[0], PPMOutput[1], PPMOutput[2], PPMOutput[3], PPMOutput[4], PPMOutput[5]);
		//HAL_UART_Transmit_IT(&SERIAL_DEBUG, strBuffer, strSize);
		//printf("%lu->\t", DWT_GetuS());
		//printf("CH1: %lu\tCH2: %lu\tCH3: %lu\tCH4: %lu\tCH5: %lu\tCH6: %lu\r\n", PPMRC.Roll, PPMRC.Pitch, PPMRC.Throttle, PPMRC.Yaw, PPMRC.SWC, PPMRC.SWB);
		//strSize = sprintf((char*)strBuffer, "Velocity: %.2f %.2f\r\n", inputVel.xV, inputVel.yV);
		//strSize = sprintf((char*)strBuffer, "Velocity\r\n");
		//HAL_UART_Transmit_IT(&SERIAL_DEBUG, strBuffer, strSize);
		//printf("%lu,%f,%f,%f,", i, eulerPV.angle.yaw, eulerPV.angle.pitch, eulerPV.angle.roll);
		//printf("%f,%f,%f,", linearAccel.axis.x, linearAccel.axis.y, linearAccel.axis.z);
		//printf("%f,%f,%f,", linearVelocity.axis.x, linearVelocity.axis.y, linearVelocity.axis.z);
		//printf("%f,%f,%f\r\n", linearPosition.axis.x, linearPosition.axis.y, linearPosition.axis.z);
		//printf("%f\t%f\t%f\r\n", eulerPV.angle.yaw, eulerPV.angle.pitch, eulerPV.angle.roll);
		//printf("%f\r\n",IMU_delta);
		//printf("%f\t%f\t%f\r\n", gyrX, gyrY, gyrZ);

		//printf("%f\t%f\t%f\r\n", linearVelocity.axis.x, linearVelocity.axis.y, linearVelocity.axis.z);
		//printf("%f\t\%f\t%f\r\n", linearAccel.axis.x, linearAccel.axis.y, linearAccel.axis.z);
		//printf("%f\t\%f\t%f\r\n", mpu9250._ax, mpu9250._ay, mpu9250._az);
		//i++;
		//printf("M1: %lu\tM2: %lu\tM3: %lu\tM4: %lu\r\n", TIM10->CCR1, TIM2->CCR2, TIM2->CCR3, TIM2->CCR4);
		/*for(int i = 0; i < 5; i++){
		 eulerPV2.angle.yaw += eulerPVArray[i].angle.yaw;
		 eulerPV2.angle.pitch += eulerPVArray[i].angle.pitch;
		 eulerPV2.angle.roll += eulerPVArray[i].angle.roll;
		 }
		 eulerPV2.angle.yaw /= 5.0;
		 eulerPV2.angle.pitch /= 5.0;
		 eulerPV2.angle.roll /= 5.0;

		 printf("%f\t%f\t%f\r\n", eulerPV2.angle.yaw, eulerPV2.angle.pitch, eulerPV2.angle.roll);

		 eulerPV2.angle.yaw = 0.0;
		 eulerPV2.angle.pitch = 0.0;
		 eulerPV2.angle.roll = 0.0;*/

		if(Fly_Mode_Flag != FLY_MODE_OFF){
			printf("%f\t%f\t%f\r\n", eulerPV.angle.yaw, eulerPV.angle.pitch, eulerPV.angle.roll);
		}

		osDelay(10);
	}
  /* USER CODE END Start_PrintSerial_Task */
}

/* USER CODE BEGIN Header_Start_BMP_Task */
/**
 * @brief Function implementing the BMP_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_BMP_Task */
void Start_BMP_Task(void *argument)
{
  /* USER CODE BEGIN Start_BMP_Task */
	Kalman_t altitude_kalman;
	uint32_t meas_time;

	/* Interface selection is to be updated as parameter
	 * For I2C :  BMP2_I2C_INTF
	 * For SPI :  BMP2_SPI_INTF
	 */

	osMutexAcquire(SPI1_MutexHandle, osWaitForever);
	bmp_rslt = bmp2_interface_selection(&bmp_dev, BMP2_SPI_INTF);
	bmp2_error_codes_print_result("bmp2_interface_selection", bmp_rslt);

	bmp_rslt = bmp2_init(&bmp_dev);
	bmp2_error_codes_print_result("bmp2_init", bmp_rslt);

	/* Always read the current settings before writing, especially when all the configuration is not modified */
	bmp_rslt = bmp2_get_config(&bmp_conf, &bmp_dev);
	bmp2_error_codes_print_result("bmp2_get_config", bmp_rslt);

	printf("os_temp: %d\r\n", bmp_conf.os_temp);
	printf("os_pres: %d\r\n", bmp_conf.os_pres);
	printf("filter: %d\r\n", bmp_conf.filter);
	printf("odr: %d\r\n", bmp_conf.odr);
	printf("powermode: %d\r\n", bmp_dev.power_mode);

	/* Configuring the over-sampling mode, filter coefficient and output data rate */
	/* Overwrite the desired settings */
	bmp_conf.filter = BMP2_FILTER_OFF;

	/* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
	bmp_conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;

	/* Setting the output data rate */
	bmp_conf.odr = BMP2_ODR_62_5_MS;

	bmp_rslt = bmp2_set_config(&bmp_conf, &bmp_dev);
	bmp2_error_codes_print_result("bmp2_set_config", bmp_rslt);

	/* Set normal power mode */
	bmp_rslt = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &bmp_conf, &bmp_dev);
	bmp2_error_codes_print_result("bmp2_set_power_mode", bmp_rslt);

	/* Calculate measurement time in microseconds */
	bmp_rslt = bmp2_compute_meas_time(&meas_time, &bmp_conf, &bmp_dev);
	bmp2_error_codes_print_result("bmp2_compute_meas_time", bmp_rslt);

	/* Read pressure and temperature data */
	//bmp_rslt = get_data(meas_time, &bmp_conf, &bmp_dev);
	//bmp2_error_codes_print_result("get_data", bmp_rslt);
	bmp_rslt = bmp2_get_config(&bmp_test, &bmp_dev);
	bmp2_error_codes_print_result("bmp2_get_config", bmp_rslt);

	printf("os_temp: %d\r\n", bmp_test.os_temp);
	printf("os_pres: %d\r\n", bmp_test.os_pres);
	printf("filter: %d\r\n", bmp_test.filter);
	printf("odr: %d\r\n", bmp_test.odr);
	printf("powermode: %d\r\n", bmp_dev.power_mode);

	HAL_SPI_DeInit(&hspi1);
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
		Error_Handler();

	bmp_rslt = bmp2_get_status(&bmp_status, &bmp_dev);
	bmp2_error_codes_print_result("bmp2_get_status", bmp_rslt);
	uint32_t nData = 0;
	static double tmpCalPres;

	while (nData < 100) {
		bmp_rslt = bmp2_get_status(&bmp_status, &bmp_dev);
		bmp2_error_codes_print_result("bmp2_get_status", bmp_rslt);

		if (bmp_status.measuring == BMP2_MEAS_DONE) {
			bmp_rslt = bmp2_get_sensor_data(&bmp_comp_data, &bmp_dev);
			bmp2_error_codes_print_result("bmp2_get_sensor_data", bmp_rslt);
			bmp_comp_data.pressure /= 100.0;
			tmpCalPres += bmp_comp_data.pressure;
			nData++;
		}
	}
	osMutexRelease(SPI1_MutexHandle);

	HAL_TIM_Base_Start_IT(&htim6);
	double calPres = tmpCalPres / 100.0;
	kalman_init(&altitude_kalman, 0.12, 0.12,
			44330.0 * (1.0 - pow(bmp_comp_data.pressure / calPres, 0.1903)));
	//osEventFlagsSet(IMUEventHandle, 0x1);
	osThreadFlagsSet(IMU_TaskHandle, 0x1);
	/* Infinite loop */
	for (;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		if (bmp_rslt == BMP2_OK) {
			bmp_comp_data.pressure /= 100.0;
			altitude = 44330.0
					* (1.0 - pow(bmp_comp_data.pressure / calPres, 0.1903));
			estimated_altitude = kalman_updateEstimate(&altitude_kalman,
					altitude);
			//printf("%.4f\t%.4f\r\n", altitude, estimated_altitude);
		}
		osThreadFlagsClear(0x1);
		/*osMutexAcquire(SPI1_MutexHandle, osWaitForever);
		 osMutexRelease(SPI1_MutexHandle);
		 osDelay(10);*/
		//osDelay(osWaitForever);
	}
  /* USER CODE END Start_BMP_Task */
}

/* USER CODE BEGIN Header_Start_Task_Controller */
/**
 * @brief Function implementing the Task_Controller thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Task_Controller */
void Start_Task_Controller(void *argument)
{
  /* USER CODE BEGIN Start_Task_Controller */
	/*
	 CW 2\     /1 CCW
	 	  \   /
	       \ /
	       / \
	      /   \
    CCW 3/     \4 CW
	 QuadCopter
	 Motor 1000 KV
	 Propeller : 9 x 4.5
	 MaX thrust = 49.663985 N
	 Thrust for each motor = 12.41599
	 Formula :
	 F = 1.225 * ((3.14(0.0254 * d)^2)/4) *(RPM*0.0254*pitch*1/60)^2 * (d/(3.29546*pitch))^1.5;
	 F = 0.050252560785 * 522.5796 * 0.47279287884410658042558653798071
	 */

	FusionEulerAngles eulerSP;

	PIDInit(&PIDPitch, 0.027f, 0.0053f, 0.0055f, pid_ts);
	PIDInit(&PIDRoll, 0.027f, 0.0053f, 0.0055f, pid_ts);
	PIDInit(&PIDYaw, 0.017f, 0.0025f, 0.0f, pid_ts);

	float motor1Thrust, motor2Thrust, motor3Thrust, motor4Thrust;
	static float thrust;
	float RPMmotor1, RPMmotor2, RPMmotor3, RPMmotor4;
	float throttle = 1000.0;

	const float angleMotor1 = 45.0f / RAD_TO_DEG;
	const float angleMotor2 = 135.0f / RAD_TO_DEG;
	const float angleMotor3 = 225.0f / RAD_TO_DEG;
	const float angleMotor4 = 315.0f / RAD_TO_DEG;
	const float L = 0.225;

	const double sinAngleMotor1 = sin(angleMotor1);
	const double sinAngleMotor2 = sin(angleMotor2);
	const double sinAngleMotor3 = sin(angleMotor3);
	const double sinAngleMotor4 = sin(angleMotor4);

	const double cosAngleMotor1 = cos(angleMotor1);
	const double cosAngleMotor2 = cos(angleMotor2);
	const double cosAngleMotor3 = cos(angleMotor3);
	const double cosAngleMotor4 = cos(angleMotor4);

	//osEventFlagsWait(ControllerEventHandle, 0x1, osFlagsWaitAny, osWaitForever);
	//osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);

	/* Infinite loop */
	for (;;) {
		if(Fly_Mode_Flag == FLY_MODE_ON){

		} else if(Fly_Mode_Flag == FLY_MODE_HOLD){

		} else if(Fly_Mode_Flag == FLY_MODE_OFF){
			PIDReset(&PIDRoll);
			PIDReset(&PIDPitch);
			PIDReset(&PIDYaw);
			TIM2->CCR2 = 1000;
			TIM2->CCR3 = 1000;
			TIM2->CCR4 = 1000;
			TIM10->CCR1 = 1000;
			osEventFlagsWait(Calibration_EventHandle, 0x4, osFlagsWaitAny, osWaitForever);
			osEventFlagsSet(Calibration_EventHandle, 0x1);
			osEventFlagsWait(Calibration_EventHandle, 0x2, osFlagsWaitAny, osWaitForever);
			continue;
		}

		eulerSP.angle.pitch = map(PPMRC.Pitch, 1000.0, 2000.0, -30.0, 30.0);
		eulerSP.angle.yaw = map(PPMRC.Yaw, 1000.0, 2000.0, -30.0, 30.0);
		eulerSP.angle.roll = map(PPMRC.Roll, 1000.0, 2000.0, -30.0, 30.0);

		static FusionEulerAngles eulerPVAverage;

		for (int i = 0; i < 5; i++) {
			eulerPVAverage.angle.roll += eulerPVArray[i].angle.roll;
			eulerPVAverage.angle.pitch += eulerPVArray[i].angle.pitch;
			eulerPVAverage.angle.yaw += eulerPVArray[i].angle.yaw;
		}

		eulerPVAverage.angle.roll /= 5.0;
		eulerPVAverage.angle.pitch /= 5.0;
		eulerPVAverage.angle.yaw /= 5.0;

		PIDControl(&PIDRoll, eulerPVAverage.angle.roll, eulerSP.angle.roll);
		PIDControl(&PIDPitch, eulerPVAverage.angle.pitch, eulerSP.angle.pitch);
		PIDControl(&PIDYaw, eulerPVAverage.angle.yaw, eulerSP.angle.yaw);

		if (PPMRC.Throttle >= 1100)
			throttle = (float) PPMRC.Throttle;
		else
			throttle = 1000.0;

		thrust = (float) map(throttle, 1000.0, 2000.0, 0.0, 49.663985f);

		motor1Thrust = (thrust / 4.0 - PIDPitch.output * sinAngleMotor1
				+ PIDRoll.output * cosAngleMotor1 - PIDYaw.output);
		motor2Thrust = (thrust / 4.0 - PIDPitch.output * sinAngleMotor2
				+ PIDRoll.output * cosAngleMotor2 + PIDYaw.output);
		motor3Thrust = (thrust / 4.0 - PIDPitch.output * sinAngleMotor3
				+ PIDRoll.output * cosAngleMotor3 - PIDYaw.output);
		motor4Thrust = (thrust / 4.0 - PIDPitch.output * sinAngleMotor4
				+ PIDRoll.output * cosAngleMotor4 + PIDYaw.output);

		RPMmotor1 = sqrt(motor1Thrust / 0.023759052) / 0.001905;
		RPMmotor2 = sqrt(motor2Thrust / 0.023759052) / 0.001905;
		RPMmotor3 = sqrt(motor3Thrust / 0.023759052) / 0.001905;
		RPMmotor4 = sqrt(motor4Thrust / 0.023759052) / 0.001905;

		RPMmotor1 = constrain(RPMmotor1, 0.0, 12000.0);
		RPMmotor2 = constrain(RPMmotor2, 0.0, 12000.0);
		RPMmotor3 = constrain(RPMmotor3, 0.0, 12000.0);
		RPMmotor4 = constrain(RPMmotor4, 0.0, 12000.0);

		TIM2->CCR3 = (uint32_t) map(RPMmotor1, 0, 12000, 1000, 2000);
		TIM2->CCR4 = (uint32_t) map(RPMmotor2, 0, 12000, 1000, 2000);
		TIM10->CCR1 = (uint32_t) map(RPMmotor3, 0, 12000, 1000, 2000);
		TIM2->CCR2 = (uint32_t) map(RPMmotor4, 0, 12000, 1000, 2000);

		eulerPVAverage.angle.pitch = 0.0;
		eulerPVAverage.angle.yaw = 0.0;
		eulerPVAverage.angle.roll = 0.0;

		osDelay(5);
		//osDelay(osWaitForever);
	}
  /* USER CODE END Start_Task_Controller */
}

/* USER CODE BEGIN Header_Start_SerialReceive_Task */
/**
 * @brief Function implementing the SerialRecv_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_SerialReceive_Task */
void Start_SerialReceive_Task(void *argument)
{
  /* USER CODE BEGIN Start_SerialReceive_Task */

	uint8_t MainBuf[MainBuf_SIZE] = { 0 };
	uint8_t RxBuf[RxBuf_SIZE] = { 0 };
	uint8_t *pos_start, *pos_end;
	uint8_t k_temp[15];
	double kp_new = 0.0, ki_new = 0.0, kd_new = 0.0;

	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE) != HAL_OK)
		printf("USART2: DMA ERROR!\r\n");
	else
		printf("USART2 DMA SUCCESS!\r\n");
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	/* Infinite loop */
	for (;;) {
		osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
		memcpy((uint8_t*) MainBuf, RxBuf, USART2Length);
		if (MainBuf[0] == '@' && MainBuf[USART2Length - 1] == '#') {
			pos_start = memchr(MainBuf + 2, 'k', USART2Length);

			while (pos_start != NULL) {
				pos_end = memchr(pos_start + 2, '!', USART2Length);
				memcpy(k_temp, pos_start + 3, (pos_end) - (pos_start + 3));

				switch (MainBuf[pos_start - MainBuf + 1]) {
				case 'p':
					kp_new = strtod((char*)k_temp, NULL);
					break;
				case 'i':
					ki_new = strtod((char*)k_temp, NULL);
					break;
				case 'd':
					kd_new = strtod((char*)k_temp, NULL);
					break;
				}

				pos_start = memchr(pos_end + 1, 'k', USART2Length);
				memset(k_temp, 0, 15);
			}

			if (Fly_Mode_Flag == FLY_MODE_OFF) {
				switch (MainBuf[1]) {
				case 'P':
					PIDInit(&PIDPitch, kp_new, ki_new, kd_new, pid_ts);
					printf("PID Pitch: Kp: %f, Ki: %f, Kd: %f\r\n", kp_new, ki_new, kd_new);
					break;
				case 'R':
					PIDInit(&PIDRoll, kp_new, ki_new, kd_new, pid_ts);
					printf("PID Roll: Kp: %f, Ki: %f, Kd: %f\r\n", kp_new, ki_new, kd_new);
					break;
				case 'Y':
					PIDInit(&PIDYaw, kp_new, ki_new, kd_new, pid_ts);
					printf("PID Yaw: Kp: %f, Ki: %f, Kd: %f\r\n", kp_new, ki_new, kd_new);
					break;
				case 'I':
					printf("PIDPitch:\tKp: %f, Ki: %f, Kd: %f\r\n", PIDPitch.kp, PIDPitch.ki, PIDPitch.kd);
					printf("PIDRoll:\tKp: %f, Ki: %f, Kd: %f\r\n", PIDRoll.kp, PIDRoll.ki, PIDRoll.kd);
					printf("PIDYaw: \tKp: %f, Ki: %f, Kd: %f\r\n", PIDYaw.kp, PIDYaw.ki, PIDYaw.kd);
					printf("Time Sampling: %f\r\n", pid_ts);
					break;
				default:
					continue;
				}
			}
		}

		//printf("%s\r\n", MainBuf);
		memset(MainBuf, 0, MainBuf_SIZE);

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
  /* USER CODE END Start_SerialReceive_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM6) {
		static uint32_t count;
		if (MPU9250_getDrdyStatus(&mpu9250)) {
			MPU9250_readSensor(&mpu9250);
			IMU_cntPrev = IMU_cntNow;
			IMU_cntNow = TIM5->CNT;
			IMU_delta = ((float) IMU_cntNow - (float) IMU_cntPrev)
					/ (1000000.0f);
			IMURateFreq = (1.0f / IMU_delta);
			osThreadFlagsSet(IMU_TaskHandle, 0x1);
		}

		bmp_rslt = bmp2_get_status(&bmp_status, &bmp_dev);
		bmp2_error_codes_print_result("bmp2_get_status", bmp_rslt);

		if (count % 100 == 0) {
			if (bmp_status.measuring == BMP2_MEAS_DONE) {
				bmp_rslt = bmp2_get_sensor_data(&bmp_comp_data, &bmp_dev);
				bmp2_error_codes_print_result("bmp2_get_sensor_data", bmp_rslt);
				bmp_cntPrev = bmp_cntNow;
				bmp_cntNow = TIM5->CNT;
				bmp_delta = ((float) bmp_cntNow - (float) bmp_cntPrev)
						/ (1000000.0f);
				bmpRateFreq = (1.0f / bmp_delta);
				osThreadFlagsSet(BMP_TaskHandle, 0x1);
			}
		}

		count++;
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	PIDReset(&PIDRoll);
	PIDReset(&PIDYaw);
	PIDReset(&PIDPitch);
	while (1) {

	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
