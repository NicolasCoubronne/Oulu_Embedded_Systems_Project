/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tof.c
 * @brief   This file provides code for the configuration
 *          of the TOF sensors.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tof.h"
#include <inc/vl53l0x_api.h>

/* Private define ------------------------------------------------------------*/
#define HandleI2C hi2c3;
#define I2cDevAddr_TOF1 0x52
#define I2cDevAddr_TOF2 0x54

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


VL53L0X_Dev_t  vl53l0x_1; // first TOF	0x52
VL53L0X_DEV    TOF1 = &vl53l0x_1;
/*
VL53L0X_Dev_t  vl53l0x_2; // second TOF 0x54
VL53L0X_DEV    TOF2 = &vl53l0x_2;
*/
extern I2C_HandleTypeDef HandleI2C;
/* TOF1 init function */

VL53L0X_Error TOF1_VL53L0X_Init(uint32_t* refSpadCount,uint8_t* isApertureSpads,uint8_t* VhvSettings,uint8_t* PhaseCal)
{
	TOF1->I2cHandle = &HandleI2C;
	TOF1->I2cDevAddr = I2cDevAddr_TOF1;
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	HAL_GPIO_WritePin(TOF1_XSHUT_GPIO_Port, TOF1_XSHUT_Pin, GPIO_PIN_RESET); // Disable XSHUT
	HAL_Delay(20);
	HAL_GPIO_WritePin(TOF1_XSHUT_GPIO_Port, TOF1_XSHUT_Pin, GPIO_PIN_SET); // Enable XSHUT
	HAL_Delay(20);

	//Device initialization
	VL53L0X_WaitDeviceBooted( TOF1 );
	status = VL53L0X_DataInit( TOF1 );
	if (status!=VL53L0X_ERROR_NONE) return status;
	VL53L0X_StaticInit( TOF1 );
	//SPADs calibration
	VL53L0X_PerformRefSpadManagement(TOF1, refSpadCount, isApertureSpads);
	//Temperature calibration
	VL53L0X_PerformRefCalibration(TOF1, VhvSettings, PhaseCal);
	//System settings: Mode Single Ranging
	//VL53L0X_SetDeviceMode(TOF1, VL53L0X_DEVICEMODE_SINGLE_RANGING);

	VL53L0X_SetDeviceMode(TOF1, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	status = VL53L0X_SetGpioConfig (TOF1, TOF1_IN_Pin,  VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,  VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,  VL53L0X_INTERRUPTPOLARITY_HIGH);
	return status;

}

VL53L0X_Error TOF1_VL53L0X_Init_Single(uint32_t* refSpadCount,uint8_t* isApertureSpads,uint8_t* VhvSettings,uint8_t* PhaseCal, uint8_t mode)
{
	TOF1->I2cHandle = &HandleI2C;
	TOF1->I2cDevAddr = I2cDevAddr_TOF1;
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	HAL_GPIO_WritePin(TOF2_XSHUT_GPIO_Port, TOF2_XSHUT_Pin, GPIO_PIN_RESET); // Disable XSHUT
	HAL_Delay(20);
	HAL_GPIO_WritePin(TOF2_XSHUT_GPIO_Port, TOF2_XSHUT_Pin, GPIO_PIN_SET); // Enable XSHUT
	HAL_Delay(20);

	//Device initialization
	VL53L0X_WaitDeviceBooted( TOF1 );
	status = VL53L0X_DataInit( TOF1 );
	if (status!=VL53L0X_ERROR_NONE) return status;
	VL53L0X_StaticInit( TOF1 );
	//SPADs calibration
	VL53L0X_PerformRefSpadManagement(TOF1, refSpadCount, isApertureSpads);
	//Temperature calibration
	VL53L0X_PerformRefCalibration(TOF1, VhvSettings, PhaseCal);
	//System settings: Mode Single Ranging
	if (mode==0) status=VL53L0X_SetDeviceMode(TOF1, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	else{
		VL53L0X_SetDeviceMode(TOF1, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
		status = VL53L0X_SetGpioConfig (TOF1, TOF2_IN_Pin,  VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,  VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,  VL53L0X_INTERRUPTPOLARITY_HIGH);
	}

	return status;

}

void TOFX_VL53L0X_LongRangeSettings(VL53L0X_DEV TOFX)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	status = VL53L0X_SetLimitCheckEnable(TOFX, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);

	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetLimitCheckEnable(TOFX, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetLimitCheckValue(TOFX,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.1*65536));
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetLimitCheckValue(TOFX,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(60*65536));
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(TOFX,33000);
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetVcselPulsePeriod(TOFX,VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetVcselPulsePeriod(TOFX,VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
