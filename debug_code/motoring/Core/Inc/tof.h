/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tof.h
  * @brief   This file contains all the function prototypes for
  *          the tof.c file
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef INC_TOF_H_
#define INC_TOF_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <inc/vl53l0x_api.h>
/* Exported types ------------------------------------------------------------*/
extern VL53L0X_Dev_t  vl53l0x_1; // first TOF	0x52
extern VL53L0X_DEV    TOF1;

extern VL53L0X_Dev_t  vl53l0x_2; // second TOF 0x54
extern VL53L0X_DEV    TOF2;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Private defines ------------------------------------------------------------*/
VL53L0X_Error TOF1_VL53L0X_Init(uint32_t*,uint8_t*,uint8_t*,uint8_t*);
VL53L0X_Error TOF1_VL53L0X_Init_Single(uint32_t*,uint8_t*,uint8_t*,uint8_t*, uint8_t);
VL53L0X_Error TOF2_VL53L0X_Init(uint32_t,uint8_t,uint8_t,uint8_t);
void TOFX_VL53L0X_LongRangeSettings(VL53L0X_DEV);
/* Exported function prototypes ------------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* INC_TOF_H_ */
