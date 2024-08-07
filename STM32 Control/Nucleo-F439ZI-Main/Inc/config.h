/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : config.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __CONFIG_H
#define __CONFIG_H

//initial setting is 3000, because 3x300 measured normal delta time
//this was tested and it is ~1sec
#define TEAM_AUTOWARE 0
#define TEAM_HUMDA 1

#define SELECTED_TEAM TEAM_HUMDA

#ifdef __cplusplus
extern "C" {
#endif

/* Common suggested configuration */
#define DELTA_TIME_THRESHOLD 1000

//CAN comm
#define ID_MCU   0x100 //Mid level cmd from Main ECU
#define ID_TBW   0x101 //TBW
#define ID_BBW   0x102 //BBW
#define ID_LSBW   0x103 //LSBW
#define ID_USBW  0x104 //USBW
#define ID_HCU  0x110 //High level CAN cmd
#define ID_RC  0x105 //High level CAN cmd

//mode of operation
#define SOURCE_CAN 1
#define SOURCE_USART 0
#define HIGH_LEVEL_CMD_SOURCE SOURCE_USART 0
#define HCU_TIMEOUT_CAN 500

/* TEAM configurations */
/* TEAM HUMDA */
#if SELECTED_TEAM == TEAM_HUMDA
//initial setting is 3000, because 3x300 measured normal delta time
//this was tested and it is ~1sec
#define DELTA_TIME_THRESHOLD 1000
#define HIGH_LEVEL_CMD_SOURCE SOURCE_CAN

#endif

/* TEAM AUTOWARE */
#if SELECTED_TEAM == TEAM_AUTOWARE

#endif


#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
