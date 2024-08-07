/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "uart_serial.h"
#include "config.h"

#include <math.h>

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void compute_auto_brake();
void autonomous_speed_throttle_pid();

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
int gokart_mode = 0;
int gokart_EM_mode = 0;

float throttle_desired = 0.0;

float speed_measured = 0.0;
float speed_desired = 0.0;

float steer_desired = 0.0;
float steer_measured = 0.0;
float steer_max = 55.0;

float steering_wheel = 0.0;

float brake_desired = 0.0;
float brake_measured = 0.0;
float brake_max = 150.0;

uint8_t reverse_desired = 0;

float steer_desired_can = 0;
float speed_desired_can = 0;
float brake_desired_can = 0;
float throttle_desired_can = 0;
uint8_t reverse_desired_can = 0;
uint32_t hcu_previous_time_can = 0;
uint32_t hcu_timeout_can = 1;
uint32_t hcu_delta_time_can = 0;

app_state_t app;
app_state_t *main_app;

CAN_TxHeaderTypeDef TxHeader[2];
CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;

uint8_t CAN_TxData[8];
uint8_t CAN_RxData[8];

// Throttle PID
float pid_speed_error = 0.0;
float pid_speed_error_pre = 0.0;
float pid_integral = 0.0;
float pid_derivative = 0.0;
float p_term = 0.0;
float i_term = 0.0;
float d_term = 0.0;

//Control Disconnect variable
volatile uint32_t ctrl_connected = 0;
volatile uint32_t ctrl_connection_counter = 0;
volatile uint32_t prev_time = 0;
uint32_t delta_time = 0;

// char UART_TxData[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int Debug_UART_Get_Byte() {
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) == SET) {
		return (uint8_t) (huart3.Instance->DR & (uint8_t) 0x00FF);
	}
	return -1;
}

// This function merges array a, b, c into array d in sequence (no sort)
void mergearray(char a[], char b[], char c[], char d[], int arr1size,
		int arr2size, int arr3size) {
	// resultant Array Size Declaration
	int i, j;

	// copying array 1 elements in to c array
	for (i = 0; i < arr1size; i++) {
		d[i] = a[i];
	}

	// copying array 2 elements in to c array
	for (i = 0, j = arr1size; j < arr1size + arr2size && i < arr2size;
			i++, j++) {
		d[j] = b[i];
	}

	// copying array 3 elements in to c array
	for (i = 0, j = arr1size + arr2size;
			j < arr1size + arr2size + arr3size && i < arr3size; i++, j++) {
		d[j] = c[i];
	}
}

void send_gokart_info(float steer, float speed, int is_info) {
	char data[6];
	char info_steer[13];
	char info_speed[13];
	char info_type[11];
	char info_out[37];

	sprintf(data, "%.2f", steer / 180.0 * 3.14);
	mergearray("steer ", data, " ", info_steer, 6, 6, 1);

	sprintf(data, "%.2f", speed);
	mergearray("speed ", data, " ", info_speed, 6, 6, 1);

	if (is_info == 0) {
		mergearray("type ", "cmmd", "\r\n", info_type, 5, 4, 2);
	} else {
		mergearray("type ", "info", "\r\n", info_type, 5, 4, 2);
	}

	mergearray(info_steer, info_speed, info_type, info_out, 13, 13, 11);

	HAL_UART_Transmit(&huart6, info_out, sizeof(info_out), 10); // Sending in normal mode
	//HAL_UART_Transmit(&huart6, '/n', 1, 10); // Sending in normal mode
}

void handle_remote_command() {
	float acc_percent = app.acc_percent;

	float steer_percent = app.steer_percent;

	reverse_desired = (uint8_t)app.gokart_reverse;

	if (acc_percent > 0.00) {
		throttle_desired = 1.75 * acc_percent * 100.0;
		throttle_desired = max(min(throttle_desired, 100.0), 0.0);
		brake_desired = 0.0;
	} else {
		throttle_desired = 0.0;
		brake_desired = -acc_percent * brake_max;
	}

//  if (gear_shift == 0)
//  {
//    motor_direction = 0;
//  }
//  else
//  {
//    motor_direction = 1;
//  }

	steer_desired = steer_percent * steer_max;

	printf("steer desired: %f\r\n", steer_desired);
}

void handle_autonomous_command() {
	printf("acc_percent: %f\r\n", app.acc_percent);
	if (app.acc_percent <= 0.00)
	{
		emergency_stop();
	} else
	{
		brake_desired = 0.0;
		if (HIGH_LEVEL_CMD_SOURCE == SOURCE_USART) {
			uint8_t place_holder[10];
			sscanf(drive_msg, "%s %f %s %f", place_holder, &steer_desired,
					place_holder, &speed_desired);
		}
		if (HIGH_LEVEL_CMD_SOURCE == SOURCE_CAN) {
			if (hcu_timeout_can == 1) {
				steer_desired = 0;
				speed_desired = 0;
				reverse_desired = 0;
				emergency_stop();
			} else {
				steer_desired = steer_desired_can;
				speed_desired = speed_desired_can;
				brake_desired = (float)brake_desired_can;
				reverse_desired = reverse_desired_can;
			}
		}
		autonomous_speed_throttle_pid();
				//	compute_auto_brake();
	}
}

void handle_manual_command() {
	steer_desired = steering_wheel;
}

#define MAX_THROTTLE 40 // replace with your maximum throttle value
#define MIN_THROTTLE 15 // replace with your minimum throttle value
#define WAVE_FREQ 1     // frequency of the cosine wave in Hz
int loop_count = 0.0;

// Assuming you call this function every 25ms (40Hz)
float calculateThrottle(int loopCount) {
	float amplitude = (MAX_THROTTLE - MIN_THROTTLE) / 2.0;
	float offset = (MAX_THROTTLE + MIN_THROTTLE) / 2.0;
	float phase = (float) loopCount / 40 * WAVE_FREQ * 2.0 * M_PI; // 40Hz control loop
	return amplitude * cos(phase) + offset;
}

void autonomous_speed_throttle_pid() {
	float Kp = 75; // Best Kp = 2.0
	float Ki = 0.0;
	float Kd = 0.0;
	float min_throttle = 0.0;//90.0;
	float pid_cycle_time = 0.200;

	pid_speed_error = speed_desired - speed_measured;
	p_term = Kp * pid_speed_error;

	pid_integral += pid_speed_error * pid_cycle_time;
	i_term = Ki * pid_integral;

	pid_derivative = (pid_speed_error - pid_speed_error_pre) / pid_cycle_time;
	d_term = Kd * pid_derivative;
	pid_speed_error_pre = pid_speed_error;

	throttle_desired = min_throttle + p_term + d_term + i_term;

	printf("p_term: %f \r\n", p_term);
	printf("i_term: %f \r\n", i_term);
	printf("d_term: %f \r\n", d_term);
	printf("pid_speed_error: %f \r\n", pid_speed_error);
	printf("pid_speed_error_pre: %f \r\n", pid_speed_error_pre);
	printf("throttle_desired: %f \r\n", throttle_desired);
	printf("speed_desired: %f \r\n", speed_desired);
	printf("speed_measured: %f \r\n", speed_measured);
	printf("brake measured: %f\r\n", brake_measured);
	printf("\r\n");
}

void compute_auto_brake() {
	float speed_error = speed_desired - speed_measured;

	if (speed_error < -1.5) {
		brake_desired = (-speed_error - 1.5) * brake_max / 2;
	} else {
		brake_desired = 0.0;
	}
}

void cast_command() {
	if (steer_desired > steer_max) {
		steer_desired = steer_max;
	} else if (steer_desired < -steer_max) {
		steer_desired = -steer_max;
	}

	if (brake_desired > brake_max) {
		brake_desired = brake_max;
	} else if (brake_desired < 0.0) {
		brake_desired = 0.0;
	}

	if (throttle_desired > 100.0) {
		throttle_desired = 100.0;
	} else if (throttle_desired < 0.0) {
		throttle_desired = 0.0;
	}
}

void send_command() {
	// Send CANBus command
	CAN_TxData[0] = (int) (steer_desired + steer_max);
	CAN_TxData[1] = (int) (brake_desired);
	CAN_TxData[2] = (int) (throttle_desired);
	CAN_TxData[3] = (int) (reverse_desired);
	CAN_TxData[4] = (int) (gokart_mode);
	CAN_TxData[5] = (int) (gokart_EM_mode);

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader[0], CAN_TxData, &TxMailbox);
}

void send_diag_rc(uint16_t data0, uint16_t data1, uint16_t data2,
		uint16_t data3) {
	// Send CANBus command
	CAN_TxData[0] = (uint16_t) data0;
	CAN_TxData[2] = (uint16_t) data1;
	CAN_TxData[4] = (uint16_t) data2;
	CAN_TxData[6] = (uint16_t) data3;
	//CAN_TxData[4] = app->rc_receiver_state.channels[0].servo_position;
	//CAN_TxData[5] = app->rc_receiver_state.channels[0].servo_position;

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader[1], CAN_TxData, &TxMailbox);
}
void print_info() {
	if (gokart_mode == 0) {
		printf("mode: remote \r\n");
	} else if (gokart_mode == 1) {
		printf("mode: autonomous \r\n");
	} else if (gokart_mode == 2) {
		printf("mode: manual \r\n");
	}

	if (reverse_desired == 1) {
		printf("motor_direction: reverse \r\n\n");
	} else {
		printf("motor_direction: forward \r\n");
	}

	printf("throttle desired %.2f \r\n\n", throttle_desired);
	printf("steer desired %.2f ", steer_desired);
	printf("steer measured %.2f \r\n", steer_measured);
	printf("brake desired %.2f ", brake_desired);
	printf("brake measured %.2f \r\n", brake_measured);
	printf("speed desired %.2f  ", speed_desired);
	printf("speed measured %.2f \r\n", speed_measured);
	printf("\r\n");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, CAN_RxData)
			!= HAL_OK) {
		Error_Handler();
	}

	if (RxHeader.StdId == ID_TBW) {
		speed_measured = CAN_RxData[0] / 10.0;
	}

	if (RxHeader.StdId == ID_BBW) {
		brake_measured = CAN_RxData[0];
	}

	if (RxHeader.StdId == ID_LSBW) {
		steer_measured = CAN_RxData[0] - steer_max;
	}

	if (RxHeader.StdId == ID_USBW) {
		steering_wheel = CAN_RxData[0] - steer_max;
	}

	if (SELECTED_TEAM == TEAM_HUMDA) {
		if (RxHeader.StdId == ID_HCU) {
			//identical layout as ID_MCU
			steer_desired_can = CAN_RxData[0] - steer_max;
			brake_desired_can = CAN_RxData[1];
			throttle_desired_can = CAN_RxData[2];

			//TODO for temporary testing!!!
			speed_desired_can = throttle_desired_can;

			reverse_desired_can = CAN_RxData[3];

			//example tx msg
			//CAN_TxData[0] = (int) (steer_desired + steer_max);
			//CAN_TxData[1] = (int) (brake_desired);
			//CAN_TxData[2] = (int) (throttle_desired);
			//CAN_TxData[3] = (int) (reverse_desired);

			hcu_previous_time_can = HAL_GetTick();
			hcu_timeout_can = 0;
		}

	}

}

void emergency_stop() {
	brake_desired = brake_max;
	throttle_desired = 0.0;
}

/**
 * @brief Periodic call back function triggered by TIM modules. Execute different functions depending on htim.
 * @param htim The TIM instance that triggers this call back.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// 40Hz - 25ms handles braking on control disconnect
	if (htim == &htim13) {
		//printf("\r\nControl State: %d", ctrl_connected);
		//delta_time is around 300 normally
		delta_time = HAL_GetTick() - prev_time;
		if (delta_time > DELTA_TIME_THRESHOLD)
			if (ctrl_connection_counter > 0)
				ctrl_connection_counter--;

		//CAN timeout check
		hcu_delta_time_can = HAL_GetTick() - hcu_previous_time_can;
		if (HCU_TIMEOUT_CAN < hcu_delta_time_can)
			hcu_timeout_can = 1;

		//rc receive saturation check
		if (ctrl_connection_counter < CTRL_SATURATION_THRESHOLD) {
			ctrl_connected = 0;
		} else {
			ctrl_connected = 1;
		}
	}

	// 40Hz - 25ms handle gokart command and send to subsystems
	if (htim == &htim6) {
		if (ctrl_connected = 0) {
			emergency_stop();
		} else {
			gokart_mode = app.control_mode;
			gokart_EM_mode = app.gokart_EM_status;

			if (gokart_mode == 0) {
				handle_remote_command();
			}
			if (gokart_mode == 1) {
				handle_autonomous_command();
			}
			//this comes last to be able to override all previous cmds
			if (gokart_EM_mode) {
				emergency_stop();
			} else if (gokart_mode == 2) {
				// handle_manual_command();
				//not used now, should not happen

				emergency_stop();
			}
		}

		cast_command();
		send_command();
	}

	// 40Hz - 25ms send out gokart drive info to higher level device
	if (htim == &htim7) {
		// the current gokart drive state information
		send_gokart_info(steer_measured, speed_measured, 1);
	}

	// 10Hz - 100ms send out gokart drive command to higher level device
	if (htim == &htim10) {
		send_gokart_info(steer_desired, speed_desired, 0);
	}

	// 5Hz - 200ms print gokart info
	if (htim == &htim11) {
		//		 print_info();
		//if (gokart_mode == 1) {
		//	handle_autonomous_command();
		//	cast_command();
		//	send_command();
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

/***************************************************************************/
//Spectrum binding
#define NUM_BIND_PULSES 3

void spektrum_send_pulses(char num) {
	char i;

	for (i = 0; i < num; i++) {
		// no documentation of requirements for pulse width but 200us and a
		// duty cycle of 50% was tested succesfully
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		//10000000 is like 1 sec
		for (int i = 0; i < 1000; i++) {
		}
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		for (int i = 0; i < 1000; i++) {
		}
	}
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
}
/***************************************************************************/

int Bind_Spectrum() {
	//This pin should be the data pin
	//HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	//This pin should be the Power pin
	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);

	// put the Spektrum RX into bind mode
	// pulses must be sent within 200ms of Spektrum RX powerup
	HAL_Delay(100);
	spektrum_send_pulses(NUM_BIND_PULSES);
	return 0;
}

int main(void) {
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
	MX_USART3_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_CAN1_Init();
	MX_ADC1_Init();
	MX_USART6_UART_Init();
	MX_TIM6_Init();
	MX_I2C1_Init();
	MX_TIM7_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_TIM13_Init();
	/* USER CODE BEGIN 2 */

	main_app = &app;
	app_run(&app);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	//binding remote controller SPM9645
	//Receiver needs to be powered from 5V
	//Receiver data pin shall be connected to G7 same as LD1 LED on nucleo 439
	Bind_Spectrum();

	while (1) {

	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue =
	RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	hadc1.Init.ExternalTrigConvEdge =
	ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */
	TxHeader[0].DLC = 8;
	TxHeader[0].ExtId = 0;
	TxHeader[0].IDE = CAN_ID_STD;
	TxHeader[0].RTR = CAN_RTR_DATA;
	TxHeader[0].StdId = ID_MCU;
	TxHeader[0].TransmitGlobalTime = DISABLE;

	TxHeader[1].DLC = 8;
	TxHeader[1].ExtId = 0;
	TxHeader[1].IDE = CAN_ID_STD;
	TxHeader[1].RTR = CAN_RTR_DATA;
	TxHeader[1].StdId = ID_RC;
	TxHeader[1].TransmitGlobalTime = DISABLE;
	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 6;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10; // Which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

	canfilterconfig.FilterIdHigh = ID_TBW << 5; // ID 0x101 left-aligned in 16-bit field
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = ID_BBW << 5; // ID 0x102 left-aligned in 16-bit field
	canfilterconfig.FilterMaskIdLow = 0x0000;

	if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK) {
		// Filter configuration Error
		Error_Handler();
	}

	canfilterconfig.FilterBank = 11; // Use another filter bank for the next set of IDs
	canfilterconfig.FilterIdHigh = ID_LSBW << 5; // ID 0x103 left-aligned in 16-bit field
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = ID_USBW << 5; // ID 0x104 left-aligned in 16-bit field
	canfilterconfig.FilterMaskIdLow = 0x0000;

	if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK) {
		// Filter configuration Error
		Error_Handler();
	}

	canfilterconfig.FilterBank = 11; // Use another filter bank for the next set of IDs
	canfilterconfig.FilterIdHigh = ID_HCU << 5; // ID 0x103 left-aligned in 16-bit field
	canfilterconfig.FilterIdLow = 0x0000;
	canfilterconfig.FilterMaskIdHigh = 0x0000 << 5; // ID 0x104 left-aligned in 16-bit field
	canfilterconfig.FilterMaskIdLow = 0x0000;

	if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK) {
		// Filter configuration Error
		Error_Handler();
	}
	//HAL_CAN_Start(&hcan1);
	//HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_Prepare();

	/* USER CODE END CAN1_Init 2 */

}

//this is also a working filter, not used, for only std msg-s
void CAN_filter_config() {
	/*
	 CAN_FilterTypeDef sFilter_config; // Create a new filter config structure for each call
	 sFilter_config.SlaveStartFilterBank = 14;
	 sFilter_config.FilterBank = 0;
	 sFilter_config.FilterMode = CAN_FILTERMODE_IDLIST;
	 sFilter_config.FilterScale = CAN_FILTERSCALE_16BIT;
	 sFilter_config.FilterIdHigh = ID1 << 5;
	 sFilter_config.FilterIdLow = ID2 << 5;

	 sFilter_config.FilterMaskIdHigh = ID3 << 5;

	 sFilter_config.FilterMaskIdLow = ID4 << 5;

	 sFilter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	 sFilter_config.FilterActivation = CAN_FILTER_ENABLE;

	 if (HAL_CAN_ConfigFilter(&hcan1, &sFilter_config) != HAL_OK) {
	 Error_Handler();
	 }
	 */
}

//Can filter
//from here:
//https://community.st.com/t5/stm32-mcus-products/stm32f103-can-filter-range-of-id-s/td-p/620967
void CAN_filter_config_ext(uint8_t filter_index, uint32_t ID, uint32_t MASK,
		uint32_t FIFO) {
	//CAN comm
	/*
	 #define REMOTE_FRAME   0 /* If = 1 the frame should be a remote frame. If = 0 the frame will be a data frame */
	/*
	 *
	 */
	/*#define EXTID          0 /* If = 0 the frame should be a frame with standard ID. If = 1 the frame should be a frame with extended ID */
	//
	/*
	 #define FILTER_ID1    ((ID1 << 3) | (REMOTE_FRAME<<1) | (EXTID <<2))  // ID1 = 0x0CF00400 (FxFR1[31:0])
	 #define FILTER_MASK1 ((ID2 << 3) | (REMOTE_FRAME<<1) | (EXTID <<2))  // ID2 = 0x0CFD9200 (FxFR2[31:0])
	 #define FILTER_ID2    ((ID3 << 3) | (REMOTE_FRAME<<1) | (EXTID <<2))  // ID1 = 0x0CF00400 (FxFR1[31:0])
	 #define FILTER_MASK2 ((ID4 << 3) | (REMOTE_FRAME<<1) | (EXTID <<2))  // ID2 = 0x0CFD9200 (FxFR2[31:0])
	 #define FILTER_ID3    ((ID5 << 3) | (REMOTE_FRAME<<1) | (EXTID <<2))  // ID1 = 0x0CF00400 (FxFR1[31:0])
	 #define FILTER_MASK3 ((ID5 << 3) | (REMOTE_FRAME<<1) | (EXTID <<2))  // ID2 = 0x0CFD9200 (FxFR2[31:0])

	 CAN_FilterTypeDef sFilter_config; // Create a new filter config structure for each call
	 sFilter_config.SlaveStartFilterBank = 14;
	 sFilter_config.FilterBank = filter_index;
	 sFilter_config.FilterMode = CAN_FILTERMODE_IDLIST;
	 sFilter_config.FilterScale = CAN_FILTERSCALE_32BIT;
	 sFilter_config.FilterIdHigh = (ID1 >> 16);
	 sFilter_config.FilterIdLow = (ID & 0xFFFF);
	 sFilter_config.FilterMaskIdHigh = (MASK >> 16);
	 sFilter_config.FilterMaskIdLow = (MASK & 0xFFFF);
	 sFilter_config.FilterFIFOAssignment = FIFO;
	 sFilter_config.FilterActivation = CAN_FILTER_ENABLE;

	 if (HAL_CAN_ConfigFilter(&hcan1, &sFilter_config) != HAL_OK) {
	 Error_Handler();
	 }
	 */
}

//better Activatenotification
void CAN_Prepare() {
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX0 & RX1 fifos notifications #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan1,
	CAN_IT_RX_FIFO0_MSG_PENDING |
	CAN_IT_RX_FIFO0_FULL |
	CAN_IT_RX_FIFO0_OVERRUN |
	CAN_IT_RX_FIFO1_MSG_PENDING |
	CAN_IT_RX_FIFO1_FULL |
	CAN_IT_RX_FIFO1_OVERRUN |
	CAN_IT_ERROR_WARNING |
	CAN_IT_ERROR_PASSIVE |
	CAN_IT_BUSOFF |
	CAN_IT_LAST_ERROR_CODE |
	CAN_IT_ERROR) != HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1,
	I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 3 * 168 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 200 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload =
	TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC,
	TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput =
	TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 168 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 25000;
	htim6.Init.AutoReloadPreload =
	TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 168 - 1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 25000;
	htim7.Init.AutoReloadPreload =
	TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */
	HAL_TIM_Base_Start_IT(&htim7);
	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 336 - 1;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 50000;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload =
	TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */
	HAL_TIM_Base_Start_IT(&htim10);
	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 1344 - 1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 25000;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload =
	TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */
	HAL_TIM_Base_Start_IT(&htim11);
	/* USER CODE END TIM11_Init 2 */

}

/**
 * @brief TIM13 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM13_Init(void) {

	/* USER CODE BEGIN TIM13_Init 0 */

	/* USER CODE END TIM13_Init 0 */

	/* USER CODE BEGIN TIM13_Init 1 */

	/* USER CODE END TIM13_Init 1 */
	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 9999;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 199;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload =
	TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM13_Init 2 */
	HAL_TIM_Base_Start_IT(&htim13);
	/* USER CODE END TIM13_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */
	uart_serial_start(&huart6);
	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
	GPIO_InitStruct.Pin =
	RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PG4 USB_OverCurrent_Pin Speed_Sensor_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | USB_OverCurrent_Pin | Speed_Sensor_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : PG5 USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
	GPIO_InitStruct.Pin =
	USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_VBUS_Pin */
	GPIO_InitStruct.Pin = USB_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

	HAL_Delay(500);

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	HAL_Delay(500);
	//	while (1) {
	//	}
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
