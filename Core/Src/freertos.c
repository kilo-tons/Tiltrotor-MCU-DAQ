/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "tim.h"
#include "uavcan.h"
#include "comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId CommIO_RdTskHandle;
osThreadId UAVCAN_TskHandle;
osThreadId Encoder_TskHandle;
osThreadId CommIO_RpTaskHandle;
osMutexId CommIO_inMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CommIO_RdThread(void const * argument);
void UAVCAN_Thread(void const * argument);
void Encoder_Thread(void const * argument);
void CommIO_RpThread(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of CommIO_inMutex */
  osMutexDef(CommIO_inMutex);
  CommIO_inMutexHandle = osMutexCreate(osMutex(CommIO_inMutex));

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
  /* definition and creation of CommIO_RdTsk */
  osThreadDef(CommIO_RdTsk, CommIO_RdThread, osPriorityLow, 0, 256);
  CommIO_RdTskHandle = osThreadCreate(osThread(CommIO_RdTsk), NULL);

  /* definition and creation of UAVCAN_Tsk */
  osThreadDef(UAVCAN_Tsk, UAVCAN_Thread, osPriorityNormal, 0, 384);
  UAVCAN_TskHandle = osThreadCreate(osThread(UAVCAN_Tsk), NULL);

  /* definition and creation of Encoder_Tsk */
  osThreadDef(Encoder_Tsk, Encoder_Thread, osPriorityRealtime, 0, 128);
  Encoder_TskHandle = osThreadCreate(osThread(Encoder_Tsk), NULL);

  /* definition and creation of CommIO_RpTask */
  osThreadDef(CommIO_RpTask, CommIO_RpThread, osPriorityBelowNormal, 0, 256);
  CommIO_RpTaskHandle = osThreadCreate(osThread(CommIO_RpTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_CommIO_RdThread */
/**
  * @brief  Function implementing the CommIO_RdTsk thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CommIO_RdThread */
void CommIO_RdThread(void const * argument)
{
  /* USER CODE BEGIN CommIO_RdThread */
	comm_IO_read_init();
	/* Infinite loop */
	for(;;)
	{
		// Wait for packet from UART interrupt
		osSemaphoreWait(CommIO_Read_SemHandle, osWaitForever);

		// Block DAQ_Input
		osMutexWait(CommIO_inMutexHandle, osWaitForever);

		switch(DAQ_input.func) {
		case READ:
			// Begin respond
			osSignalSet(CommIO_RpTaskHandle, COMM_RESPOND);
			break;

		case WRITE:
			// Write PWM signal to ESC
			htim3.Instance->CCR1 = DAQ_input.pwm - 1;

		case STOP:
			// Stop respond, turn off ESC
			osSignalSet(CommIO_RpTaskHandle, COMM_IDLE);
			htim3.Instance->CCR1 = 1000 - 1;
		}

		osMutexRelease(CommIO_inMutexHandle);
		// osDelay(1);
	}
  /* USER CODE END CommIO_RdThread */
}

/* USER CODE BEGIN Header_UAVCAN_Thread */
/**
* @brief Function implementing the UAVCAN_Tsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UAVCAN_Thread */
void UAVCAN_Thread(void const * argument)
{
  /* USER CODE BEGIN UAVCAN_Thread */
	uint32_t timestamp = 0;
  /* Infinite loop */
	for(;;)
	{

	if (HAL_GetTick() - timestamp > 1000) {
		timestamp = HAL_GetTick();
		HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
	}

	sendCanard();
	receiveCanard();
	spinCanard();
	osDelay(1);
	}
  /* USER CODE END UAVCAN_Thread */
}

/* USER CODE BEGIN Header_Encoder_Thread */
/**
* @brief Function implementing the Encoder_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Encoder_Thread */
void Encoder_Thread(void const * argument)
{
  /* USER CODE BEGIN Encoder_Thread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END Encoder_Thread */
}

/* USER CODE BEGIN Header_CommIO_RpThread */
/**
* @brief Function implementing the CommIO_RpTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommIO_RpThread */
void CommIO_RpThread(void const * argument)
{
  /* USER CODE BEGIN CommIO_RpThread */
  /* Infinite loop */
	for(;;)
	{
		osSignalWait(COMM_RESPOND, osWaitForever);

		osMutexWait(CommIO_inMutexHandle, osWaitForever);

		const uint16_t polling_period = DAQ_input.polling_period;
		const uint32_t heartbeat_timestamp = DAQ_input.timestamp;

		osMutexRelease(CommIO_inMutexHandle);

		const uint32_t tmp = HAL_GetTick();

		if (polling_period != 0 && (tmp - heartbeat_timestamp < HEARTBEAT_TIMEOUT) && heartbeat_timestamp != 0) {

			// Run code
			osMutexWait(Airspeed_MutexHandle, osWaitForever);

			float airspeed = uavcan_airspeed_data.airspeed;
			const uint32_t airspeed_timestamp = uavcan_airspeed_data.timestamp;

			osMutexRelease(Airspeed_MutexHandle);

			if (tmp - airspeed_timestamp > 2000) {
				airspeed = 0;
			}

			uint8_t buf[12];
			comm_IO_packet_respond_encode(RESPOND, 0, airspeed, buf);

			HAL_UART_Transmit_IT(&huart2, buf, COMM_BUFFER_LENGTH);

			// Hack, set the signal to RESPOND immediately because the function
			// will clear the signal, thus blocking the thread again

			osSignalSet(CommIO_RpTaskHandle, COMM_RESPOND);

		} else {
			// Stop thread and turn off ESC
			osSignalSet(CommIO_RpTaskHandle, COMM_IDLE);
			htim3.Instance->CCR1 = 1000 - 1;
		}

		osDelay(polling_period);
	}
  /* USER CODE END CommIO_RpThread */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

