/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "semphr.h"
#include "camera.h"
#include "st7735.h"
#include "stdio.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAITING_FOR_CONNECTION  0x00
#define WAITING_FOR_MOVEMENT 	0x01
#define PICTURE_IS_BEING_SENT 	0x02
#define PICTURE_IS_SENT 		0x03
#define CONNECTION_IS_DONE      0x04

#define PICTURE_SIZE CAM_VGA_WIDTH * CAM_VGA_HEIGHT * 2
/*We send PICTURE_DIVIDER pieces of picture in order to minimize buffer size
 Every piece is a cropped original picture*/
#define PICTURE_DIVIDER 8
#define BUFFER_SIZE  PICTURE_SIZE/PICTURE_DIVIDER
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define __HAL_GPIO_EXTI_MASK_IT(__EXTI_LINE__) (EXTI->IMR = (EXTI->IMR) & (~(__EXTI_LINE__)))
#define __HAL_GPIO_EXTI_UNMASK_IT(__EXTI_LINE__) (EXTI->IMR = (EXTI->IMR) | (__EXTI_LINE__))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t CAM_Frame_Buffer[BUFFER_SIZE] =
{ 0 };

volatile uint8_t displayState = 0;

osSemaphoreId bDCMIComplitedSemaphoreHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId camTaskHandle;
osThreadId displayTaskHandle;
osSemaphoreId bCameraSemaphoreHandle;
osSemaphoreId bEthSemaphoreHandle;
osSemaphoreId bDisplaySemaphoreHandle;

extern DCMI_HandleTypeDef hdcmi;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void tcp_thread(void *arg);

void tcpecho_init(void);
void USER_DCMI_IRQHandler(DCMI_HandleTypeDef* phdcmi);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartCamTask(void const * argument);
void StartDisplayTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of bCameraSemaphore */
  osSemaphoreDef(bCameraSemaphore);
  bCameraSemaphoreHandle = osSemaphoreCreate(osSemaphore(bCameraSemaphore), 1);

  /* definition and creation of bEthSemaphore */
  osSemaphoreDef(bEthSemaphore);
  bEthSemaphoreHandle = osSemaphoreCreate(osSemaphore(bEthSemaphore), 1);

  /* definition and creation of bDisplaySemaphore */
  osSemaphoreDef(bDisplaySemaphore);
  bDisplaySemaphoreHandle = osSemaphoreCreate(osSemaphore(bDisplaySemaphore), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  osSemaphoreDef(bDCMIComplitedSemaphore);
  bDCMIComplitedSemaphoreHandle = osSemaphoreCreate(osSemaphore(bDCMIComplitedSemaphore), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of camTask */
  osThreadDef(camTask, StartCamTask, osPriorityLow, 0, 512);
  camTaskHandle = osThreadCreate(osThread(camTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, StartDisplayTask, osPriorityLow, 0, 512);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */

	__HAL_GPIO_EXTI_MASK_IT(USER_Btn_Pin);

	osSemaphoreWait(bEthSemaphoreHandle, 10);
	osSemaphoreWait(bCameraSemaphoreHandle, 10);

	displayState = WAITING_FOR_CONNECTION;
	osSemaphoreRelease(bDisplaySemaphoreHandle);

	tcpecho_init();
	osThreadSetPriority(NULL, osPriorityIdle);

	/* Infinite loop */
	for (;;)
	{
		osDelay(1000);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCamTask */
/**
 * @brief Function implementing the camTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCamTask */
void StartCamTask(void const * argument)
{
  /* USER CODE BEGIN StartCamTask */
	uint16_t crop_start_line = 0;
	/* Infinite loop */
	for (;;)
	{

		for (uint8_t i = 0; i < PICTURE_DIVIDER; i++)
		{
			osSemaphoreWait(bCameraSemaphoreHandle, osWaitForever);
			crop_start_line = i * CAM_VGA_HEIGHT / PICTURE_DIVIDER;
			camera_startCapCropped(CAMERA_CAP_SINGLE_FRAME,
					(void*) CAM_Frame_Buffer, crop_start_line,
					CAM_VGA_HEIGHT / PICTURE_DIVIDER);
//			osDelay(200);
			printf("I`m waiting for DCMI semaphore!\n\r");
			osSemaphoreWait(bDCMIComplitedSemaphoreHandle, osWaitForever);
//			osSemaphoreWait(bCameraSemaphoreHandle, osWaitForever);
			osSemaphoreRelease(bEthSemaphoreHandle);
		}
	}
  /* USER CODE END StartCamTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the displayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	/* Infinite loop */
	for (;;)
	{
		osSemaphoreWait(bDisplaySemaphoreHandle, osWaitForever);

		printf("I'm in Display Task after semaphore!\n\r");

		ST7735_FillScreen(ST7735_BLACK);
		switch (displayState)
		{
		case WAITING_FOR_CONNECTION:
			ST7735_WriteString(ST7735_WIDTH / 10 * 1, ST7735_HEIGHT / 10 * 2,
					"WAITING FOR\n CONNECTION!", Font_11x18,
					ST7735_WHITE, ST7735_BLACK);
			break;
		case WAITING_FOR_MOVEMENT:
			ST7735_WriteString(ST7735_WIDTH / 10 * 1, ST7735_HEIGHT / 10 * 2,
					"MOVEMENT\nWEREN'T DETECTED!", Font_11x18,
					ST7735_WHITE, ST7735_BLACK);
			break;

		case PICTURE_IS_BEING_SENT:
			ST7735_WriteString(ST7735_WIDTH / 10 * 1, ST7735_HEIGHT / 10 * 2,
					"WAIT,\nTHE PICTURE IS BEING SENT!", Font_11x18,
					ST7735_WHITE, ST7735_BLACK);
			break;

		case PICTURE_IS_SENT:
			ST7735_WriteString(ST7735_WIDTH / 10 * 1, ST7735_HEIGHT / 10 * 2,
					"THE PICTURE HAS BEEN SENT!", Font_11x18,
					ST7735_WHITE, ST7735_BLACK);
			break;
		case CONNECTION_IS_DONE:
			ST7735_WriteString(ST7735_WIDTH / 10 * 1, ST7735_HEIGHT / 10 * 2,
					"CONNECTION\n IS DONE", Font_11x18,
					ST7735_WHITE, ST7735_BLACK);
			break;
		}
	}

  /* USER CODE END StartDisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void tcpecho_init()
{
	sys_thread_new("tcp_thread", tcp_thread, NULL, DEFAULT_THREAD_STACKSIZE * 2,
	TCPIP_THREAD_PRIO);
}

static void tcp_thread(void *arg)
{
	err_t err, accept_err, recv_err;
	struct netconn *conn;
	struct netconn *newconn;
	struct netbuf *buf;

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);
	if (conn != NULL)
	{
		/* Bind connection to well known port number 7. */
		err = netconn_bind(conn, NULL, 7);
		if (err == ERR_OK)
		{
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);
			while (1)
			{
				/* Grab new connection. */
				accept_err = netconn_accept(conn, &newconn);
				displayState = CONNECTION_IS_DONE;
				osSemaphoreRelease(bDisplaySemaphoreHandle);
				/* Process the new connection. */
				if (accept_err == ERR_OK)
				{
					__HAL_GPIO_EXTI_UNMASK_IT(USER_Btn_Pin);
					while ((recv_err = netconn_recv(newconn, &buf)) == ERR_OK)
					{
						osSemaphoreWait(bEthSemaphoreHandle, osWaitForever);
						displayState = PICTURE_IS_BEING_SENT;
						osSemaphoreRelease(bDisplaySemaphoreHandle);
						for (uint8_t i = 0; i < PICTURE_DIVIDER - 1; i++)
						{
							netconn_write(newconn, CAM_Frame_Buffer,
									BUFFER_SIZE, NETCONN_COPY);
							osSemaphoreRelease(bCameraSemaphoreHandle);
							osSemaphoreWait(bEthSemaphoreHandle, osWaitForever);
						}

						netconn_write(newconn, CAM_Frame_Buffer, BUFFER_SIZE,
								NETCONN_COPY);
						netbuf_delete(buf);

						displayState = PICTURE_IS_SENT;
						osSemaphoreRelease(bDisplaySemaphoreHandle);
//						osSemaphoreRelease(bDCMIComplitedSemaphoreHandle);
						__HAL_GPIO_EXTI_UNMASK_IT(USER_Btn_Pin);
					}
					/* Close connection and discard connection identifier. */
					netconn_close(newconn);
					netconn_delete(newconn);
				}
			}
		}
		else
		{
			netconn_delete(newconn);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);

	__HAL_GPIO_EXTI_MASK_IT(USER_Btn_Pin);
	if (GPIO_Pin == GPIO_PIN_13)
	{
		osSemaphoreRelease(bCameraSemaphoreHandle);
	}

}

void USER_DCMI_IRQHandler(DCMI_HandleTypeDef* phdcmi)
{
	if (phdcmi->Instance == hdcmi.Instance)
	{
		printf("I`m releasing DCMI semaphore!\n\r");
		camera_stopCap();
		osSemaphoreRelease(bDCMIComplitedSemaphoreHandle);
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
