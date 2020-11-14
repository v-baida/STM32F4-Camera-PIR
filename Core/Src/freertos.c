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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAITING_FOR_MOVEMENT 0x00
#define PICTURE_IS_BEING_SENT 0x01
#define PICTURE_IS_SENT 0x02
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define __HAL_GPIO_EXTI_MASK_IT(__EXTI_LINE__) (EXTI->IMR = (EXTI->IMR) & (~(__EXTI_LINE__)))
#define __HAL_GPIO_EXTI_UNMASK_IT(__EXTI_LINE__) (EXTI->IMR = (EXTI->IMR) | (__EXTI_LINE__))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile uint8_t displayState = 0;

extern uint8_t CAM_Frame_Buffer[CAM_VGA_WIDTH * CAM_VGA_HEIGHT * 2];
extern UART_HandleTypeDef huart3;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes =
{ .name = "defaultTask", .priority = (osPriority_t) osPriorityNormal,
		.stack_size = 256 * 4 };
/* Definitions for camTask */
osThreadId_t camTaskHandle;
const osThreadAttr_t camTask_attributes =
{ .name = "camTask", .priority = (osPriority_t) osPriorityLow, .stack_size = 256
		* 4 };
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes =
{ .name = "uartTask", .priority = (osPriority_t) osPriorityHigh, .stack_size =
		256 * 4 };
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes =
{ .name = "displayTask", .priority = (osPriority_t) osPriorityLow, .stack_size =
		256 * 4 };
/* Definitions for bCameraSemaphore */
osSemaphoreId_t bCameraSemaphoreHandle;
const osSemaphoreAttr_t bCameraSemaphore_attributes =
{ .name = "bCameraSemaphore" };
/* Definitions for bUartSemaphore */
osSemaphoreId_t bUartSemaphoreHandle;
const osSemaphoreAttr_t bUartSemaphore_attributes =
{ .name = "bUartSemaphore" };
/* Definitions for bDisplaySemaphore */
osSemaphoreId_t bDisplaySemaphoreHandle;
const osSemaphoreAttr_t bDisplaySemaphore_attributes =
{ .name = "bDisplaySemaphore" };

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCamTask(void *argument);
void StartUARTTask(void *argument);
void StartDisplayTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of bCameraSemaphore */
	bCameraSemaphoreHandle = osSemaphoreNew(1, 1, &bCameraSemaphore_attributes);

	/* creation of bUartSemaphore */
	bUartSemaphoreHandle = osSemaphoreNew(1, 1, &bUartSemaphore_attributes);

	/* creation of bDisplaySemaphore */
	bDisplaySemaphoreHandle = osSemaphoreNew(1, 1,
			&bDisplaySemaphore_attributes);

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
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of camTask */
	camTaskHandle = osThreadNew(StartCamTask, NULL, &camTask_attributes);

	/* creation of uartTask */
	uartTaskHandle = osThreadNew(StartUARTTask, NULL, &uartTask_attributes);

	/* creation of displayTask */
	displayTaskHandle = osThreadNew(StartDisplayTask, NULL,
			&displayTask_attributes);

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
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;)
	{
		printf("I'm in Default Task!\n\r");
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
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
void StartCamTask(void *argument)
{
	/* USER CODE BEGIN StartCamTask */
	/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(bCameraSemaphoreHandle, portMAX_DELAY);
		printf("I'm in Cam Task after semaphore!\n\r");
		osDelay(200);
		camera_startCap(CAMERA_CAP_SINGLE_FRAME, (void*) CAM_Frame_Buffer);
		osDelay(100);
		camera_stopCap();
		xSemaphoreGive(bUartSemaphoreHandle);
	}
	/* USER CODE END StartCamTask */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
 * @brief Function implementing the uartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument)
{
	/* USER CODE BEGIN StartUARTTask */
	/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(bUartSemaphoreHandle, portMAX_DELAY);

		displayState = PICTURE_IS_BEING_SENT;
		xSemaphoreGive(bDisplaySemaphoreHandle);

		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
		printf("I'm in UART Task after first semaphore!\n\r");
		HAL_UART_Transmit_DMA(&huart3, &CAM_Frame_Buffer[0], 65534);

		xSemaphoreTake(bUartSemaphoreHandle, portMAX_DELAY);
		printf("I'm in UART Task after second semaphore!\n\r");
		HAL_UART_Transmit_DMA(&huart3, &CAM_Frame_Buffer[65534], 65534);

		xSemaphoreTake(bUartSemaphoreHandle, portMAX_DELAY);
		printf("I'm in UART Task after third semaphore!\n\r");
		HAL_UART_Transmit_DMA(&huart3, &CAM_Frame_Buffer[131068], 22532);

		xSemaphoreTake(bUartSemaphoreHandle, portMAX_DELAY);

		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

		displayState = PICTURE_IS_SENT;
		xSemaphoreGive(bDisplaySemaphoreHandle);

//		__HAL_GPIO_EXTI_CLEAR_IT(PIR_SENSOR_Pin);
		__HAL_GPIO_EXTI_UNMASK_IT(PIR_SENSOR_Pin);
//		HAL_NVIC_EnableIRQ(PIR_SENSOR_EXTI_IRQn);
	}
	/* USER CODE END StartUARTTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the displayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
	/* USER CODE BEGIN StartDisplayTask */
	/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(bDisplaySemaphoreHandle, portMAX_DELAY);

		ST7735_FillScreen(ST7735_BLACK);
		switch (displayState)
		{
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
		}
	}
	printf("I'm in Display Task after semaphore!\n\r");

	/* USER CODE END StartDisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);

//	HAL_NVIC_DisableIRQ(PIR_SENSOR_EXTI_IRQn);
	__HAL_GPIO_EXTI_MASK_IT(PIR_SENSOR_Pin);

	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
//	if (GPIO_Pin == GPIO_PIN_13)
//	{
//		printf("I'm in EXTI Callback!\n\r");
//		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
//		xSemaphoreGiveFromISR(bCameraSemaphoreHandle,
//				&xHigherPriorityTaskWoken);
//	}

	if (GPIO_Pin == GPIO_PIN_9)
	{
		printf("I'm in EXTI Callback!\n\r");
//		printf("IPSR value = %d\n\r", __get_IPSR());
		xSemaphoreGiveFromISR(bCameraSemaphoreHandle,
				&xHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);

	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	printf("I'm in UART Callback\n\r");
	xSemaphoreGiveFromISR(bUartSemaphoreHandle, &xHigherPriorityTaskWoken);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
