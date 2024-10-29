/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>
#include <string.h>
/* Includes MSG Type */
#include <geometry_msgs/msg/twist.h>
#include <rrr_robot_interfaces/srv/rrr_target_path.h>
#include <rrr_robot_interfaces/srv/rrr_mode.h>
#include "rosidl_runtime_c/string_functions.h"
//#include

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* IDK */
rcl_init_options_t init_options;

/* Node */
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

/* Service Client*/
rcl_client_t savepath_client;
rcl_client_t ref_client;
rcl_client_t mode_client;

/* pub */
rcl_publisher_t cmdvel_publisher;

/* Massage */
geometry_msgs__msg__Twist twist_msg;
rrr_robot_interfaces__srv__RRRTargetPath_Request savepath_request;
rrr_robot_interfaces__srv__RRRMode_Request mode_request;
rrr_robot_interfaces__srv__RRRMode_Request ref_request;

/* Timer */
rcl_timer_t timer;

/* Start Joy Parameter */
//Joy XY
uint16_t ADC_RawRead[200] = { 0 };
uint16_t x_axis = 0;
uint16_t y_axis = 0;
uint16_t center_x = 0;
uint16_t center_y = 0;

// Joy Z
float linearZ_velocity = 0.0f;
uint8_t z_step = 0.1f;

//mode
uint16_t teleop_num = 0;
uint16_t ref_num = 0;

// Stare Button
GPIO_PinState aButtonState = GPIO_PIN_RESET;
GPIO_PinState bButtonState = GPIO_PIN_RESET;
GPIO_PinState cButtonState = GPIO_PIN_RESET;
GPIO_PinState dButtonState = GPIO_PIN_RESET;
GPIO_PinState kButtonState = GPIO_PIN_RESET;

// Prev Button
GPIO_PinState aPrevButton = GPIO_PIN_RESET;
GPIO_PinState bPrevButton = GPIO_PIN_RESET;
GPIO_PinState cPrevButton = GPIO_PIN_RESET;
GPIO_PinState dPrevButton = GPIO_PIN_RESET;
GPIO_PinState kPrevButton = GPIO_PIN_RESET;
/* End Joy Parameter */

//DEBUG
uint16_t DEBUG_Count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* Start MicroRos Function*/
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout, uint8_t *err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);
/* End MicroRos Function*/

/* Start my Function*/
void ReadADC_AVERAGE();
void SentCMDVEL();
void CheckButtonK();
void CheckButtonB();
void CheckButtonD();
// Client Callback
//void SavePath_client_callback(const rrr_robot_interfaces__srv__RRRTargetPath_Response * response_msg);
void ref_client_callback(const void * response_msg);
void mode_client_callback(const void * response_msg);
/* End my Function*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END Header_StartDefaultTask */

/* Start Out of Start DefaultTask */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	if (timer != NULL) {
		/* Code here*/
		ReadADC_AVERAGE();
		SentCMDVEL();
		CheckButtonK();
		CheckButtonB();
		CheckButtonD();
	}
}

void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */

	// micro-ROS configuration
	rmw_uros_set_custom_transport(true, (void*) &hlpuart1, cubemx_transport_open, cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator =
			rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	// micro-ROS app
	allocator = rcl_get_default_allocator();

	//create init_options
	init_options = rcl_get_zero_initialized_init_options();
	RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
	RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 50)); //Set Domain ID

	rclc_support_init_with_options(
			&support, 0,
			NULL,
			&init_options,
			&allocator
	);

	// create node
	rclc_node_init_default(
			&node,
			"cubemx_node",
			"",
			&support
	); //Node name

	// create cmd_vel publisher
	rclc_publisher_init_default(
			&cmdvel_publisher,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			"/cmd_vel"
	);

	// create save_path client
	rclc_client_init_default(
			&savepath_client,
			&node,
			ROSIDL_GET_SRV_TYPE_SUPPORT(rrr_robot_interfaces, srv, RRRTargetPath),
			"/SavePath"
	);

	// create Ref client
	rclc_client_init_default(
			&ref_client,
			&node,
			ROSIDL_GET_SRV_TYPE_SUPPORT(rrr_robot_interfaces, srv, RRRMode), //Wait to change
			"/Ref"
	);

	// create Mode client
	rclc_client_init_default(
			&mode_client,
			&node,
			ROSIDL_GET_SRV_TYPE_SUPPORT(rrr_robot_interfaces, srv, RRRMode), //Wait to change
			"/Mode"
	);

	// create Timer
	rclc_timer_init_default(
			&timer, &support,
			RCL_MS_TO_NS(10),
			timer_callback
	);

	// create executer
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 1, &allocator);
	rclc_executor_add_timer(&executor, &timer);
//	rclc_executor_add_client(&executor, &savepath_client, &savepath_response, SavePath_client_callback);
	rclc_executor_spin(&executor); //ต้องเรียกก่อนถึงจะเริ่มทำงาน

	for (;;) {
		osDelay(10);
	}
	/* USER CODE END 5 */
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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //    	rcl_send_request(&savepath_client, &savepath_request, &sequence_number);

  /* Start Analog Read */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, ADC_RawRead, 200);
  /* End Analog Read */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ReadADC_AVERAGE() {
	uint32_t temp_1 = 0;
	uint32_t temp_2 = 0;
	for (int i = 0; i < 200; i++) {
		if (i % 2 == 0) {
			temp_1 += ADC_RawRead[i];
		} else if (i % 2 == 1) {
			temp_2 += ADC_RawRead[i];
		}
	}
	x_axis = (temp_1 / 100);
	y_axis = (temp_2 / 100);
}


void SentCMDVEL(){
	/* Call Velocity */
	float linearX_velocity = (y_axis - 2048) / 2048.0f;  // Normalize -1.0 to 1.0
	float linearY_velocity = (x_axis - 2048) / 2048.0f; // Normalize -1.0 to 1.0

	/* Check DEADZONE*/
	if (fabs(linearX_velocity) < 0.025f) {
		linearX_velocity = 0.0f;
	}

	if (fabs(linearY_velocity) < 0.015f) {
		linearY_velocity = 0.0f;
	}

	/* Z button */
	aButtonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
	cButtonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);

	/* readbutton state */
	if (aButtonState == GPIO_PIN_RESET && aPrevButton == GPIO_PIN_SET) {
		linearZ_velocity += 0.1;

		HAL_Delay(10);

		/* update button state */
		aPrevButton = aButtonState;
		cPrevButton = cButtonState;
	    if (linearZ_velocity > 1.0f) linearZ_velocity = 1.0;
	}
	if (cButtonState == GPIO_PIN_RESET && cPrevButton == GPIO_PIN_SET) {
		linearZ_velocity -= 0.1;
	    if (linearZ_velocity < -1.0f) linearZ_velocity = -1.0;
	}

	if (fabs(linearZ_velocity) < 0.001f) {
		linearZ_velocity = 0.0f;
	}

	HAL_Delay(10);

	/* update button state */
	aPrevButton = aButtonState;
	cPrevButton = cButtonState;

	twist_msg.linear.x = linearX_velocity;
	twist_msg.linear.y = linearY_velocity;
	twist_msg.linear.z = linearZ_velocity;

	RCSOFTCHECK(rcl_publish(&cmdvel_publisher, &twist_msg, NULL));
}

void CheckButtonK() {
    kButtonState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

    if (kButtonState == GPIO_PIN_RESET && kPrevButton == GPIO_PIN_SET) {

    	rrr_robot_interfaces__srv__RRRTargetPath_Request__init(&savepath_request);

    	savepath_request.save_path = true;
    	savepath_request.call_path = false;

    	int64_t sequence_number;
    	RCSOFTCHECK(rcl_send_request(&savepath_client, &savepath_request, &sequence_number));

    }
    HAL_Delay(10);

    kPrevButton = kButtonState;
}

void CheckButtonB() {
    bButtonState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

    if (bButtonState == GPIO_PIN_RESET && bPrevButton == GPIO_PIN_SET) {

    	rrr_robot_interfaces__srv__RRRMode_Request__init(&mode_request);

    	teleop_num = teleop_num % 2;
    	teleop_num = teleop_num + 1;

    	if (teleop_num == 1){
    		rosidl_runtime_c__String__assign(&mode_request.mode_call, "Teleop");
    		rosidl_runtime_c__String__assign(&mode_request.ref, "");
    	}
		else if (teleop_num == 2){
			rosidl_runtime_c__String__assign(&mode_request.mode_call, "Auto");
			rosidl_runtime_c__String__assign(&mode_request.ref, "");
		};

    	int64_t sequence_number;
        RCSOFTCHECK(rcl_send_request(&mode_client, &mode_request, &sequence_number));
    }
    HAL_Delay(10);

    bPrevButton = bButtonState;
}


void CheckButtonD() {
    dButtonState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);

    if (dButtonState == GPIO_PIN_RESET && dPrevButton == GPIO_PIN_SET) {

    	rrr_robot_interfaces__srv__RRRMode_Request__init(&ref_request);


    	ref_num = ref_num % 2;
    	ref_num = ref_num + 1;

    	if (ref_num == 1){
    		rosidl_runtime_c__String__assign(&ref_request.mode_call, "Teleop");
    		rosidl_runtime_c__String__assign(&ref_request.ref, "base");
    	}
		else if (ref_num == 2){
			rosidl_runtime_c__String__assign(&ref_request.mode_call, "Teleop");
			rosidl_runtime_c__String__assign(&ref_request.ref, "hand");
		};

    	int64_t sequence_number;
    	RCSOFTCHECK(rcl_send_request(&ref_client, &ref_request, &sequence_number));
    }
    HAL_Delay(10);

    dPrevButton = dButtonState;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
