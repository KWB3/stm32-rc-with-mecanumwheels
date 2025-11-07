/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01p.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define uart_test					// UART 테스트용 정의, 주석제거시 활성화 uart3 사용
#define RX_BUFFER_SIZE 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
Byte0   : 0xAB            // HEADER (시작 마커)
Byte1   : seq             // 시퀀스 번호 (0..255)
Byte2   : joy0_x  	  	// 좌스틱 X (0..255)
Byte3   : joy0_y   		// 좌스틱 Y (0..255)
Byte4   : joy1_x 	 	 // 우스틱 X (0..255)
Byte5   : joy1_y 	 	// 우스틱 Y (0..255)
Byte6   : btn_flags (u8)  // 버튼 비트맵: bit0..3=버튼1..4,
Byte7   : checksum (u8)   // simple checksum (예: sum of bytes1..6) & 0xFF
Byte8   : 0xBA            // TAIL (끝 마커)
 */
#ifdef uart_test
uint8_t rx_data;                // 1바이트 수신 버퍼
uint8_t rx_buffer[RX_BUFFER_SIZE]; // 수신 문자열 버퍼
uint16_t rx_index = 0;
uint8_t rxx_text[RX_BUFFER_SIZE]; // 수신 문자열 버퍼
#endif

uint8_t rxx_data[NRF24L01P_PAYLOAD_LENGTH] = {127,127,127,127,127,127,127,127,127};
uint8_t rx_flag;

typedef enum {
	PACKET_OK = 0,           // 정상 패킷
	PACKET_INVALID_HEADER,   // 헤더가 잘못됨
	PACKET_INVALID_TAIL,     // 테일이 잘못됨
	PACKET_INVALID_CS,      // checksum 잘못됨
} PacketStatus_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);

	return len;
}
#ifdef uart_test
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		if (rx_data == '\n' || rx_data == '\r')
		{
			rx_buffer[rx_index] = '\0'; // 문자열 종료

			// 에코 출력
			printf("%s\r\n", rx_buffer);
			rx_index = 0; // 인덱스 초기화
		}
		else
		{
			if (rx_index < RX_BUFFER_SIZE - 1)
			{
				rx_buffer[rx_index++] = rx_data;
			}
		}

		// 다음 바이트 수신 대기
		HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	}
}
#endif

PacketStatus_t checkPackit()
{
	if(rxx_data[0] != 0xAB)
	{
		return PACKET_INVALID_HEADER;
	}

	if(rxx_data[8] != 0xBA)
	{
		return PACKET_INVALID_TAIL;
	}

	if(((rxx_data[1] + rxx_data[2] + rxx_data[3]+ rxx_data[4]+ rxx_data[5] + rxx_data[6]) & 0xFF) != rxx_data[7])
	{
		return PACKET_INVALID_CS;
	}

	return PACKET_OK;
}

void lf_backward()
{
	HAL_GPIO_WritePin(m1c1_GPIO_Port, m1c1_Pin, 1);
	HAL_GPIO_WritePin(m1c2_GPIO_Port, m1c2_Pin, 0);
}

void lf_forward()
{
	HAL_GPIO_WritePin(m1c1_GPIO_Port, m1c1_Pin, 0);
	HAL_GPIO_WritePin(m1c2_GPIO_Port, m1c2_Pin, 1);
}

void rf_forward()
{
	HAL_GPIO_WritePin(m2c1_GPIO_Port, m2c1_Pin, 1);
	HAL_GPIO_WritePin(m2c2_GPIO_Port, m2c2_Pin, 0);
}

void rf_backward()
{
	HAL_GPIO_WritePin(m2c1_GPIO_Port, m2c1_Pin, 0);
	HAL_GPIO_WritePin(m2c2_GPIO_Port, m2c2_Pin, 1);
}

void lr_forward()
{
	HAL_GPIO_WritePin(m3c1_GPIO_Port, m3c1_Pin, 1);
	HAL_GPIO_WritePin(m3c2_GPIO_Port, m3c2_Pin, 0);
}

void lr_backward()
{
	HAL_GPIO_WritePin(m3c1_GPIO_Port, m3c1_Pin, 0);
	HAL_GPIO_WritePin(m3c2_GPIO_Port, m3c2_Pin, 1);
}

void rr_backward()
{
	HAL_GPIO_WritePin(m4c1_GPIO_Port, m4c1_Pin, 1);
	HAL_GPIO_WritePin(m4c2_GPIO_Port, m4c2_Pin, 0);
}

void rr_forward()
{
	HAL_GPIO_WritePin(m4c1_GPIO_Port, m4c1_Pin, 0);
	HAL_GPIO_WritePin(m4c2_GPIO_Port, m4c2_Pin, 1);
}
void brake()
{
	HAL_GPIO_WritePin(m1c1_GPIO_Port, m1c1_Pin, 1);
	HAL_GPIO_WritePin(m1c2_GPIO_Port, m1c2_Pin, 1);
	HAL_GPIO_WritePin(m2c1_GPIO_Port, m2c1_Pin, 1);
	HAL_GPIO_WritePin(m2c2_GPIO_Port, m2c2_Pin, 1);
	HAL_GPIO_WritePin(m3c1_GPIO_Port, m3c1_Pin, 1);
	HAL_GPIO_WritePin(m3c2_GPIO_Port, m3c2_Pin, 1);
	HAL_GPIO_WritePin(m4c1_GPIO_Port, m4c1_Pin, 1);
	HAL_GPIO_WritePin(m4c2_GPIO_Port, m4c2_Pin, 1);
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
}
/*
 * Numbers represent each direction
 *										<roller direction>
 *           7   8   9			      (lf)  \\\		///  (rf)
 *           4  body 6						    body
 *           1   2   3				  (lr)	///		\\\  (rr)
 *
 * Each direction describes the direction of rotation of the motor.
 * Check the roller direction of the Mecanum wheel.
 * lf - left front(pwm1), rf - right front(pwm2), lr - left rear(pwm3), rr - right rear(pwm4)
 *
 *       9 - lf, rr motors rotate forward
 *       8 - All motors rotate forward
 *       7 - rf, lr motors rotate forward
 *       6 - lf, rr motors rotate forward, rf, lr motors rotate backward
 *       4 - rf, lr motors rotate forward, lf, rr motors rotate backward
 *       3 - rf, lr motors rotate backward
 *       2 - All motors rotate backward
 *       1 - lf, rr motors rotate backward
 */
void direction9(uint8_t ccr)
{
	lf_forward();
	rr_forward();
	TIM1->CCR1 = ccr;
	TIM1->CCR2 =  0;
	TIM1->CCR3 =  0;
	TIM1->CCR4 =  ccr;
}

void direction8(uint8_t ccr)
{
	lf_forward();
	rr_forward();
	rf_forward();
	lr_forward();
	TIM1->CCR1 =  ccr;
	TIM1->CCR2 =  ccr;
	TIM1->CCR3 =  ccr;
	TIM1->CCR4 =  ccr;
}

void direction7(uint8_t ccr)
{
	rf_forward();
	lr_forward();
	TIM1->CCR1 = 0;
	TIM1->CCR2 = ccr;
	TIM1->CCR3 = ccr;
	TIM1->CCR4 = 0;
}

void direction6(uint8_t ccr)
{
	rf_backward();
	lr_backward();
	lf_forward();
	rr_forward();
	TIM1->CCR1 =  ccr;
	TIM1->CCR2 =  ccr;
	TIM1->CCR3 =  ccr;
	TIM1->CCR4 =  ccr;
}

void direction4(uint8_t ccr)
{
	rf_forward();
	lr_forward();
	lf_backward();
	rr_backward();
	TIM1->CCR1 =  ccr;
	TIM1->CCR2 =  ccr;
	TIM1->CCR3 =  ccr;
	TIM1->CCR4 =  ccr;
}

void direction3(uint8_t ccr)
{
	rf_backward();
	lr_backward();
	TIM1->CCR1 = 0;
	TIM1->CCR2 = ccr;
	TIM1->CCR3 = ccr;
	TIM1->CCR4 = 0;
}

void direction2(uint8_t ccr)
{
	lf_backward();
	lr_backward();
	rr_backward();
	rf_backward();
	TIM1->CCR1 = ccr;
	TIM1->CCR2 = ccr;
	TIM1->CCR3 = ccr;
	TIM1->CCR4 = ccr;
}

void direction1(uint8_t ccr)
{
	lf_backward();
	rr_backward();
	TIM1->CCR1 = ccr;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = ccr;
}

void turnLeft()
{
	lf_backward();
	lr_backward();
	rf_forward();
	rr_forward();
	TIM1->CCR1 = 100;
	TIM1->CCR2 = 100;
	TIM1->CCR3 = 100;
	TIM1->CCR4 = 100;
}

void turnRight()
{
	lf_forward();
	lr_forward();
	rf_backward();
	rr_backward();
	TIM1->CCR1 = 100;
	TIM1->CCR2 = 100;
	TIM1->CCR3 = 100;
	TIM1->CCR4 = 100;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_USART3_UART_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */

	nrf24l01p_rx_init(2500, _1Mbps);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
#ifdef uart_test
	HAL_UART_Receive_IT(&huart3, &rx_data, 1);
#endif
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	while (1)
	{

		if((rx_flag == 1) && (checkPackit() == PACKET_OK))
		{
			if(rxx_data[6] & 0x01) //bt1 pushed
			{
				turnLeft();
			}
			else if(rxx_data[6] & 0x02) //bt2 pushed
			{
				turnRight();
			}
			else if(rxx_data[3] > 135)
			{
				direction8(rxx_data[3]-127);
			}
			else if(rxx_data[3] < 120)
			{
				direction2(127 - rxx_data[3]);
			}
			else if(rxx_data[2] > 135)
			{
				direction4(rxx_data[2]-127);
			}
			else if(rxx_data[2] < 120)
			{
				direction6(127 - rxx_data[2]);
			}
			else if(rxx_data[4] > 135)
			{
				direction7(rxx_data[4]-127);
			}
			else if(rxx_data[4] < 120)
			{
				direction3(127 - rxx_data[4]);
			}
			else if(rxx_data[5] > 135)
			{
				direction9(rxx_data[5]-127);
			}
			else if(rxx_data[5] < 120)
			{
				direction1(127 - rxx_data[5]);
			}
			else
			{
				brake();
			}

			rx_flag = 0;
		}

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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
	/* EXTI2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
	{
		rx_flag = 1;
		nrf24l01p_rx_receive(rxx_data);
#ifdef uart_test
		sprintf(rxx_text, "%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\r\n", rxx_data[0], rxx_data[1], rxx_data[2], rxx_data[3], rxx_data[4], rxx_data[5], rxx_data[6], rxx_data[7], rxx_data[8]);
		HAL_UART_Transmit(&huart3, rxx_text, RX_BUFFER_SIZE, HAL_MAX_DELAY);
#endif
	}

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
