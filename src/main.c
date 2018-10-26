#include <stm32f4xx_hal.h>
#include "main.h"
#include <string.h>


volatile HAL_StatusTypeDef status;
UART_Struct uart3;
char message[50];

void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void _Error_Handler(char *file, int line) {
	while(1) {} // Hang on error
}



int main(void) {
	// General initialization
	HAL_Init();
	SystemClock_Config();
	GPIO_BEGIN_INIT();
	uart_init(&uart3);

	DGPIO_INIT_OUT(LED1,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(LED2,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(LED3,GPIO_PIN_RESET);
	char message[] = "Successful initialization\r\n";
	//uart_transmit(&uart3, &message, sizeof(message), 200);
	//uart_transmit(&uart3,&d,200);

	while(1) {
		status = uart_transmit(&uart3,message,10,100);
		//uart_transmit(&uart3, &message, sizeof(message), 200);
		//uart_transmit_char(&uart3, &d, 200);
		HAL_GPIO_TogglePin(GPIO(LED2));
		if (status==HAL_OK)
			HAL_GPIO_WritePin(GPIO(LED3),1);
		//HAL_Delay(500);
	}

    return 0;
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	// Configure the main internal regulator output voltage 
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the CPU, AHB and APB busses clocks 
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 256;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

    // Initializes the CPU, AHB and APB busses clocks 
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SDIO
								|RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
	PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	// Configure the Systick interrupt time 
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	// Configure the Systick 
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	// SysTick_IRQn interrupt configuration
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
