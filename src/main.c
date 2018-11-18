#include <stm32f4xx_hal.h>
#include "main.h"
#include <string.h>


volatile HAL_StatusTypeDef status;

TIM_HandleTypeDef htim1;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void _Error_Handler(char *file, int line) {
	while(1) {} // Hang on error
}



int main(void) {
	// General initialization
	HAL_Init();
	SystemClock_Config();
	GPIO_BEGIN_INIT();
	uart_init();
	MX_TIM1_Init();

	DGPIO_INIT_OUT(LED1,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(LED2,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(LED3,GPIO_PIN_RESET);
	//DGPIO_INIT_OUT(BMSLED1,GPIO_PIN_SET);
	//DGPIO_INIT_OUT(BMSLED2,GPIO_PIN_SET);
	//DGPIO_INIT_OUT(BMSLED3,GPIO_PIN_SET);
	char message[] = "Successful initialization\r\n";
	uart_transmit(&message, HAL_MAX_DELAY);
	Set_PWM_Duty_Cycle(30,1);
	Set_PWM_Duty_Cycle(50,2);
	Set_PWM_Duty_Cycle(90,3);

	while(1) {
		//uart_transmit(&message, HAL_MAX_DELAY);
		HAL_GPIO_TogglePin(GPIO(LED1));
		HAL_GPIO_TogglePin(GPIO(LED2));
		HAL_GPIO_TogglePin(GPIO(LED3));
		//HAL_GPIO_TogglePin(GPIO(BMSLED1));
		//HAL_GPIO_TogglePin(GPIO(BMSLED2));
		//HAL_GPIO_TogglePin(GPIO(BMSLED3));
		HAL_Delay(200);
	}

    return 0;
}


void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;


  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

