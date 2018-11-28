#include <stm32f4xx_hal.h>
#include "main.h"
#include <string.h>


volatile HAL_StatusTypeDef status;

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim1;

static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);


// Encoder variables
int _CPR = 20; //CPR = counts per revolution
uint32_t last_mech = 0;
int _ppairs = 10;
float _offset = 2;
int rotations = 0;

void _Error_Handler(char *file, int line) {
	while(1) {} // Hang on error
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1){ //Interrupt Handler for PWM Timer
 HAL_GPIO_WritePin(GPIO(LED2),1);
}

int main(void) {
	// General initialization
	HAL_Init();
	SystemClock_Config();
	GPIO_BEGIN_INIT();
	uart_init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();

	DGPIO_INIT_OUT(LED1,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(LED2,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(LED3,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN1,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN2,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN3,GPIO_PIN_RESET);
	//DGPIO_INIT_OUT(BMSLED1,GPIO_PIN_SET);
	//DGPIO_INIT_OUT(BMSLED2,GPIO_PIN_SET);
	//DGPIO_INIT_OUT(BMSLED3,GPIO_PIN_SET);
	char message[] = "Successful initialization\r\n";
	uart_transmit(&message, HAL_MAX_DELAY);
	Set_PWM_Duty_Cycle(30,1);
	Set_PWM_Duty_Cycle(50,2);
	Set_PWM_Duty_Cycle(10,3);
	
	HAL_GPIO_WritePin(GPIO(EN1),1); //Turn on half bridges
	HAL_GPIO_WritePin(GPIO(EN2),1);
	HAL_GPIO_WritePin(GPIO(EN3),1);

	Encoder_Start();
	Encoder_Read();

	HAL_TIM_Base_Start_IT(&htim1); //Turn on Interrupt for the PWM TImer 

	int d=1;

	while(1) {
		//uart_transmit(&message, HAL_MAX_DELAY);
		//HAL_GPIO_TogglePin(GPIO(LED1));
		HAL_GPIO_TogglePin(GPIO(LED3));
		//HAL_GPIO_TogglePin(GPIO(LED3));
		Set_PWM_Duty_Cycle(d,2);
		Set_PWM_Duty_Cycle(100-d,1);
		d++;
		if(d==100) d=1;

		HAL_Delay(100);
	}

    return 0;
}

float Get_Mech_Pos() {                            //returns rotor angle in radians.
    uint32_t raw = Encoder_Read();
    if ((last_mech-raw)>3500) {rotations++;}
    if ((last_mech-raw)<-3500) {rotations--;}
    float unsigned_mech = (6.28318530718f/(float)_CPR)* (float) ((raw)  );//%_CPR);
    last_mech = raw;
    return (float) unsigned_mech + 6.28318530718f* (float) rotations;
}


float Get_Elec_Pos() {                            //returns rotor electrical angle in radians.
    uint32_t raw = Encoder_Read();
    float elec = ((6.28318530718f/(float)_CPR) * (float) ((_ppairs*raw)%_CPR)) - _offset;
    if(elec < 0) elec += 6.28318530718f;
    return elec;
}


/*
float Get_Elec_Vel() {                            //returns rotor electrical angle in radians, better for slow speed
    int raw = TIM2->CCR1;
    //   (8192/4 (quadrature) /8 (input prescaler)) / (7 * 2pi * 72,000,000)
    //1/(8192/4/8/(7*2*3.14159265*72000000)) 
    
    float elec_vel = 12370021/((float)raw)*4;

    return elec_vel;
}

void ZeroEncoderCount(void){
    if (ZSense->read() == 1){
       //wait_us(1);
       if (ZSense->read() == 1){
       TIM3->CNT = 0x000;
       if (!isEncoderZeroed1) {rotations = 0;}
       isEncoderZeroed1 = true;
       }
       }
}
*/

uint32_t Encoder_Read(void){
    return TIM2->CNT;
}

void Encoder_Start(void){
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void Encoder_Stop(void){
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
}

static void MX_TIM2_Init(void){

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_ADC1_Init(void){

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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

