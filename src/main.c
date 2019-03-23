#include <stm32f4xx_hal.h>
#include "main.h"
#include <string.h>


volatile HAL_StatusTypeDef status;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim1;

static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);


// Encoder variables
int _CPR = 4096; //CPR = counts per revolution
int _ppairs = 7;
float _offset = 6.0;
//float _offset = 5.88;

float reference_angle = 0.7;                                                

//Current variables

uint32_t adc_buf[2];
uint32_t curr_fb1;
uint32_t curr_fb2;
uint32_t curr_fb3;
uint32_t curr_offset1=1935;
uint32_t curr_offset2=1937;

uint32_t cal1;
uint32_t cal2;

float ia;
float ib;
float ic;

float id;
float iq;
float iq_set=500;
float iq_error;
float iq_error_sum=0;
float id_error;
float id_error_sum=0;

float Ki=0.000;
float Kp=.03;

//Voltage variables
float vq_set=28;
float vd_set=0.0;

float v_a;
float v_b;
float v_c;

float theta;
float theta_old;


bool flag=true;
bool adc_ready;

void _Error_Handler(char *file, int line) {
	while(1) {} // Hang on error
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1){
	//HAL_GPIO_WritePin(GPIO(LED2),0);	
	curr_fb1=adc_buf[0];
	curr_fb3=adc_buf[1];
	adc_ready = true;
}

void HAL_GPIO_EXT10_IRQHandler(uint16_t GPIO_Pin){
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
		HAL_GPIO_EXTI_Callback(GPIO_Pin);
	  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == GPIO_PIN_10){
	  TIM2->CNT=0;//Set encoder count to zero
  } 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1){//Interrupt Handler for PWM Timer

	HAL_GPIO_WritePin(GPIO(LED2),1);	
	adc_ready=false;
	HAL_ADC_Start_DMA(&hadc1,adc_buf,2);

//	while(!adc_ready){
//		printf("%d\r\n",adc_ready); //Wait for ADCs to finish converting
//
//	} 
	ia=(float)curr_fb1-(float)curr_offset1; //Should be around 1.558V
	ib=(float)curr_fb3-(float)curr_offset2;
	ic=-ia-ib;
	
	theta=Get_Elec_Pos();
	dq0(ia,ib,ic,&id,&iq,theta);

	/*
	abc(vd_set,vq_set,&v_a,&v_b,&v_c,Get_Elec_Pos());
	//abc(vd_set,vq_set,&v_a,&v_b,&v_c,reference_angle);

	v_a+=50;
	v_b+=50;
	v_c+=50;
	

	Set_PWM_Duty_Cycle((uint8_t)v_a,1);
	Set_PWM_Duty_Cycle((uint8_t)v_b,2);
	Set_PWM_Duty_Cycle((uint8_t)v_c,3);
	//reference_angle += .041;
	//if(reference_angle>6.28)reference_angle=0;
	
		
	//HAL_GPIO_WritePin(GPIO(LED2),0);	
	//else 
//		reference_angle-=.001;
	//if(reference_angle>=6.28) flag=false;
	//if(reference_angle<=0) flag=true;
	//reference_angle += .100;
	*/

	iq_error = iq_set-iq;
	iq_error_sum+=iq_error;

	vq_set=iq_error*Kp+iq_error_sum*Ki;
	//vq_set=iq_error*Kp;

	//id_error = -id;
	//id_error_sum+=id_error;

	//vd_set=id_error*Kp+id_error_sum*Ki;
	//vd_set=id_error*Kp;
	vd_set=0;

	abc(vd_set,vq_set,&v_a,&v_b,&v_c,theta);
	
	v_a+=50;
	v_b+=50;
	v_c+=50;
	
	
	Set_PWM_Duty_Cycle((uint8_t)v_a,1);
	Set_PWM_Duty_Cycle((uint8_t)v_b,2);
	Set_PWM_Duty_Cycle((uint8_t)v_c,3);
	
	HAL_GPIO_WritePin(GPIO(LED2),0);	
}

int main(void) {
	// General initialization
	HAL_Init();
	SystemClock_Config();
	GPIO_BEGIN_INIT();
	MX_GPIO_Init();
	MX_DMA_Init();
	uart_init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();

	//DGPIO_INIT_OUT(LED1,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(LED2,GPIO_PIN_RESET);
	//DGPIO_INIT_OUT(LED3,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN1,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN2,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN3,GPIO_PIN_RESET);

	Set_PWM_Duty_Cycle(50,1);
	Set_PWM_Duty_Cycle(50,2);
	Set_PWM_Duty_Cycle(50,3);
	
	printf("FPU Full Access\r\n");
	

	Encoder_Start();
	Encoder_Read();
	printf("Encoder started\r\n");


	HAL_GPIO_WritePin(GPIO(EN1),1); //Turn on half bridges
	HAL_GPIO_WritePin(GPIO(EN2),1);
	HAL_GPIO_WritePin(GPIO(EN3),1);
	printf("Half bridges on \r\n");
	HAL_ADC_Start(&hadc1);
	printf("ADC Started\r\n");
	printf("Successful initialization\r\n");
	HAL_TIM_Base_Start_IT(&htim1); //Turn on Interrupt for the PWM TImer 
	
	/**Main loop**/
	while(1) {
		/*
		curr_fb1+=cal1;
		curr_fb1/=2;

		curr_fb2+=cal2;
		curr_fb2/=2;

		HAL_Delay(200);
		cal1=curr_fb1;
		cal2=curr_fb2;
		*/
		//printf("%f %f %f\r\n",ia,ic,ib);
		//printf("%f %f\r\n",id,iq);
		SLO_PRINTF("%f\r\n",vq_set);
		//printf("%f %f %f\r\n",v_a,v_b,v_c);
		//printf("%lu %lu\r\n",curr_fb1,curr_fb3);
		//printf("%f\r\n", Get_Elec_Pos(),reference_angle);
		HAL_Delay(200);
	}

    return 0;
}
void abc(float d, float q, float *a, float *b, float *c,float theta){
    /// Inverse DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///

    float cf = cosf(theta);
    float sf = sinf(theta);
    
    *a = cf*d - sf*q;                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf)*d - (-0.86602540378f*cf-.5f*sf)*q;
    *c = (-0.86602540378f*sf-.5f*cf)*d - (0.86602540378f*cf-.5f*sf)*q;
}
    
void dq0(float a, float b, float c, float *d, float *q, float theta){
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    
    float cf = cosf(theta);
    float sf = sinf(theta);
    
    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);
       
}

float Get_Mech_Pos(){
	uint32_t raw=Encoder_Read();
   	float unsigned_mech = (6.28318530718f/(float)_CPR) * (float) (raw%_CPR);
    if(unsigned_mech < 0) unsigned_mech += 6.28318530718f;
	else if(unsigned_mech > 6.28318530718f) unsigned_mech -= 6.28318530718f;
    return (float) unsigned_mech;// + 6.28318530718f* (float) rotations;
}

float Get_Elec_Pos() {
	//returns rotor electrical angle in radians.
    uint32_t raw = Encoder_Read();
    float elec = ((6.28318530718f/(float)_CPR) * (float) ((_ppairs*raw)%_CPR)) - _offset;
    if(elec < 0) elec += 6.28318530718f;
	else if(elec > 6.28318530718f) elec -= 6.28318530718f;
    return elec;
}



uint32_t Encoder_Read(void){
    return TIM2->CNT;
}

void Encoder_Start(void){
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void Encoder_Stop(void){
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
}

static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

}

static void MX_ADC1_Init(void){

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE; 
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 150;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}
