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
int _CPR = 4096; //CPR = counts per revolution
int _ppairs = 14;
float _offset = 3.5;

float reference_angle = 0;                                                

//Current variables

uint32_t curr_fb1;
uint32_t curr_fb2;
uint32_t curr_fb3;
uint32_t curr_offset=1830;

float ia;
float ib;
float ic;

float id;
float iq;
float iq_set=100;
float iq_error;
float iq_error_sum=0;
float id_error;
float id_error_sum=0;

float Ki=0.0;
float Kp=.8;

//Voltage variables
float vq_set=0;
float vd_set=500;

float v_a;
float v_b;
float v_c;


bool flag=true;

void _Error_Handler(char *file, int line) {
	while(1) {} // Hang on error
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1){//Interrupt Handler for PWM Timer
	
	HAL_GPIO_WritePin(GPIO(LED2),1);	
	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_fb1 = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_fb3 = HAL_ADC_GetValue(&hadc1);

	//HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	//curr_fb2 = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);
	


	ia=(float)curr_fb1-(float)curr_offset; //Should be around 1.558V
	ic=(float)curr_fb3-(float)curr_offset;
	ib=(float)curr_fb2-(float)curr_offset;
	//printf("%lu\r\n",curr_fb2,(unsigned long) curr_fb2,(unsigned long) curr_fb3);
	//printf("%f %f %f\r\n", ia,ib,ic);

	
	
	
	abc(vd_set,vq_set,&v_a,&v_b,&v_c,reference_angle);
	/*
	
	v_a/=11;
	v_b/=11;
	v_c/=11;

	v_a+=50;
	v_b+=50;
	v_c+=50;
	
	*/	
	
	//printf("%d %d %d\r\n",(uint8_t) v_a,(uint8_t) v_b,(uint8_t) v_c);
	
	//printf("%f ",Get_Mech_Pos());
	//printf("%f\r\n",reference_angle);

	//Set_PWM_Duty_Cycle((uint8_t)v_a,1);
	//Set_PWM_Duty_Cycle((uint8_t)v_b,2);
	//Set_PWM_Duty_Cycle((uint8_t)v_c,3);

	//HAL_Delay(1);                                                         //give the rotor some time to settle into position
	//if(flag)
		reference_angle += .08;
		
	HAL_GPIO_WritePin(GPIO(LED2),0);	
	//else 
//		reference_angle-=.001;
	//if(reference_angle>=6.28) flag=false;
	//if(reference_angle<=0) flag=true;
	//printf("%f\r\n",reference_angle);
	//reference_angle += .100;

	/*
	dq0(ia,ib,ic,&id,&iq);
	//printf("%f ",id);
	//printf("%f\r\n",iq);

	iq_error = iq_set-iq;
	iq_error_sum+=iq_error;
	//printf("%f ", iq);
	//printf("%f ", iq_error);

	vq_set=iq_error*Kp+iq_error_sum*Ki;

	id_error = -id;
	id_error_sum+=id_error;

	vd_set=id_error*Kp+id_error_sum*Ki;
	//printf("%f ",vq_set);
	//printf("%f\r\n",vd_set);

	abc(vd_set,vq_set,&v_a,&v_b,&v_c,Get_Elec_Pos());

	v_a/=15;
	v_b/=15;
	v_c/=15;
	
	v_a+=50;
	v_b+=50;
	v_c+=50;
	
	printf("%d %d %d\r\n",(uint8_t)v_a,(uint8_t)v_b,(uint8_t)v_c);
	
	Set_PWM_Duty_Cycle((uint8_t)v_a,1);
	Set_PWM_Duty_Cycle((uint8_t)v_b,2);
	Set_PWM_Duty_Cycle((uint8_t)v_c,3);
	*/
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

	//DGPIO_INIT_OUT(LED1,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(LED2,GPIO_PIN_RESET);
	//DGPIO_INIT_OUT(LED3,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN1,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN2,GPIO_PIN_RESET);
	DGPIO_INIT_OUT(EN3,GPIO_PIN_RESET);
	/*
	Set_PWM_Duty_Cycle(30,1);
	Set_PWM_Duty_Cycle(50,2);
	Set_PWM_Duty_Cycle(10,3);
	*/
	

	Encoder_Start();
	Encoder_Read();
	printf("Encoder started\r\n");


	

	//Calibrating for current offset
	/*
	uint32_t temp1;
	uint32_t temp3;

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_offset1 = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_offset3 = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	for (int i=0;i<100;i++){
		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
		temp1 = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
		temp3 = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Stop(&hadc1);
		//printf("%d\r\n", i);
		HAL_Delay(100);

		curr_offset1+=temp1;
		curr_offset1/=2;
		curr_offset3+=temp3;
		curr_offset3/=2;
	}
	printf("Curr offset1: %lu\r\n",(unsigned long)curr_offset1);
	printf("Curr offset3: %lu\r\n",(unsigned long)curr_offset3);
	printf("ADC Calibrated\r\n");
	*/

	HAL_GPIO_WritePin(GPIO(EN1),1); //Turn on half bridges
	HAL_GPIO_WritePin(GPIO(EN2),1);
	HAL_GPIO_WritePin(GPIO(EN3),1);
	printf("Half bridges on \r\n");
	printf("Successful initialization\r\n");
	HAL_TIM_Base_Start_IT(&htim1); //Turn on Interrupt for the PWM TImer 
	
	
	
	/*
	Set_PWM_Duty_Cycle(0,1);
	Set_PWM_Duty_Cycle(0,2);
	Set_PWM_Duty_Cycle(0,3);
	
	uint32_t cal1;
	uint32_t cal3;
	*/
	
	
	/*
	vd_set=500;
	vq_set=00;
	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_fb1 = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_fb3 = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);
	//printf("%lu ",(unsigned long) curr_fb1);
	//printf("Curr fb3: %lu \r\n",(unsigned long) curr_fb3);

	ia=(float)curr_fb1-(float)curr_offset1; //Should be around 1.558V
	ic=(float)curr_fb3-(float)curr_offset3;
	//printf("%f ", ia);
	//printf("%f\r\n",ic);
	while(reference_angle<=7*6.28){
		abc(vd_set,vq_set,&v_a,&v_b,&v_c,reference_angle);
		v_a/=10;
		v_b/=10;
		v_c/=10;

		v_a+=50;
		v_b+=50;
		v_c+=50;
		
		//printf("%d ",(uint8_t) v_a);
		//printf("%d ",(uint8_t) v_b);
		//printf("%d\r\n",(uint8_t) v_c);
		
		//printf("%f ",Get_Mech_Pos());
		//printf("%f\r\n",reference_angle);

		Set_PWM_Duty_Cycle((uint8_t)v_a,1);
		Set_PWM_Duty_Cycle((uint8_t)v_b,2);
		Set_PWM_Duty_Cycle((uint8_t)v_c,3);

		if(flag)
			reference_angle += .01;
		else 
			reference_angle-=.01;
		if(reference_angle>=6.28){
		   	///flag=false;
			//reference_angle=0;
		}
		//if(reference_angle<=0) flag=true;
		printf("%f ",Get_Mech_Pos());
		printf("%f\r\n",reference_angle);
		HAL_Delay(2);
	}
	*/
	
	

	while(1) {
		/*	
		HAL_GPIO_WritePin(GPIO(LED2),1);	
		
		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
		 HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
		HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Stop(&hadc1);
		HAL_GPIO_WritePin(GPIO(LED2),0);	
		//printf("cal1: %lu\r\n",(unsigned long) cal1);
		//printf("cal3: %lu\r\n",(unsigned long) cal3);
		HAL_Delay(1000);
		*/
	}

    return 0;
}
void abc(float d, float q, float *a, float *b, float *c,float theta){
    /// Inverse DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
	//float theta= Get_Elec_Pos();
	//printf("%f ",theta);
    float cf = cos(theta);
    float sf = sin(theta);
    
    *a = cf*d - sf*q;                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf)*d - (-0.86602540378f*cf-.5f*sf)*q;
    *c = (-0.86602540378f*sf-.5f*cf)*d - (0.86602540378f*cf-.5f*sf)*q;
}
    
    
void dq0(float a, float b, float c, float *d, float *q){
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    
	
	float theta= Get_Elec_Pos();
    float cf = cos(theta);
    float sf = sin(theta);
    
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

float Get_Elec_Pos() {                            //returns rotor electrical angle in radians.
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
  hadc1.Init.Resolution = ADC_RESOLUTION_6B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
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

