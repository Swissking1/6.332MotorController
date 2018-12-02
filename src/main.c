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
int _CPR = 4090; //CPR = counts per revolution
uint32_t last_mech = 0;
int _ppairs = 10;
float _offset = 2;
int rotations = 0;


//Current variables

uint32_t curr_fb1;
uint32_t curr_fb3;
uint32_t curr_offset;

int32_t ia;
int32_t ib;
int32_t ic;

int32_t i_alpha;
int32_t i_beta;

int32_t id;
int32_t iq;
int32_t iq_set=5;
int32_t iq_error;
int32_t iq_error_sum=0;
int32_t id_error;
int32_t id_error_sum=0;

float Ki=0;
float Kp=.01;

//Voltage variables
uint32_t vq_set;
uint32_t vd_set;

uint32_t v_alpha;
uint32_t v_beta;

uint32_t v_a;
uint32_t v_b;
uint32_t v_c;


char oof[]="oof\r\n";

void _Error_Handler(char *file, int line) {
	while(1) {} // Hang on error
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1){ //Interrupt Handler for PWM Timer
	
	HAL_GPIO_WritePin(GPIO(LED2),1);
	
	
	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_fb1 = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_fb3 = HAL_ADC_GetValue(&hadc1);


	HAL_ADC_Stop(&hadc1);

	ia=curr_fb1-curr_offset; //Should be around 1.558V
	ic=curr_fb3-curr_offset;

	ib=-ib-ic;

	i_alpha=ia;
	i_beta=clarke(ia,ib);

	iq=park(i_alpha,i_beta,1);
	id=park(i_alpha,i_beta,0);

	iq_error = iq_set-iq;
	iq_error_sum+=iq_error;

	vq_set=iq_error*Kp+iq_error_sum*Ki;

	id_error = -id;
	id_error_sum+=id_error;

	vd_set=id_error*Kp+id_error_sum*Ki;

	v_alpha=inverse_park(vq_set,vd_set,1);
	v_beta=inverse_park(vq_set,vd_set,0);

	v_a=v_alpha;
	v_b=inverse_clarke(v_alpha,v_beta,1);
	v_c=inverse_clarke(v_alpha,v_beta,0);

	v_a/=42946673;
	v_b/=42946673;
	v_c/=42946673;

	printf("%lu \r\n",(unsigned long)v_a);	
	Set_PWM_Duty_Cycle(v_a,1);
	Set_PWM_Duty_Cycle(v_b,2);
	Set_PWM_Duty_Cycle(v_c,3);
	
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
	//Set_PWM_Duty_Cycle(30,1);
	//Set_PWM_Duty_Cycle(50,2);
	//Set_PWM_Duty_Cycle(10,3);
	
	HAL_GPIO_WritePin(GPIO(EN1),1); //Turn on half bridges
	HAL_GPIO_WritePin(GPIO(EN2),1);
	HAL_GPIO_WritePin(GPIO(EN3),1);

	Encoder_Start();
	Encoder_Read();


	char message[] = "Successful initialization\r\n";
	uart_transmit(&message, HAL_MAX_DELAY);
	

	int d=1;

	//Calibrating for current offset
	uint32_t temp;

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
	curr_offset = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	for (int i=0;i<1000;i++){
		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY); //Wait for ADC conversion
		temp = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Stop(&hadc1);
		curr_offset+=temp;
		curr_offset/=2;
	}
	char adc_good[]="ADC Calibrated\r\n";
	uart_transmit(&adc_good, HAL_MAX_DELAY);
	uart_transmit(&oof,HAL_MAX_DELAY);
	//HAL_TIM_Base_Start_IT(&htim1); //Turn on Interrupt for the PWM TImer 

	while(1) {
		//HAL_GPIO_TogglePin(GPIO(LED3));
		printf("Encoder Raw: %lu\r\n", (unsigned long)Encoder_Read());
		printf("Get mechanical position: %f\r\n", Get_Mech_Pos());
	
		/*
		//HAL_GPIO_TogglePin(GPIO(LED3));
		Set_PWM_Duty_Cycle(d,2);
		Set_PWM_Duty_Cycle(100-d,1);
		d++;
		if(d==100) d=1;
	

		*/
		HAL_Delay(100);
	}

    return 0;
}

int32_t park(int32_t i_alpha, int32_t i_beta, bool flag){
	float pos=Get_Mech_Pos();
	if(flag)// flag is iq
		return -i_alpha*sin(pos)+i_beta*cos(pos);
	else
		return i_alpha*cos(pos)+i_beta*sin(pos);

}

int32_t clarke(int32_t ia, int32_t ib){
	return 1/sqrt(3)*(ia+2*ib);
}


int32_t inverse_park(int32_t vq_set, int32_t vd_set, bool flag){
	float pos=Get_Mech_Pos();
	if(flag)// flag is v_alpha
		return vd_set*cos(pos)-vq_set*sin(pos);
	else
		return vd_set*sin(pos)+vq_set*cos(pos);
}

int32_t inverse_clarke(int32_t v_alpha, int32_t v_beta,bool flag){
	if(flag) //flag is v_b
		return -0.5*v_alpha+sqrt(3)/2*v_beta;
	else
		return -0.5*v_alpha-sqrt(3)/2*v_beta;

}

float Get_Mech_Pos(){
	uint32_t raw=Encoder_Read();
   	float unsigned_mech = (6.28318530718f/(float)_CPR) * (float) (raw);//%_CPR);
    return (float) unsigned_mech;// + 6.28318530718f* (float) rotations;
}
/*
float Get_Mech_Pos() {                            //returns rotor angle in radians.
    int raw = _CPR-Encoder_Read();
    if ((last_mech-raw)>3500) {rotations++;}
    if ((last_mech-raw)<-3500) {rotations--;}
    float unsigned_mech = (6.28318530718f/(float)_CPR)* (float) ((raw)  );//%_CPR);
    last_mech = raw;
    return (float) unsigned_mech + 6.28318530718f* (float) rotations;
}
*/


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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
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

