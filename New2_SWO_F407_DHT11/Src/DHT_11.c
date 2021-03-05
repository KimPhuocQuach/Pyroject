#include "DHT_11.h"
TIM_HandleTypeDef htim6;

void initTimerDelay(void)															//Must call in main() above while(1)
{
  MX_TIM6_Init();
	HAL_TIM_Base_Start(&htim6);

	if (SysTick_Config(SystemCoreClock / 1000))			//1ms
  {
    /* Capture error */
    while (1);
  }
	printf("...Initializing...\n");
}

void readSensor(void)																	//Must call in while(1) of main()
{
	static uint32_t lastRead = 0;
	uint32_t now = HAL_GetTick();
	if((uint32_t)(now - lastRead) > 1000)
	{
		if(DHT11_Check_Response())
			printf("Temp = %d, humid = %d\n", temp_byte1, rh_byte1);
		else printf("ERROR\n");
		lastRead = HAL_GetTick();
	}
}

void delay (uint16_t time)							//Code 1: Timer 6
{
	//change your code here for the delay in microseconds 
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim6))<time);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH ;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
		delay (30);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
		{
			while (HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN));
			i |= (1<<(7-j));
		}
		else i &= ~(1<<(7-j));
	}
	return i;
}

uint8_t DHT11_Check_Response (void)
{
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
	HAL_Delay(20);			
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);
	delay(40);

	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
		rh_byte1 = DHT11_Read ();
		rh_byte2 = DHT11_Read ();
		temp_byte1 = DHT11_Read ();
		temp_byte2 = DHT11_Read ();
		sum = DHT11_Read();
		Set_Pin_Output(DHT11_PORT, DHT11_PIN);
		HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
		if(sum == (rh_byte1+rh_byte2+temp_byte1+temp_byte2))
			return 1;
		else return 0;
	}
	else{
		Set_Pin_Output(DHT11_PORT, DHT11_PIN);
		HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
		return 0;
	}
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 167;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
