#include "main.h"
#include "BH1750.h"
#include <stdio.h>
#include "i2c-2004.h"
#include "DHT_22.h"
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);

typedef struct {
	//err = false: can not get data, err = true: save into temp and rh
	bool dht22_err;		//whether dht22 meets error
	float temp;				//DHT22 - temp
	float rh;					//DHT22 - humidity
	uint16_t lx;			//BH1750 - lumen
	//Mode = 0 (Auto) - 1 (Manual); State = 0 (OFF) - 1 (ON)
	uint8_t mode;			//Relay auto/manual mode
	uint8_t state;		//Relay on/off
	bool scan_now;		//LCD update now or not
	uint8_t sleep;		//true: sleep in ... mins after ON, false: relay can be on
	uint8_t soil;			//soil moisture measurement
	bool relay_gui_on;//1 = on, 0 = off: 
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t h_start;
	uint8_t m_start;
//	uint8_t s_start;
//	uint8_t h_stop;
//	uint8_t m_stop;
//	uint8_t s_stop;
	//uint32_t checksum;
} controller;

char buffer[10];	//For lcd_show
uint32_t adc;			//Raw ADC_value from Sensor
uint8_t send[33] = {0x00}, receive[16];		//GUI <-> STM32
//DS1307
#define DS3231_ADD 0x68
uint8_t raw[3], receive_data[7],send_data[7];
uint8_t BCD2DEC(uint8_t data);
uint8_t DEC2BCD(uint8_t data);
void ds1307_cal_1(controller* ht);

uint8_t read_dht22(float* t, float* rh);
uint16_t read_bh1750(BH1750_device_t* test_dev);
void lcd_show(controller* c);
void relay_on(controller* ht);
void relay_off(controller* ht);
uint8_t cal_mw(uint32_t* adc);
void send_uart(controller* ht);
void receive_uart_1(controller* ht);
void receive_uart_2(controller* ht);
void receive_uart_3(controller* ht);

int main(void){
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
	//Programming Codes
	//clear buffer receive[16];
	for(uint8_t i = 0; i<16; i++)
		receive[i] = 0x00;
	HAL_ADC_Start_DMA(&hadc1, &adc, 1);
	HAL_UART_Receive_DMA(&huart3, receive, 16);
	
	lcd_init();
  lcd_send_cmd (0x80|0x00);
  lcd_send_string("T= ");
  lcd_send_cmd (0x80|0x0A);
  lcd_send_string("RH= ");
	lcd_send_cmd (0x80|0x40);
  lcd_send_string("I= ");
	lcd_send_cmd (0x80|0x4B);
  lcd_send_string("Mw= ");		//Soil Humidity Display
	
	lcd_send_cmd (0x80|0x14);
  lcd_send_string("Relay: ");
	lcd_send_cmd (0x80|0x1E);
  lcd_send_string(" - ");
	lcd_send_cmd (0x80|0x54);
  lcd_send_string("RTC time:   /  /  ");
	BH1750_device_t* test_dev = BH1750_init_dev_struct(&hi2c1, "test device", true);
  BH1750_init_dev(test_dev);
	init_DHT22();
	controller* ht = (controller*)calloc(1, sizeof(controller));
	ht->state = 0; ht->mode = 0; ht->sleep = 0; ht->relay_gui_on = false;

  while (1)
  {
		receive_uart_1(ht);
		receive_uart_2(ht);
		receive_uart_3(ht);
		
    //Sensors Reading
		ht->dht22_err = read_dht22(&(ht->temp),&(ht->rh));
		ht->lx = read_bh1750(test_dev);
		ht->soil = cal_mw(&adc);
		relay_on(ht);
		relay_off(ht);
		ds1307_cal_1(ht);
		send_uart(ht);		//8 bits data + 1 bit EVEN parity
		lcd_show(ht);
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;//2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 168-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xFFFF-1;
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
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;//NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void receive_uart_3(controller* ht){
	if(receive[0] == 'R' && receive[1] == 'S' && receive[2] == 'S' && receive[3] == 'T')
	{
		ht->h_start = (receive[5] - 0x30)*10 + receive[6] - 0x30;
		ht->m_start = (receive[9] - 0x30)*10 + receive[10] - 0x30;
		//ht->s_start = (receive[13] - 0x30)*10 + receive[14] - 0x30;
		//clear buffer receive[16];
		for(uint8_t i = 0; i<16; i++)
			receive[i] = 0x00;
	}
	if(ht->hour == ht->h_start && ht->min == ht->m_start)
			ht->relay_gui_on = true;
}

void receive_uart_2(controller* ht){
	if(receive[0] == 'R' && receive[1] == 'S' && receive[2] == 'S' && receive[3] == 'S')
	{
		ht->hour = (receive[5] - 0x30)*10 + receive[6] - 0x30;
		ht->min = (receive[9] - 0x30)*10 + receive[10] - 0x30;
		ht->sec = (receive[13] - 0x30)*10 + receive[14] - 0x30;
		
		send_data[0]=DEC2BCD(ht->sec);
		send_data[1]=DEC2BCD(ht->min);
		send_data[2]=DEC2BCD(ht->hour);
		
		HAL_I2C_Mem_Write_IT(&hi2c1,DS3231_ADD<<1,0,I2C_MEMADD_SIZE_8BIT,send_data,3);//7);
		//clear buffer receive[16];
		for(uint8_t i = 0; i<16; i++)
			receive[i] = 0x00;
	}
}
void ds1307_cal_1(controller* ht){
	static uint32_t last_4 = 0;
	if((uint32_t)(HAL_GetTick() - last_4) > 1000)
	{
		HAL_I2C_Mem_Read_IT(&hi2c1,DS3231_ADD<<1,0,I2C_MEMADD_SIZE_8BIT,receive_data,7);
		ht->sec=BCD2DEC(receive_data[0]);
		ht->min=BCD2DEC(receive_data[1]);
		ht->hour=BCD2DEC(receive_data[2]);
		//ht->scan_now = true;
		last_4 = HAL_GetTick();
	}
}

uint8_t BCD2DEC(uint8_t data){
	return (data>>4)*10 + (data&0x0f);
}

uint8_t DEC2BCD(uint8_t data){
	return (data/10)<<4|(data%10);
}

void send_uart(controller* ht){
	static uint32_t last_3 = 0;
	if((uint32_t)(HAL_GetTick() - last_3) > 1000)
	{
		send[0] = ((uint8_t)ht->temp / 100) % 10 + 0x30;
		send[1] = ((uint8_t)ht->temp / 10) % 10 + 0x30;
		send[2] = (uint8_t)ht->temp % 10 + 0x30;
		send[3] = 0x2C;				// ,
		send[4] = ((uint8_t)ht->rh / 100) % 10 + 0x30;
		send[5] = ((uint8_t)ht->rh / 10) % 10 + 0x30;
		send[6] = (uint8_t)ht->rh % 10 + 0x30;
		
		send[7] = 0x2C;				// ,
		send[8] = ((uint8_t)ht->lx / 100000) % 10 + 0x30;
		send[9] = ((uint8_t)ht->lx / 10000) % 10 + 0x30;
		send[10] = ((uint8_t)ht->lx / 1000) % 10 + 0x30;
		send[11] = ((uint8_t)ht->lx / 100) % 10 + 0x30;
		send[12] = ((uint8_t)ht->lx / 10) % 10 + 0x30;
		send[13] = (uint8_t)ht->lx % 10 + 0x30;
		
		send[14] = 0x2C;				// ,
		send[15] = ((uint8_t)ht->soil / 100) % 10 + 0x30;
		send[16] = ((uint8_t)ht->soil / 10) % 10 + 0x30;
		send[17] = (uint8_t)ht->soil % 10 + 0x30;
		
		send[18] = 0x2C;				// ,
		send[19] = ht->state + 0x30;
		send[20] = 0x2C;				// ,
		send[21] = ht->mode + 0x30;
		
		send[22] = 0x2C;				// ,
		send[23] = ((uint8_t)ht->hour / 10) % 10 + 0x30;
		send[24] = (uint8_t)ht->hour % 10 + 0x30;
		
		send[25] = 0x2C;				// ,
		send[26] = ((uint8_t)ht->min / 10) % 10 + 0x30;
		send[27] = (uint8_t)ht->min % 10 + 0x30;
		
		send[28] = 0x2C;				// ,
		send[29] = ((uint8_t)ht->sec / 10) % 10 + 0x30;
		send[30] = (uint8_t)ht->sec % 10 + 0x30;
			
		send[31] = 0x0D;				// /r
		send[32] = 0x0A;				// /n
		
		HAL_UART_Transmit_IT(&huart3, send, 33);
		last_3 = HAL_GetTick();
	}
}

void receive_uart_1(controller* ht){
	if(receive[0] == 'R' && receive[1] == 'S' && receive[2] == '1' && receive[3] == 'O')
	{
		ht->relay_gui_on = true;
		//clear buffer receive[16];
		for(uint8_t i = 0; i<16; i++)
			receive[i] = 0x00;
	}
	//else ht->relay_gui_on = false;
}

uint8_t cal_mw(uint32_t* adc){
	return (uint8_t)((4096.0 - (float)(*adc))/4096.0*100.0);
}

void relay_off(controller* ht){
	//Turn OFF the relay after 1 mins. (Back to Auto or already Auto)
	static uint32_t last_2 = 0;
	if((uint32_t)(HAL_GetTick() - last_2) > 10000){
		if(ht->sleep){
			ht->sleep = 0;
		}
		if(ht->state){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			ht->mode = 0; ht->state = 0;
			ht->scan_now = true;
			ht->sleep = 1;					//Turn ON sleep-mode flat
		}
		last_2 = HAL_GetTick();
	}
}

void relay_on(controller* ht){
	//Button triggering and Relay
	if((!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) || ht->relay_gui_on) && !(ht->sleep)){
	//Button PRESS (Auto -> Manual)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		ht->mode = 1; ht->state = 1;
		ht->scan_now = true;//Refresh LCD right now
		
		ht->relay_gui_on = false;
	}else{
	//Sensor Conditioning (Still Auto)
		if((ht->temp > 40 || ht->rh < 40 || ht->lx > 500 || (ht->soil < 40 && ht->soil > 10)) && ht->rh > 0 && !(ht->sleep)){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			ht->state = 1;//ht->scan_now = true;
		}
	}
}

void lcd_show(controller* c){
	static uint32_t lastScan = 0;
  if ((HAL_GetTick() - lastScan > 1000) || c->scan_now){
		lcd_send_cmd(0x80|0x03);
		lcd_send_string("       ");
		lcd_send_cmd(0x80|0x0D);
		lcd_send_string("       ");
		if(c->dht22_err){
			//Temp. Display
			sprintf(buffer, "%0.1f", c->temp);
			lcd_send_cmd(0x80|0x03);
			lcd_send_string(buffer);
			lcd_send_string("*C");
			//Humidity Display
			sprintf(buffer, "%0.1f", c->rh);
			lcd_send_cmd(0x80|0x0D);
			lcd_send_string(buffer);
			lcd_send_string("%");
		}else{//When DHT22 meets ERROR
			lcd_send_cmd(0x80|0x03);
			lcd_send_string("ERROR");
			lcd_send_cmd(0x80|0x0D);
			lcd_send_string("ERROR");
		}
		//Display Luminosity
		lcd_send_cmd (0x80|0x43);
		lcd_send_string("        ");
		lcd_send_cmd (0x80|0x43);
		sprintf(buffer, "%d", c->lx);
		lcd_send_string(buffer);
		lcd_send_string("Lx");
		//Display Soil Mw
		lcd_send_cmd (0x80|0x4F);
		lcd_send_string("    ");
		lcd_send_cmd (0x80|0x4F);
		sprintf(buffer, "%d", c->soil);
		lcd_send_string(buffer);
		lcd_send_string("%");
		//Display Relay States
		if(!(c->state)){
			lcd_send_cmd (0x80|0x1B);
			lcd_send_string("OFF");
		}else{
			lcd_send_cmd (0x80|0x1B);
			lcd_send_string("ON ");
		}
		//Relay in AUTO//MANUAL MODE
		if(!(c->mode)){
			lcd_send_cmd (0x80|0x21);
			lcd_send_string("AUTO  ");
		}else{
			lcd_send_cmd (0x80|0x21);
			lcd_send_string("MANUAL");
		}
		//Time from RTC
		lcd_send_cmd (0x80|0x5E);
		lcd_send_string("  /  /    ");
		lcd_send_cmd (0x80|0x5E);
		sprintf(buffer, "%d", c->hour);
		lcd_send_string(buffer);
		lcd_send_cmd (0x80|0x61);
		sprintf(buffer, "%d", c->min);
		lcd_send_string(buffer);
		lcd_send_cmd (0x80|0x64);
		sprintf(buffer, "%d", c->sec);
		lcd_send_string(buffer);
		
		//RESET to Refreshing Freq. = 1Hz
		c->scan_now = false;
		lastScan = HAL_GetTick();
	}
}

bool read_dht22(float* t, float* rh){
	uint8_t d[5]; 
	bool ret;
	uint16_t cal;
	static uint32_t last_1 = 0;
	//Max Sampling Freq. of DHT22 = 2 sec.
	if((uint32_t)(HAL_GetTick() - last_1) > 2000){
		if(DHT22_Check_Response(d)){
			//Temp. Calculation
			cal = d[2]&0x7F; cal*=256;
			cal += d[3]; *t = cal*0.1;
			if(d[2] & 0x80)	*t*=-1;
			//Humidity Calculation
			cal = d[0]; cal*=256;
			cal += d[1]; *rh = cal*0.1;
			//Read OK
			ret = true;
		}
		else {
			ret = false;//Read not OK -> ERROR
		}
		last_1 = HAL_GetTick();
	}
	return ret;
}

uint16_t read_bh1750(BH1750_device_t* test_dev){
	static uint32_t last = 0;
	if((uint32_t)(HAL_GetTick() - last) > 200)//200ms
	{
		test_dev->poll(test_dev);//Read and compute luminosity
		last = HAL_GetTick();
	}
	return test_dev->value;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
