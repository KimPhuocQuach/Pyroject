#include "main.h"
#include "BH1750.h"
#include <stdio.h>
#include "i2c-2004.h"
#include "DHT_22.h"
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);

typedef struct {
	//err = false: can not get data, err = true: save into temp and rh
	bool dht22_err;		
	float temp;
	float rh;
	uint16_t lx;
	//Mode = 0 (Auto) - 1 (Manual); State = 0 (OFF) - 1 (ON)
	uint8_t mode;
	uint8_t state;
	bool scan_now;
	uint8_t sleep;	//true: sleep in ... mins after ON, false: relay can be on
} controller;

char buffer[10];	//Cho lcdshow
uint8_t read_dht22(float* t, float* rh);
uint16_t read_bh1750(BH1750_device_t* test_dev);
void lcd_show(controller* c);
void relay_on(controller* ht);
void relay_off(controller* ht);
void relay_sleep(controller* ht);
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
  lcd_send_cmd (0x80|0x00);
  lcd_send_string("T= ");
  lcd_send_cmd (0x80|0x0A);
  lcd_send_string("RH= ");
	lcd_send_cmd (0x80|0x40);
  lcd_send_string("I= ");
	lcd_send_cmd (0x80|0x14);
  lcd_send_string("Relay: ");
	lcd_send_cmd (0x80|0x1E);
  lcd_send_string(" - ");
	lcd_send_cmd (0x80|0x54);
  lcd_send_string("RTC time: 24/24/24");
	BH1750_device_t* test_dev = BH1750_init_dev_struct(&hi2c1, "test device", true);
  BH1750_init_dev(test_dev);
	init_DHT22();
	controller* ht = (controller*)calloc(1, sizeof(controller));
	ht->state = 0; ht->mode = 0; ht->sleep = 0;
  while (1)
  {
		//Sensors Reading
		ht->dht22_err = read_dht22(&(ht->temp),&(ht->rh));
		ht->lx = read_bh1750(test_dev);

		relay_on(ht);
		relay_off(ht);
		relay_sleep(ht);
		
		//LCD_2004 Displaying
		lcd_show(ht);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;//NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void relay_sleep(controller* ht){
	//Relay Sleep Mode
	static uint32_t last = 0;
	if((uint32_t)(HAL_GetTick() - last) > 20000){
		if(ht->sleep){						//The time has to be larger than relay_off
			ht->sleep = 0;
		}
		last = HAL_GetTick();
	}
}

void relay_off(controller* ht){
	//Turn OFF the relay after 1 mins. (Back to Auto or already Auto)
	static uint32_t last1 = 0;
	if((uint32_t)(HAL_GetTick() - last1) > 10000){
		if(ht->state){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			ht->mode = 0; ht->state = 0;
			ht->scan_now = true;
			ht->sleep = 1;					//Turn ON sleep-mode flat
		}
		last1 = HAL_GetTick();
	}
}

void relay_on(controller* ht){
	//Button triggering and Relay
	if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) && !(ht->sleep)){
	//Button PRESS (Auto -> Manual)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		ht->mode = 1; ht->state = 1;
		ht->scan_now = true;//Refresh LCD right now
	}else{
	//Sensor Conditioning (Still Auto)
		if((ht->temp > 40 || ht->rh < 40 || ht->lx > 200) && ht->rh > 0 && !(ht->sleep)){
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
		//RESET to Refreshing Freq. = 1Hz
		c->scan_now = false;
		lastScan = HAL_GetTick();
	}
}

bool read_dht22(float* t, float* rh){
	uint8_t d[5]; 
	bool ret;
	uint16_t cal;
	static uint32_t lastRead_1 = 0;
	//Max Sampling Freq. of DHT22 = 2 sec.
	if((uint32_t)(HAL_GetTick() - lastRead_1) > 2000){
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
		lastRead_1 = HAL_GetTick();
	}
	return ret;
}

uint16_t read_bh1750(BH1750_device_t* test_dev){
	static uint32_t lastRead_2 = 0;
	if((uint32_t)(HAL_GetTick() - lastRead_2) > 200)//200ms
	{
		test_dev->poll(test_dev);//Read and compute luminosity
		lastRead_2 = HAL_GetTick();
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
