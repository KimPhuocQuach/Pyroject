#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_tim.h"
//#include "stm32f4xx_hal_tim_ex.h"
//#include "stm32f4xx.h"
#include "stdio.h"
#include "i2c-lcd.h"

#define DHT11_PORT GPIOA						//Pin read/Write Data from DHT11 Sensor
#define DHT11_PIN GPIO_PIN_1

static uint8_t rh_byte1, rh_byte2, temp_byte1, temp_byte2, i;
static uint16_t sum;
static char buffer[20];

void initTimerDelay(void);							//Importance to add in main() (Note: before while(1))
void readSensor(void);									//Add in while(1) of main()
static void MX_TIM6_Init(void);			//Dont care
void delay (uint16_t time);					//Delay in microsec.
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DHT11_Read (void);					//Read 1 byte (in 5 bytes transmission)
uint8_t DHT11_Check_Response (void);//Check DHT connection and data valid???
