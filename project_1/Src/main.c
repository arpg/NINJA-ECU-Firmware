/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"
#include "gpio.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_FREERTOS_Init(void);

osThreadId hscheduler,himu,hmotor,hled,hservo,husart,hcopy;
osSemaphoreId sema_sched_id;

/* Private function prototypes -----------------------------------------------*/
static void led_thread(void const *argument);
static void servo_thread(void const *argument);
static void motor_thread(void const *argument);
static void usart_thread(void const *argument);
void motor_rotate_forward(char sens, uint8_t duty_motor);
void motor_rotate_backward(char sens, uint8_t duty_motor);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void restart1();
int allign(uint8_t buff[26]);
static void copy_thread(void const *argument);

/********************Defines***************/

struct ComRecDataPack{
	char delimiter[4];
	float steering_angle;
	float motor_power_percent;
	unsigned int dev_time;
	unsigned int chksum;
}rec_pack,pack1,pack2;

struct ComTrDataPack{
	char delimiter[4];
	float enc0;
	float enc1;
	float enc2;
	float enc3;
	float steer_ang;
	float swing_ang0;
	float swing_ang1;
	float swing_ang2;
	float swing_ang3;
	float motor_current;
	unsigned int dev_time;
	unsigned int chksum;
}tra_pack;
#define pwm_period 400

#define servo_d1 TIM1->CCR1
#define servo_d2 TIM1->CCR3

#define Led_gpio      GPIOE
#define Led_pin       GPIO_PIN_2
#define Led_pin2       GPIO_PIN_3

#define ph_u_l        GPIO_PIN_14         //N1
#define ph_v_l        GPIO_PIN_7         //N3
#define ph_w_l        GPIO_PIN_6          //N2

#define ph_u_h        GPIO_PIN_0				//P1
#define ph_v_h        GPIO_PIN_1				//P3
#define ph_w_h        GPIO_PIN_14				//P2

#define sens_gpio     GPIOD
#define sens_ph1_pin  GPIO_PIN_11
#define sens_ph2_pin  GPIO_PIN_10
#define sens_ph3_pin  GPIO_PIN_9

/* Motor controller dedicated pins */
#define DcCal_gpio    GPIOD
#define DcCal_pin     GPIO_PIN_3
#define EnGate_gpio   GPIOB
#define EnGate_pin    GPIO_PIN_8

#define pulse_1 TIM4->CCR2       //n2                           // compare registers of channels for motor pwm
#define pulse_2 TIM4->CCR1       //n3
#define pulse_3 TIM4->CCR3       //n1

//#define packet_size 22
/******************Global Variables**************/
uint8_t anv=0x30;
uint8_t mcpy=0;
int a=0,b,packet_size;
unsigned char state = 0;  
int recev; 
uint8_t tx_buff[4] = {'a','b','c','\n'};     
uint8_t mx_buff[26],mx_buff1[26]; 
uint8_t buff11[22]; 
uint32_t enc1[8],enc2[8];
#define enco1 TIM3->CNT 
#define enco2 TIM2->CNT 
#define enco3 TIM8->CNT 
#define enco4 TIM5->CNT 

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
	MX_TIM8_Init();
	MX_DMA_Init();
  MX_USART3_UART_Init();
	
	osThreadDef(copyt,copy_thread,osPriorityRealtime,0,configMINIMAL_STACK_SIZE);
	osThreadDef(ledt,led_thread,osPriorityRealtime,0,configMINIMAL_STACK_SIZE);
	osThreadDef(servot,servo_thread,osPriorityRealtime,0,configMINIMAL_STACK_SIZE);
	osThreadDef(motor,motor_thread,osPriorityHigh,0,configMINIMAL_STACK_SIZE);
	osThreadDef(usart,usart_thread,osPriorityHigh,0,configMINIMAL_STACK_SIZE);
	hled=osThreadCreate(osThread(ledt),NULL);
	//hservo=osThreadCreate(osThread(servot),NULL);
	hmotor=osThreadCreate(osThread(motor),NULL);
	husart=osThreadCreate(osThread(usart),NULL);
	hcopy=osThreadCreate(osThread(copyt),NULL);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_ADC_Start_DMA(&hadc1
	osSemaphoreDef(sem1);
	sema_sched_id=osSemaphoreCreate(osSemaphore(sem1),0);
	packet_size=sizeof(rec_pack);
	b=packet_size;
	HAL_UART_Transmit_DMA(&huart3,&tx_buff[0],sizeof(tx_buff));
		HAL_UART_Receive_DMA(&huart3,&mx_buff[0],packet_size);
	//HAL_UART_Receive_DMA(&huart3,&rx_buff[0],sizeof(rx_buff));
	HAL_GPIO_WritePin(DcCal_gpio,DcCal_pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EnGate_gpio,EnGate_pin,GPIO_PIN_SET);
	HAL_TIM_Encoder_Start_DMA(&htim3,TIM_CHANNEL_ALL,enc1,enc2,8);
	HAL_TIM_Encoder_Start_DMA(&htim2,TIM_CHANNEL_ALL,enc1,enc2,8);
	HAL_TIM_Encoder_Start_DMA(&htim8,TIM_CHANNEL_ALL,enc1,enc2,8);
	HAL_TIM_Encoder_Start_DMA(&htim5,TIM_CHANNEL_ALL,enc1,enc2,8);
	
//	HAL_DMAEx_MultiBufferStart(huart3.hdmarx, (uint32_t)huart3.pRxBuffPtr,buff11[0],buff12[0],sizeof(buff11));

	
	servo_d1=75;
	servo_d2=75;
	recev=0;
	/* Start scheduler */
  osKernelStart();
  
	/* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1);
}

/** System Clock Configuration
*/
static void copy_thread(void const *argument) {
	int c;
	while(1)
	{
		osSemaphoreWait(sema_sched_id,osWaitForever);
		c=b;
		if (c!=packet_size)
		{
			b=packet_size;
		}
		else 
		{
			memcpy(&rec_pack,&mx_buff,sizeof(rec_pack));
			b=allign((uint8_t *)(&rec_pack));
		}
		if (b!=packet_size && b!=-1)
		{
			HAL_UART_Receive_DMA(&huart3,&buff11[0],b);
		}
		else 
		HAL_UART_Receive_DMA(&huart3,&mx_buff[0],sizeof(rec_pack));
		
	}	
}

void restart1()
{
		mcpy=1;
}

int allign(uint8_t buff[26])
{
		if (buff[0]==0xAA && buff[1]==0x55 && buff[2]==0xE1 && buff[3]==0x1E)	
		{	
			return(packet_size);
		}
		else
		{
			for (int i=1;i<packet_size;i++)
				{
					if (buff[i]==0xAA)
					{
						if (buff[(i+1)%packet_size]==0x55 && buff[(i+2)%packet_size]==0xE1 && buff[(i+3)%packet_size]==0x1E)
						{
							return(i);
						}
					}
				}
		}		
		return (-1);
}
static void usart_thread(void const *argument) {
	char delimiter;
	while(1){
		delimiter = rec_pack.delimiter[0];
	}
}

static void servo_thread(void const *argument)
{
	while(1)
	{
		servo_d1=35;
		servo_d2=35;
		osDelay(1000);
		servo_d1=120;
		servo_d2=120;
		osDelay(1000);
		anv++;
	}
}

static void motor_thread(void const *argument)
{
	while (1)
  { 
    /* get the rotor state of the bldc */
    state = HAL_GPIO_ReadPin(sens_gpio,sens_ph1_pin);
    state = state << 1;
    state |= HAL_GPIO_ReadPin(sens_gpio,sens_ph2_pin);
    state = state << 1;
    state |= HAL_GPIO_ReadPin(sens_gpio,sens_ph3_pin);
    if(recev >= 0)
    {
			
      motor_rotate_forward(state, abs(recev));   
    }
    else
    {
      motor_rotate_backward(state, abs(recev));
    }
		if (mcpy==1)
		{
			osSemaphoreRelease(sema_sched_id);
			mcpy=0;	
		}
  }
}

static void led_thread(void const *argument)
{
	while(1)
	{
		HAL_GPIO_TogglePin(Led_gpio,Led_pin);
		osDelay(50);
	}
}


void GPIO_pwm(uint16_t GPIO_Pin, GPIO_PinState PinState, uint8_t duty)
{
  /* calculation of duty cycle relative to period */
  uint8_t duty_cycle_motor = 0;
  duty_cycle_motor = (duty * pwm_period)/100; 
  /* if pin state is set apply the duty cycle value to the compare register of the particular pin */
  if(PinState == GPIO_PIN_SET)
  {
    switch(GPIO_Pin)
     {
        case(ph_u_l):
          pulse_3 = duty_cycle_motor;
        break;

        case(ph_v_l):
          pulse_2 = duty_cycle_motor;
        break;

        case(ph_w_l):
          pulse_1 = duty_cycle_motor;
        break;

        default:
        break;
    }
  } 
  /* if pin state is reset then reset the pin */
  else 
  {
    switch(GPIO_Pin)
     {
        case(ph_u_l):
         pulse_3 = 0;
        break;

        case(ph_v_l):
          pulse_2 = 0;
        break;

        case(ph_w_l):
          pulse_1 = 0;
        break;

        default:
        break;
      }
  }
}
/* This function returns NULL and is mostly responsible for driving the BLDC motor in forward direction
   The low side of the MOSFETs are responsible for providing PWMs while the high side of the MOSFETs are 
    hard coded. The MOSFETs are arranged in a hex converter configuration. There are 6 rotor states.*/
void motor_rotate_forward(char sens, uint8_t duty_motor)
{
    switch(sens) {
      case 5:   
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_SET, duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
        break;
      case 4:   
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
        GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
        break;
      case 6:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
        GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
        break;
      case 2:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);   
        GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
        break;
      case 3:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_SET);
        GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
        break;
      case 1: 
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);   
        GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
        HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_SET);
        break;
      default:
        while(1);
      }
}

/* This function returns NULL and is mostly responsible for driving the BLDC motor in reverse direction
   The low side of the MOSFETs are responsible for providing PWMs while the high side of the MOSFETs are 
    hard coded. The MOSFETs are arranged in a hex converter configuration. There are 6 rotor states.*/
void motor_rotate_backward(char sens, uint8_t duty_motor) {
  switch(sens) {
    case 4:   
      HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
      
      break;
    case 6:   
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_SET);
      GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);      
      break;
    case 2:   
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
      HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_SET);      
      break;
    case 3:   
      HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
      GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
      break;
    case 1:   
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
      break;
    case 5: 
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
      GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
      break;
    default:
      while(1);
    }
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
