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
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_FREERTOS_Init(void);

osThreadId hscheduler,himu,hmotor,hled,hservo;
osSemaphoreId sema_sched_id;

/* Private function prototypes -----------------------------------------------*/
static void scheduler_thread(void const *argument);
static void led_thread(void const *argument);
static void servo_thread(void const *argument);
static void motor_thread(void const *argument);
void motor_rotate_forward(char sens, uint8_t duty_motor);
void motor_rotate_backward(char sens, uint8_t duty_motor);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/********************Defines***************/
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

/******************Global Variables**************/
uint8_t anv=0x30;
unsigned char state = 0;  
int recev; 
                


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
  MX_USART3_UART_Init();
	
	osThreadDef(ledt,led_thread,osPriorityRealtime,0,configMINIMAL_STACK_SIZE);
	osThreadDef(servot,servo_thread,osPriorityRealtime,0,configMINIMAL_STACK_SIZE);
	osThreadDef(motor,motor_thread,osPriorityHigh,0,configMINIMAL_STACK_SIZE);
	hled=osThreadCreate(osThread(ledt),NULL);
	hservo=osThreadCreate(osThread(servot),NULL);
	hmotor=osThreadCreate(osThread(motor),NULL);

	
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	//HAL_UART_Transmit_DMA(&huart3,&anv,sizeof(anv));
	HAL_GPIO_WritePin(DcCal_gpio,DcCal_pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EnGate_gpio,EnGate_pin,GPIO_PIN_SET);
	recev=20;
	/* Start scheduler */
  osKernelStart();
  
	/* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1){};
}

/** System Clock Configuration
*/

static void servo_thread(void const *argument)
{
	while(1)
	{
		servo_d1=201;
		servo_d2=201;
		osDelay(1000);
		servo_d1=100;
		servo_d2=100;
		osDelay(1000);
		anv++;
	}
}

static void motor_thread(void const *argument)
{
	int a=0;
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
		if (a%100000==0)		HAL_GPIO_TogglePin(Led_gpio,Led_pin2);
		a++; 
			
  }
}

static void led_thread(void const *argument)
{
	while(1)
	{
		HAL_GPIO_TogglePin(Led_gpio,Led_pin);
		osDelay(100);
	}
}


void GPIO_pwm(uint16_t GPIO_Pin, GPIO_PinState PinState, uint8_t duty)
{
  /* calculation of duty cycle relative to period */
  uint8_t duty_cycle_motor = 0;
  duty_cycle_motor = (duty * pwm_period)/127; 
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
