/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define GPIO_Output_Pin GPIO_PIN_2
#define GPIO_Output_GPIO_Port GPIOE
#define GPIO_OutputE3_Pin GPIO_PIN_3
#define GPIO_OutputE3_GPIO_Port GPIOE
#define ADC1_IN10_Pin GPIO_PIN_0
#define ADC1_IN10_GPIO_Port GPIOC
#define ADC1_IN11_Pin GPIO_PIN_1
#define ADC1_IN11_GPIO_Port GPIOC
#define ADC1_IN12_Pin GPIO_PIN_2
#define ADC1_IN12_GPIO_Port GPIOC
#define ADC1_IN13_Pin GPIO_PIN_3
#define ADC1_IN13_GPIO_Port GPIOC
#define TIM5_CH1_Pin GPIO_PIN_0
#define TIM5_CH1_GPIO_Port GPIOA
#define TIM5_CH2_Pin GPIO_PIN_1
#define TIM5_CH2_GPIO_Port GPIOA
#define ADC1_IN4_Pin GPIO_PIN_4
#define ADC1_IN4_GPIO_Port GPIOA
#define ADC1_IN5_Pin GPIO_PIN_5
#define ADC1_IN5_GPIO_Port GPIOA
#define TIM3_CH1_Pin GPIO_PIN_6
#define TIM3_CH1_GPIO_Port GPIOA
#define TIM3_CH2_Pin GPIO_PIN_7
#define TIM3_CH2_GPIO_Port GPIOA
#define ADC1_IN14_Pin GPIO_PIN_4
#define ADC1_IN14_GPIO_Port GPIOC
#define ADC1_IN15_Pin GPIO_PIN_5
#define ADC1_IN15_GPIO_Port GPIOC
#define GPIO_OutputB0_Pin GPIO_PIN_0
#define GPIO_OutputB0_GPIO_Port GPIOB
#define GPIO_OutputB1_Pin GPIO_PIN_1
#define GPIO_OutputB1_GPIO_Port GPIOB
#define GPIO_OutputB2_Pin GPIO_PIN_2
#define GPIO_OutputB2_GPIO_Port GPIOB
#define TIM1_CH1_Pin GPIO_PIN_9
#define TIM1_CH1_GPIO_Port GPIOE
#define TIM1_CH2_Pin GPIO_PIN_11
#define TIM1_CH2_GPIO_Port GPIOE
#define TIM1_CH3_Pin GPIO_PIN_13
#define TIM1_CH3_GPIO_Port GPIOE
#define GPIO_OutputB12_Pin GPIO_PIN_12
#define GPIO_OutputB12_GPIO_Port GPIOB
#define GPIO_Input_Pin GPIO_PIN_8
#define GPIO_Input_GPIO_Port GPIOD
#define GPIO_EXTI9_Pin GPIO_PIN_9
#define GPIO_EXTI9_GPIO_Port GPIOD
#define GPIO_EXTI9_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_EXTI10_Pin GPIO_PIN_10
#define GPIO_EXTI10_GPIO_Port GPIOD
#define GPIO_EXTI10_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_EXTI11_Pin GPIO_PIN_11
#define GPIO_EXTI11_GPIO_Port GPIOD
#define GPIO_EXTI11_EXTI_IRQn EXTI15_10_IRQn
#define TIM4_CH3_Pin GPIO_PIN_14
#define TIM4_CH3_GPIO_Port GPIOD
#define GPIO_InputD15_Pin GPIO_PIN_15
#define GPIO_InputD15_GPIO_Port GPIOD
#define TIM8_CH1_Pin GPIO_PIN_6
#define TIM8_CH1_GPIO_Port GPIOC
#define TIM8_CH2_Pin GPIO_PIN_7
#define TIM8_CH2_GPIO_Port GPIOC
#define TIM2_CH1_Pin GPIO_PIN_15
#define TIM2_CH1_GPIO_Port GPIOA
#define GPIO_OutputD0_Pin GPIO_PIN_0
#define GPIO_OutputD0_GPIO_Port GPIOD
#define GPIO_OutputD1_Pin GPIO_PIN_1
#define GPIO_OutputD1_GPIO_Port GPIOD
#define GPIO_OutputD2_Pin GPIO_PIN_2
#define GPIO_OutputD2_GPIO_Port GPIOD
#define GPIO_OutputD3_Pin GPIO_PIN_3
#define GPIO_OutputD3_GPIO_Port GPIOD
#define GPIO_OutputD4_Pin GPIO_PIN_4
#define GPIO_OutputD4_GPIO_Port GPIOD
#define GPIO_OutputD5_Pin GPIO_PIN_5
#define GPIO_OutputD5_GPIO_Port GPIOD
#define TIM2_CH2_Pin GPIO_PIN_3
#define TIM2_CH2_GPIO_Port GPIOB
#define TIM4_CH3B6_Pin GPIO_PIN_6
#define TIM4_CH3B6_GPIO_Port GPIOB
#define TIM4_CH3B7_Pin GPIO_PIN_7
#define TIM4_CH3B7_GPIO_Port GPIOB
#define GPIO_OutputB8_Pin GPIO_PIN_8
#define GPIO_OutputB8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
