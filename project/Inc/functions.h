#ifndef __func_H
#define __func_H

#include "stm32f2xx_hal.h"
extern void copy_func();
extern void motor_func(int motor_speed);
extern void send_func();
unsigned int checksum(uint8_t *buff,int k);
extern void GPIO_pwm(uint16_t GPIO_Pin, GPIO_PinState PinState, uint8_t duty);
extern void motor_rotate_forward(char sens, uint8_t duty_motor);
extern void motor_rotate_backward(char sens, uint8_t duty_motor);
extern int allign(uint8_t buff[26]);
extern unsigned int checksum(uint8_t buff[],int k);
extern int watchdog;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void free_run();

/********************Defines***************/

#define pwm_period 400



#define Led_gpio      GPIOE
#define Led_pin       GPIO_PIN_2
#define Led_pin2       GPIO_PIN_3
/*
#define ph_u_l        GPIO_PIN_14         //N1
#define ph_v_l        GPIO_PIN_7         //N3
#define ph_w_l        GPIO_PIN_6          //N2

#define ph_u_h        GPIO_PIN_2//0				//P1
#define ph_v_h        GPIO_PIN_1				//P3
#define ph_w_h        GPIO_PIN_0//14				//P2

#define sens_gpio     GPIOD
#define sens_ph1_pin  GPIO_PIN_11
#define sens_ph2_pin  GPIO_PIN_10
#define sens_ph3_pin  GPIO_PIN_9

// Motor controller dedicated pins 
#define DcCal_gpio    GPIOD
#define DcCal_pin     GPIO_PIN_3
#define EnGate_gpio   GPIOB
#define EnGate_pin    GPIO_PIN_8

#define pulse_1 TIM4->CCR2       //n2                           // compare registers of channels for motor pwm
#define pulse_2 TIM4->CCR1       //n3
#define pulse_3 TIM4->CCR3       //n1
*/
#define ph_u_l        GPIO_PIN_14         //C
#define ph_v_l        GPIO_PIN_7         //B
#define ph_w_l        GPIO_PIN_6          //A

#define ph_u_h        GPIO_PIN_2				//C
#define ph_v_h        GPIO_PIN_1				//B
#define ph_w_h        GPIO_PIN_0			//A

#define sens_gpio     GPIOD
#define sens_ph1_pin  GPIO_PIN_11
#define sens_ph2_pin  GPIO_PIN_10
#define sens_ph3_pin  GPIO_PIN_9

/* Motor controller dedicated pins */
#define DcCal_gpio    GPIOD
#define DcCal_pin     GPIO_PIN_3
#define EnGate_gpio   GPIOB
#define EnGate_pin    GPIO_PIN_8

#define pulse_B TIM4->CCR2       //V->B                       // compare registers of channels for motor pwm
#define pulse_A TIM4->CCR1       //W->A  
#define pulse_C TIM4->CCR3       //U->c
/******************Global Variables**************/


/******************Structures**************/
struct ComRecDataPack{
	char delimiter[4];
	float steering_angle;
	float motor_power_percent;
	unsigned int dev_time;
	unsigned int chksum;
};
struct ComRecDataPack rec_pack,pack1,pack2;

struct ComTrDataPack{
	char delimiter[4];
	unsigned int enc0;
	unsigned int enc1;
	unsigned int enc2;
	unsigned int enc3;
	unsigned int steer_ang;
	unsigned int swing_ang0;
	unsigned int swing_ang1;
	unsigned int swing_ang2;
	unsigned int swing_ang3;
	unsigned int motor_current;
	unsigned int dev_time;
	unsigned int chksum;
};
struct ComTrDataPack tra_pack;
#endif
