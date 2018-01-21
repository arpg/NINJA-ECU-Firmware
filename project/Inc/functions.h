#ifndef __func_H
#define __func_H

/********************Includes***************/
#include "stm32f2xx_hal.h"
#include "usart.h"
#include "adc.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
/********************Function Definitions***************/

void copy_func();
void motor_func(int motor_speed);
void send_func();
unsigned int checksum(uint8_t *buff,int k);
void GPIO_pwm(uint16_t GPIO_Pin, GPIO_PinState PinState, uint8_t duty);
void motor_rotate_forward(char sens, uint8_t duty_motor);
void motor_rotate_backward(char sens, uint8_t duty_motor);
int allign(uint8_t buff[26]);
unsigned int checksum(uint8_t buff[],int k);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void free_run();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void motor_pid(float desired_speed);
void SetGainP(double p);
void SetGainI(double i);
void SetGainD(double d);
void SetPlantError(double error/*desired-current*/);
double GetControlOutput(double time);

/*******************Macros***************/

#define pwm_period 400

#define Led_gpio      GPIOE
#define Led_pin       GPIO_PIN_2
#define Led_pin2       GPIO_PIN_3

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

#define enco1 TIM3->CNT
#define enco2 TIM2->CNT
#define enco3 TIM8->CNT
#define enco4 TIM5->CNT
#define clk TIM6->CNT

#define servo_d1 TIM1->CCR1
#define servo_d2 TIM1->CCR2


#define rads_sec 2000*M_PI			//Constant value for calculating rads/sec on encoders
#define tick 1000000*(M_PI/6)
#define motor_limit 80
#define angle_conv 0.75/1100

/******************Global Variables**************/
uint16_t adc[8];



/******************Structures**************/

struct ComRecDataPack
{
	char delimiter[4];
	float steering_angle;
	float rear_steering_angle;
	float motor_power_percent;
	unsigned int dev_time;
	unsigned int chksum;
};
struct ComRecDataPack rec_pack;

struct ComTrDataPack
{
	char delimiter[4];
	float enc0;
	float enc1;
	float enc2;
	float enc3;
	float steer_ang;
	float rear_steer_ang;
	unsigned int swing_ang0;
	unsigned int swing_ang1;
	unsigned int swing_ang2;
	unsigned int swing_ang3;
	unsigned int motor_current;
	unsigned int batt_volt;
	unsigned int dev_time;
	unsigned int chksum;
};
struct ComTrDataPack tra_pack;

struct clock_structure
{
	uint32_t lower_value_tick;
	uint32_t higher_value_tick;
};
struct clock_structure encoder_clock, motor_clock, pid_clock;

struct PID_Controller{
	double gain_p_;
	double gain_i_;
	double gain_d_;
	double integral_value;
	double prev_error_;
	double curr_error_;
};
struct PID_Controller bldc_motor_pid;
#endif
