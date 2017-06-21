#include "functions.h"
#include "usart.h"
#include "adc.h"
#include <string.h>
#include <stdlib.h>
uint8_t anv=0x30;
uint8_t mcpy=0;
int a=0,b,packet_size;
int misalligned=0;
unsigned char state= 0;  
int recev; 
uint8_t tx_buff[4];     
uint8_t mx_buff[60],mx_buff1[60];
uint8_t buff11[22]; 
uint32_t enc1[8],enc2[8];
#define enco1 TIM3->CNT 
#define enco2 TIM2->CNT 
#define enco3 TIM8->CNT 
#define enco4 TIM5->CNT 
#define servo_d1 TIM1->CCR1
#define servo_d2 TIM1->CCR3

uint32_t adc[10];
float motor_p;



void copy_func()
{
		int angle;
		HAL_UART_Receive_DMA(&huart3,(uint8_t *)&rec_pack,sizeof(rec_pack));
		if (rec_pack.chksum==checksum((uint8_t *)(&rec_pack),sizeof(rec_pack)))
		{
			angle= 75+(25*rec_pack.steering_angle);
			servo_d1=angle;
			servo_d2=75;
			motor_p=rec_pack.motor_power_percent;
			watchdog=0;
		}
}

void motor_func(int motor_speed)
{

    /* get the rotor state of the bldc */
//    state = HAL_GPIO_ReadPin(sens_gpio,sens_ph1_pin);
//    state = state << 1;
//    state |= HAL_GPIO_ReadPin(sens_gpio,sens_ph2_pin);
//    state = state << 1;
//    state |= HAL_GPIO_ReadPin(sens_gpio,sens_ph3_pin);

		uint32_t port_value = sens_gpio->IDR;
		state = 0;
		if(port_value&sens_ph1_pin){
			state = 0x04;
		}
		if(port_value&sens_ph2_pin){
			state |= 0x02;
		}
		if(port_value&sens_ph3_pin){
			state |= 0x01;
		}

    if(motor_speed > 0)
    {
      motor_rotate_forward(state, abs(motor_speed));   
    }
    else if(motor_speed < 0)
    {
      motor_rotate_backward(state, abs(motor_speed));
    }
    else
    {
    	free_run();
    }
}

void send_func()
{
			HAL_GPIO_WritePin(Led_gpio, Led_pin, GPIO_PIN_SET);
			//HAL_GPIO_TogglePin(Led_gpio,Led_pin2);
			//HAL_ADC_Start_DMA(&hadc1,adc,8);
			tra_pack.delimiter[0]=0xAA;
			tra_pack.delimiter[1]=0x55;
			tra_pack.delimiter[2]=0xe1;
			tra_pack.delimiter[3]=0x1e;
			tra_pack.enc0=enco1;
			tra_pack.enc1=enco1;
			tra_pack.enc2=enco1;
			tra_pack.enc3=enco1;
		
//			tra_pack.enc0=21;
//			tra_pack.enc1=22;
//			tra_pack.enc2=23;
//			tra_pack.enc3=24;
	

			tra_pack.dev_time=7;
			
//			tra_pack.steer_ang=adc[0];
//			tra_pack.swing_ang0=adc[1];
//			tra_pack.swing_ang1=adc[2];
//			tra_pack.swing_ang2=adc[3];
//			tra_pack.swing_ang3=adc[4];
//			tra_pack.motor_current=adc[5];	
			tra_pack.steer_ang=11;
			tra_pack.swing_ang0=12;
			tra_pack.swing_ang1=13;
			tra_pack.swing_ang2=14;
			tra_pack.swing_ang3=15;
			tra_pack.motor_current=3;	
			
			tra_pack.chksum=checksum( (uint8_t *)(&tra_pack),sizeof(tra_pack));
			HAL_UART_Transmit_DMA(&huart3,(uint8_t *)&tra_pack,sizeof(tra_pack));
	//		HAL_GPIO_TogglePin(Led_gpio,Led_pin2);
//
			HAL_GPIO_WritePin(Led_gpio, Led_pin, GPIO_PIN_RESET);
}

unsigned int checksum(uint8_t *buff,int k)
{
	unsigned int sum=0;
	for (int i=0; i<(k-4);i++)
	{
		sum+=buff[i];
	}
	return sum;
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
							
							return (i);
						}
					}
				}
		}		
		return (-1);
}

void GPIO_pwm(uint16_t GPIO_Pin, GPIO_PinState PinState, uint8_t duty)
{
  /* calculation of duty cycle relative to period */
  uint32_t duty_cycle_motor = 0;
  duty_cycle_motor = (uint32_t)(duty * pwm_period)/100; 
  /* if pin state is set apply the duty cycle value to the compare register of the particular pin */
  if(PinState == GPIO_PIN_SET)
  {
    switch(GPIO_Pin)
     {
        case(ph_u_l):
          pulse_C = duty_cycle_motor;
        break;

        case(ph_v_l):
          pulse_B = duty_cycle_motor;
        break;

        case(ph_w_l):
          pulse_A = duty_cycle_motor;
        break;
    }
  } 
  /* if pin state is reset then reset the pin */
  else 
  {
    switch(GPIO_Pin)
     {
        case(ph_u_l):
         pulse_C = 0;
        break;

        case(ph_v_l):
          pulse_B = 0;
        break;

        case(ph_w_l):
          pulse_A = 0;
        break;
     }
  }
}
void free_run()
{
				GPIO_pwm(ph_u_l,GPIO_PIN_RESET,0);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,0);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET, 0);
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
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
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_SET, duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
        break;
      case 4:   
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
        GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
        break;
      case 6:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
        GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
        break;
      case 2:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);   
        GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
        break;
      case 3:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_SET);
        GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
        break;
      case 1: 
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);   
        GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_SET);
        break;
      default:
        free_run();
				break;
      }
}

/* This function returns NULL and is mostly responsible for driving the BLDC motor in reverse direction
   The low side of the MOSFETs are responsible for providing PWMs while the high side of the MOSFETs are 
    hard coded. The MOSFETs are arranged in a hex converter configuration. There are 6 rotor states.*/
void motor_rotate_backward(char sens, uint8_t duty_motor) {
  switch(sens) {
    case 4:   
      HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
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
      HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_SET);
      GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);      
      break;
    case 2:   
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_SET);      
      break;
    case 3:   
      HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
      GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
      break;
    case 1:   
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
      break;
    case 5: 
      HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
      GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
      GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
      HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
      GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
      break;
      default:
        free_run();
				break;
    }
}
/*
void GPIO_pwm(uint16_t GPIO_Pin, GPIO_PinState PinState, uint8_t duty)
{
  // calculation of duty cycle relative to period 
  uint8_t duty_cycle_motor = 0;
  duty_cycle_motor = (duty * pwm_period)/100; 
  // if pin state is set apply the duty cycle value to the compare register of the particular pin 
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
    }
  } 
  // if pin state is reset then reset the pin 
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
     }
  }
}

// This function returns NULL and is mostly responsible for driving the BLDC motor in forward direction
//   The low side of the MOSFETs are responsible for providing PWMs while the high side of the MOSFETs are 
//    hard coded. The MOSFETs are arranged in a hex converter configuration. There are 6 rotor states.
void motor_rotate_forward(char sens, uint8_t duty_motor)
{
    switch(sens) {
      case 5:   
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_SET, duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
        break;
      case 4:   
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);  
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_SET);
        GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
        break;
      case 6:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
        GPIO_pwm(ph_w_l,GPIO_PIN_SET,duty_motor);
        break;
      case 2:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);   
        GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_SET);
        break;
      case 3:   
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_SET);
        GPIO_pwm(ph_u_l,GPIO_PIN_SET,duty_motor);
        break;
      case 1: 
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);   
        GPIO_pwm(ph_v_l,GPIO_PIN_SET,duty_motor);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_SET);
        break;
      default:
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
				break;
      }
}

// This function returns NULL and is mostly responsible for driving the BLDC motor in reverse direction
//   The low side of the MOSFETs are responsible for providing PWMs while the high side of the MOSFETs are 
//    hard coded. The MOSFETs are arranged in a hex converter configuration. There are 6 rotor states.
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
        HAL_GPIO_WritePin(GPIOB,ph_u_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,ph_v_h,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE,ph_w_h,GPIO_PIN_RESET);
        GPIO_pwm(ph_v_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_u_l,GPIO_PIN_RESET,duty_motor);
        GPIO_pwm(ph_w_l,GPIO_PIN_RESET,duty_motor);
			break;
	}
}
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	send_func();
	UNUSED(huart);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
/*
	if((mx_buff[0] == 0xAA) && !(mx_buff[1]) && !(mx_buff[2]) && !(mx_buff[3])){
		HAL_UART_Receive_DMA(&huart3,&mx_buff[1],packet_size-1);
	}*/
	int pos;
	if(misalligned)
		{
			misalligned=0;
			HAL_UART_Receive_DMA(&huart3,(uint8_t *)&rec_pack,sizeof(rec_pack));
		}
	if((rec_pack.delimiter[0] == 0xAA) && (rec_pack.delimiter[1] == 0x55) && (rec_pack.delimiter[2] == 0xE1) && (rec_pack.delimiter[3] == 0x1E)){
		copy_func();
	}
	else if((allign((uint8_t *)(&rec_pack)) != 20)&&(allign((uint8_t *)(&rec_pack)) != -1))
	{
		misalligned=1;
		pos=allign((uint8_t *)(&rec_pack));
		HAL_UART_Receive_DMA(&huart3,&buff11[0],pos);
	}


}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	//UNUSED(huart);
}


