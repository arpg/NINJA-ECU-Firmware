#include "functions.h"

unsigned char state = 0;
extern uint32_t uwTick;
extern uint32_t watch;
extern uint32_t timer;
int misalligned=0;
uint8_t buff11[22];
uint32_t encoder_previous_position[4];
double motor_speed;
float motor_p;
uint8_t previous_state;
int speed[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t index_t = 0;



void copy_func()
{

		int angle;
		int angle_rear;
		if (rec_pack.chksum==checksum((uint8_t *)(&rec_pack),sizeof(rec_pack)))
		{
			angle = 75+(25*rec_pack.steering_angle);
			angle_rear = 75+(25*rec_pack.rear_steering_angle);

			servo_d1=angle;
			servo_d2=angle_rear;
			motor_p=rec_pack.motor_power_percent;
			watch=0;
		}
		HAL_UART_Receive_DMA(&huart3,(uint8_t *)&rec_pack,sizeof(rec_pack));
}

void send_func()
{
			uint32_t time1;
			if(encoder_clock.higher_value_tick == uwTick)
			{
				time1 = clk-encoder_clock.lower_value_tick;
			}
			else
			{
				time1 = (uwTick-encoder_clock.higher_value_tick-1)*1000+((1000-encoder_clock.lower_value_tick)+clk);
				encoder_clock.higher_value_tick = uwTick;
				encoder_clock.lower_value_tick = clk;
			}
			//int32_t time1 = (uwTick*1000+TIM6->CNT);					//Finds time since last buffer was sent


			float encoder0_speed = (((int)(encoder_previous_position[0] - enco1))*rads_sec)/time1;		//Calculates speed of encoder in rads/sec
			float encoder1_speed = (((int)(encoder_previous_position[1] - enco2))*rads_sec)/time1;		//Calculates speed of encoder in rads/sec
			float encoder2_speed = (((int)(encoder_previous_position[2] - enco3))*rads_sec)/time1;		//Calculates speed of encoder in rads/sec
			float encoder3_speed = (((int)(encoder_previous_position[3] - enco4))*rads_sec)/time1;		//Calculates speed of encoder in rads/sec

			//HAL_ADC_Start_DMA(&hadc1,adc,8);
			tra_pack.delimiter[0]=0xAA;
			tra_pack.delimiter[1]=0x55;
			tra_pack.delimiter[2]=0xe1;
			tra_pack.delimiter[3]=0x1e;

			tra_pack.enc0=encoder0_speed;			//fl wheel
			tra_pack.enc1=encoder1_speed;			//rl wheel
			tra_pack.enc2=encoder2_speed;			//rr wheel
			tra_pack.enc3=encoder3_speed;			//fr wheel
			//tra_pack.enc3=motor_speed;


			//double angle_conv=1;
			tra_pack.dev_time=uwTick*1000+TIM6->CNT;
			//tra_pack.dev_time = (sens_gpio->IDR&(sens_ph1_pin|sens_ph2_pin|sens_ph3_pin))>>9;
			tra_pack.steer_ang=(adc[0]-2180)*angle_conv;
			tra_pack.rear_steer_ang=(adc[1]-2100)*angle_conv;
			tra_pack.swing_ang0=adc[2];
			tra_pack.swing_ang1=adc[3];
			tra_pack.swing_ang2=adc[4];
			tra_pack.swing_ang3=adc[5];
			tra_pack.batt_volt=adc[6];
			tra_pack.motor_current=adc[7];

			
			tra_pack.chksum=checksum( (uint8_t *)(&tra_pack),sizeof(tra_pack));

			encoder_previous_position[0] = enco1;
			encoder_previous_position[1] = enco2;
			encoder_previous_position[2] = enco3;
			encoder_previous_position[3] = enco4;

			 HAL_UART_Transmit_DMA(&huart3,(uint8_t *)&tra_pack,sizeof(tra_pack));

			//uwTick = 0;						//resets timer's higher value tick
			//TIM6->CNT = 0;					//resets timer's lower value tick
}

void motor_func(int motor_speed_)
{
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

	if(motor_speed_ > motor_limit)
	{
		motor_speed_ = motor_limit;
	}
	else if(motor_speed_ < -motor_limit)
	{
		motor_speed_ = -motor_limit;
	}
    if(motor_speed_ > 0)
    {
      motor_rotate_forward(state, abs(motor_speed_));
    }
    else if(motor_speed_ < 0)
    {
      motor_rotate_backward(state, abs(motor_speed_));
    }
    else
    {
    	free_run();
    }
}

void motor_pid(float desired_speed)
{
	if((desired_speed<50)&&(desired_speed>-50))
	{
		desired_speed = 0;
	}
	double time;
	if(pid_clock.higher_value_tick == uwTick)
	{
		time = clk-pid_clock.lower_value_tick;
	}
	else
	{
		time = (uwTick-pid_clock.higher_value_tick-1)*1000+((1000-pid_clock.lower_value_tick)+clk);
		pid_clock.higher_value_tick = uwTick;
		pid_clock.lower_value_tick = clk;
	}

	double error = desired_speed-motor_speed;
	//tra_pack.enc1 = error;
	//tra_pack.enc2 = desired_speed;
	SetPlantError(error);
	double magnitude = GetControlOutput(time);
	//tra_pack.enc0 = -magnitude;
	motor_func(-magnitude);
}

void SetGainP(double p)
{
	bldc_motor_pid.gain_p_ = p;
}

void SetGainI(double i)
{
	bldc_motor_pid.gain_i_ = i;
}

void SetGainD(double d)
{
	bldc_motor_pid.gain_d_ = d;
}

void SetPlantError(double error/*desired-current*/)
{
	bldc_motor_pid.prev_error_ = bldc_motor_pid.curr_error_;
	bldc_motor_pid.curr_error_ = error;
}

double GetControlOutput(double time)
{
	double p_cntrl = bldc_motor_pid.gain_p_*(bldc_motor_pid.curr_error_);

	double d_cntrl = bldc_motor_pid.gain_d_*((bldc_motor_pid.curr_error_-bldc_motor_pid.prev_error_)*1000000/time);

	bldc_motor_pid.integral_value += bldc_motor_pid.prev_error_*(time*0.000001);

	double i_cntrl = bldc_motor_pid.gain_i_*bldc_motor_pid.integral_value;

	return(p_cntrl+d_cntrl+i_cntrl);
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	send_func();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int pos;
	if(misalligned)
	{
		misalligned=0;
		HAL_UART_Receive_DMA(&huart3,(uint8_t *)&rec_pack,sizeof(rec_pack));
	}
	if((rec_pack.delimiter[0] == 0xAA) && (rec_pack.delimiter[1] == 0x55) && (rec_pack.delimiter[2] == 0xE1) && (rec_pack.delimiter[3] == 0x1E))
	{
		copy_func();
	}
	else if((allign((uint8_t *)(&rec_pack)) != 20)&&(allign((uint8_t *)(&rec_pack)) != -1))
	{
		misalligned=1;
		pos=allign((uint8_t *)(&rec_pack));
		HAL_UART_Receive_DMA(&huart3,&buff11[0],pos);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if((GPIO_Pin == GPIO_PIN_9)||(GPIO_Pin == GPIO_PIN_10)||(GPIO_Pin == GPIO_PIN_11))
	{
		uint32_t time;
		//HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);

		if(motor_clock.higher_value_tick == uwTick)
		{
			time = clk-motor_clock.lower_value_tick;
			motor_clock.lower_value_tick = clk;
		}
		else
		{
			time = (uwTick-motor_clock.higher_value_tick-1)*1000+((1000-motor_clock.lower_value_tick)+clk);
			motor_clock.higher_value_tick = uwTick;
			motor_clock.lower_value_tick = clk;

		}

		//time = timer*50000+TIM7->CNT;
		//timer = 0;
		//TIM7->CNT = 0;
		uint8_t current_state = (sens_gpio->IDR&(sens_ph1_pin|sens_ph2_pin|sens_ph3_pin))>>9;
		switch(current_state)
		{
		case(1):
			if(previous_state == 5)
			{
				//motor_speed = tick/time;
				speed[index_t] = tick/time;
			}
			else if(previous_state == 3)
			{
				//motor_speed = -tick/time;
				speed[index_t] = -tick/time;
			}
			break;

		case(3):
			if(previous_state == 1)
			{
				speed[index_t] = tick/time;
				//motor_speed = tick/time;
			}
			else if(previous_state == 2)
			{
				//motor_speed = -tick/time;
				speed[index_t] = -tick/time;
			}
			break;

		case(2):
			if(previous_state == 3)
			{
//				motor_speed = tick/time;
				speed[index_t] = tick/time;
			}
			else if(previous_state == 6)
			{
//				motor_speed = -tick/time;
				speed[index_t] = -tick/time;
			}
			break;

		case(6):
			if(previous_state == 2)
			{
//				motor_speed = tick/time;
				speed[index_t] = tick/time;
			}
			else if(previous_state == 4)
			{
//				motor_speed = -tick/time;
				speed[index_t] = -tick/time;
			}
			break;

		case(4):
			if(previous_state == 6)
			{
//				motor_speed = tick/time;
				speed[index_t] = tick/time;
			}
			else if(previous_state == 5)
			{
//				motor_speed = -tick/time;
				speed[index_t] = -tick/time;
			}
			break;

		case(5):
			if(previous_state == 4)
			{
//				motor_speed = tick/time;
				speed[index_t] = tick/time;
			}
			else if(previous_state == 1)
			{
//				motor_speed = -tick/time;
				speed[index_t] = -tick/time;
			}
			break;
		}
		index_t++;
		if(index_t==12)
		{
			index_t=0;
		}
		previous_state = current_state;
		motor_speed = 0;
		for(int i=0;i<12;i++)
		{
			motor_speed += speed[i];
		}
		motor_speed = motor_speed/12;

	}
	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
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
			return(sizeof(rec_pack));

		}
		else
		{
			for (int i=1;i<sizeof(rec_pack);i++)
				{
					if (buff[i]==0xAA)
					{
						if (buff[(i+1)%sizeof(rec_pack)]==0x55 && buff[(i+2)%sizeof(rec_pack)]==0xE1 && buff[(i+3)%sizeof(rec_pack)]==0x1E)
						{

							return (i);
						}
					}
				}
		}
		return (-1);
}
