
class Drv8302Driver{
public:
	Drv8302Driver(GPIO_TypeDef* sens_ph_gpio,uint16_t sens_ph1_pin,
					uint16_t sens_ph2_pin,uint16_t sens_ph3_pin) {			
		sens_ph_gpio_ = sens_ph_gpio;
		sens_ph1_pin_ = sens_ph1_pin;
		sens_ph2_pin_ = sens_ph2_pin;
		sens_ph3_pin_ = sens_ph3_pin;
	}
	~Drv8302Driver(){
		delete(sens_gpio_);
	}
	// cw: true for clockwise rotation and false for ccw rotation
	// duty_cycle: value between 0-100 
	void Run(bool cw,unsigned char duty_cycle) {
		duty_cycle_ = duty_cycle;
		if(cw)
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
	
private:
	
	unsigned char GetHallFB(void) {
		unsigned char state = 0;
		state = HAL_GPIO_ReadPin(sens_gpio_,sens_ph1_pin_);
		state = state << 1;
		state |= HAL_GPIO_ReadPin(sens_gpio_,sens_ph2_pin_);
		state = state << 1;
		state |= HAL_GPIO_ReadPin(sens_gpio_,sens_ph3_pin_);
		return state;
	}
	
	void ApplyPhaseCW()
	GPIO_TypeDef* sens_ph_gpio_;
	uint16_t sens_ph1_pin_;
	uint16_t sens_ph2_pin_;
	uint16_t sens_ph3_pin_;
	unsigned char duty_cycle_;
}