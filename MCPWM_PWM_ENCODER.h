#ifndef _MCPWM_PWM_ENCODER_H_
#define _MCPWM_PWM_ENCODER_H_

#include <Arduino.h>

extern "C"
{
    #include "driver/mcpwm_prelude.h"
}

// save from internet to not forget:
//void IRAM_ATTR pin_t_isr_handler(void* user_ctx)
//{
//	((PIN_t*)user_ctx)->isr_func();
//}

class MCPWM_PWM_ENCODER
{

public:
	MCPWM_PWM_ENCODER(int pwmPinA = 17, int pwmPinB = 18, int encPinA = 15, int encPinB = 16, uint32_t pwm_freq = 21000, uint8_t pwm_resolution = 10);
	
	void initPWM(int startAtDutyCycle);
    
    int32_t getEncCountA(bool resetAfterRead = false);
    int32_t getEncCountB(bool resetAfterRead = false);
    
    void setDutyCycle(mcpwm_cmpr_handle_t cmpr, uint32_t level);
    void setDeadTime(uint32_t red_ticks, uint32_t fed_ticks);
    void setEncoderDebounce(uint32_t us = 100);
    
    mcpwm_cmpr_handle_t  cmprA, cmprB;
	

private:
    void setupChannel(int gpio_num, mcpwm_cmpr_handle_t &cmpr, mcpwm_gen_handle_t &gen);

    static void IRAM_ATTR onEncA(void* ptr);
    static void IRAM_ATTR onEncB(void* ptr);
    
	int      _pwm_pin_A;		// = 17;
	int      _pwm_pin_B;		// = 18;
	uint32_t _pwm_freq;			// = 21000;
	uint8_t  _pwm_bit_depth;	// = 10;
	uint32_t _resolution;		// = (1u << RES_BITS);
	uint32_t _pwm_period_ticks; // = RESOLUTION;
	
	uint32_t _deadTimerRedTicks      = 0;
	uint32_t _deadTimerFedTicks      = 0;

	mcpwm_timer_handle_t _pwm_timer;
	mcpwm_oper_handle_t  _pwm_oper;
    mcpwm_gen_handle_t   _genA, _genB;
	
	int    				_enc_pin_A;//     = 15;
	int    				_enc_pin_B;//     = 16;
	volatile int32_t	_encCountA       = 0;
	volatile int32_t	_encCountB       = 0;
	volatile uint32_t	_lastEncATime    = 0;
	volatile uint32_t	_lastEncBTime    = 0;
	
	bool				_debounceEnabled = false;
	uint32_t 			_debounceUs      = 100;  // µs
	

};

#endif  // _MCPWM_PWM_ENCODER_H_