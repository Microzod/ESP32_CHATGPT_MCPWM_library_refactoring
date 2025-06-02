#include <Arduino.h>
#include "MCPWM_PWM_ENCODER.h"

extern "C"
{
    #include "driver/mcpwm_prelude.h"
}


MCPWM_PWM_ENCODER::MCPWM_PWM_ENCODER(int pwmPinA, int pwmPinB, int encPinA, int encPinB, uint32_t pwm_freq, uint8_t pwm_resolution)
{
    _pwm_pin_A = pwmPinA;		
    _pwm_pin_B = pwmPinB;	
    _enc_pin_A = encPinA;
    _enc_pin_B = encPinB; 
    _pwm_freq = pwm_freq;			
    _pwm_bit_depth = pwm_resolution;			
    _resolution = (1u << _pwm_bit_depth);		
    _pwm_period_ticks = _resolution;
}

void MCPWM_PWM_ENCODER::initPWM(int startAtDutyCycle = 0)
{
    
    // 1) MCPWM Timer
    mcpwm_timer_config_t _tcfg =
    {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = _pwm_freq * _resolution,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = _pwm_period_ticks,
        .intr_priority = 0
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&_tcfg, &_pwm_timer));

    // 2) Operator
    mcpwm_operator_config_t _ocfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&_ocfg, &_pwm_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_pwm_oper, _pwm_timer));

    // 3) PWM Channels
    setupChannel(_pwm_pin_A, cmprA, _genA);
    setupChannel(_pwm_pin_B, cmprB, _genB);

    // 4) Start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(_pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop( _pwm_timer, MCPWM_TIMER_START_NO_STOP ));


    // 7) Initial duties (0%)
    setDutyCycle(cmprA, startAtDutyCycle);
    setDutyCycle(cmprB, startAtDutyCycle);

    // 8) Encoders
    pinMode(_enc_pin_A, INPUT);
    pinMode(_enc_pin_B, INPUT);
    // Use attachInterruptArg, passing “this” as the context
    attachInterruptArg(digitalPinToInterrupt(_enc_pin_A), MCPWM_PWM_ENCODER::_onEncA, (void*)this, RISING);
    attachInterruptArg(digitalPinToInterrupt(_enc_pin_B), MCPWM_PWM_ENCODER::_onEncB, (void*)this, RISING);

    // The following is managed by a seperate function:
    // 9) Set dead-time:
    setDeadTime(5, 5);
	
}

void MCPWM_PWM_ENCODER::setupChannel(int gpio_num, mcpwm_cmpr_handle_t &cmpr, mcpwm_gen_handle_t &gen)
{
    mcpwm_comparator_config_t _ccfg =
    {
        .intr_priority = 0,
        .flags = { .update_cmp_on_tez = true }
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(_pwm_oper, &_ccfg, &cmpr));

    mcpwm_generator_config_t _gcfg =
    {
        .gen_gpio_num = gpio_num
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(_pwm_oper, &_gcfg, &gen));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    ));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            cmpr,
            MCPWM_GEN_ACTION_LOW
        )
    ));
}

// static ISR wrappers:
void IRAM_ATTR MCPWM_PWM_ENCODER::_onEncA(void* ctx)
{
    auto self = reinterpret_cast<MCPWM_PWM_ENCODER*>(ctx);
    self->handleEncA();
}

void IRAM_ATTR MCPWM_PWM_ENCODER::_onEncB(void* ctx)
{
    auto self = reinterpret_cast<MCPWM_PWM_ENCODER*>(ctx);
    self->handleEncB();
}

// actual member functions:
void MCPWM_PWM_ENCODER::handleEncA()
{
    // note: _encCountA is volatile
    _encCountA++;
}

void MCPWM_PWM_ENCODER::handleEncB()
{
    _encCountB++;
}

int32_t MCPWM_PWM_ENCODER::getEncCountA(bool resetAfterRead)
{
    noInterrupts();
    int32_t c = _encCountA;
    if (resetAfterRead)
    {
        _encCountA = 0;
    }
    interrupts();
    return c;
}

int32_t MCPWM_PWM_ENCODER::getEncCountB(bool resetAfterRead)
{
    noInterrupts();
    int32_t c = _encCountB;
    if (resetAfterRead)
    {
        _encCountB = 0;
    }
    interrupts();
    return c;
}

void MCPWM_PWM_ENCODER::setDutyCycle(mcpwm_cmpr_handle_t cmpr, uint32_t level)
{
    if (level > _resolution)
    {
        level = _resolution;
    }
    ESP_ERROR_CHECK( mcpwm_comparator_set_compare_value(cmpr, level) );
}

void MCPWM_PWM_ENCODER::setDeadTime(uint32_t red_ticks, uint32_t fed_ticks)
{
    // Add range check for red_ticks & fed_ticks
    
    _deadTimerRedTicks = red_ticks;
    _deadTimerFedTicks = fed_ticks;
    
    mcpwm_dead_time_config_t _dt_cfg =
    {
        .posedge_delay_ticks = _deadTimerRedTicks,
        .negedge_delay_ticks = 0,
        .flags               = { .invert_output = false }
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(_genA, _genA, &_dt_cfg));

    _dt_cfg.posedge_delay_ticks = 0;
    _dt_cfg.negedge_delay_ticks = _deadTimerFedTicks;
    _dt_cfg.flags.invert_output = true;
    ESP_ERROR_CHECK( mcpwm_generator_set_dead_time(_genA, _genB, &_dt_cfg));
}
