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
    attachInterrupt(digitalPinToInterrupt(_enc_pin_A), onEncA, RISING);
    attachInterrupt(digitalPinToInterrupt(_enc_pin_B), onEncB, RISING);
    
    // The following is managed by a seperate function:
    // 9) Set dead-time:
    setDeadTime(5, 5);
	
}

void MCPWM_PWM_ENCODER::setupChannel(int gpio_num, mcpwm_cmpr_handle_t &cmpr, mcpwm_gen_handle_t &gen)
{
    mcpwm_comparator_config_t _ccfg =
    {
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

void IRAM_ATTR MCPWM_PWM_ENCODER::onEncA(void* ptr)
{
    
    MCPWM_PWM_ENCODER* self = static_cast<MCPWM_PWM_ENCODER*>(ptr); // casts the void* ptr to a pointer of type MCPWM_PWM_ENCODER*, allowing you to access the class's instance.
    uint32_t t = micros();
    if (!self->_debounceEnabled || (t - self->_lastEncATime > self->_debounceUs))
    {
        self->_encCountA++;
        self->_lastEncATime = t;
    }
    
    /*
    // old version of this function without the pointer casting:
    uint32_t t = micros();
    if (!_debounceEnabled || (t - _lastEncATime > _debounceUs))
    {
        _encCountA++;
        _lastEncATime = t;
    }
    */
}

void IRAM_ATTR MCPWM_PWM_ENCODER::onEncB(void* ptr)
{
    
    MCPWM_PWM_ENCODER* self = static_cast<MCPWM_PWM_ENCODER*>(ptr); // casts the void* ptr to a pointer of type MCPWM_PWM_ENCODER*, allowing you to access the class's instance.
    uint32_t t = micros();
    if (!self->_debounceEnabled || (t - self->_lastEncBTime > self->_debounceUs))
    {
        self->_encCountB++;
        self->_lastEncBTime = t;
    }
    
    /*
    // old version of this function without the pointer casting:
    uint32_t t = micros();
    if (!_debounceEnabled || (t - _lastEncBTime > _debounceUs))
    {
        _encCountB++;
        _lastEncBTime = t;
    }
    */
}


int32_t MCPWM_PWM_ENCODER::get_encCountA(bool resetAfterRead = false)
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

int32_t MCPWM_PWM_ENCODER::get_encCountB(bool resetAfterRead = false)
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
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(genA, genA, &_dt_cfg));

    _dt_cfg.posedge_delay_ticks = 0;
    _dt_cfg.negedge_delay_ticks = _deadTimerFedTicks;
    _dt_cfg.flags.invert_output = true;
    ESP_ERROR_CHECK( mcpwm_generator_set_dead_time(genA, genB, &_dt_cfg));
}

void MCPWM_PWM_ENCODER::setEncoderDebounce(uint32_t us = 100)
{
    if(!us) // us = 0
    {
        _debounceEnabled = false;
    }
    
}



/*
#include <RotaryEncoderPCNT.h>
#include <Bounce2.h>

#include <Arduino.h>
extern "C"
{
    #include "driver/mcpwm_prelude.h"
}

enum Motor     { MOTOR_A = 0, MOTOR_B = 1 };
enum Direction { STOP    = 0, FORWARD = 1, BACKWARD = 2 };

// ===== L298 Control Pins =====
static const int IN1_pin = 1;
static const int IN2_pin = 2;
static const int IN3_pin = 4;
static const int IN4_pin = 5;

// pin_map[motor][0] → INx_pin (drive pin A)  
// pin_map[motor][1] → INx_pin (drive pin B)
static const int pin_map[2][2] =
{
    { IN1_pin, IN2_pin },
    { IN3_pin, IN4_pin }
};

// dir_map[direction][0] → state for pin_map[][0]  
// dir_map[direction][1] → state for pin_map[][1]
static const int dir_map[3][2] =
{
    { LOW,  LOW  },  // STOP
    { LOW,  HIGH },  // FORWARD
    { HIGH, LOW  }   // BACKWARD
};

// ===== Configurable parameters =====
static const int      pwm_pin_A        = 17;
static const int      pwm_pin_B        = 18;
static const uint32_t PWM_HZ           = 21000;
static const uint8_t  RES_BITS         = 10;
static const uint32_t RESOLUTION       = (1u << RES_BITS);
static const uint32_t PWM_PERIOD_TICKS = RESOLUTION;

// ===== quadrature encoder & Button =====
static const uint32_t MAX_DUTY         = RESOLUTION;
static const int      quadrature_encoder_pin_A = 10;
static const int      quadrature_encoder_pin_B = 9;
static const int      quadrature_vcc_pin       = 8;
static const int      quadrature_button_pin    = 7;
static const int      rgb_pin                  = 21;

RotaryEncoderPCNT encoder(quadrature_encoder_pin_A, quadrature_encoder_pin_B);// Constructor args are: pinA/Clock, pinB/Data, start position(defaults=0), glitch filter time in ns(defaults=1000).
Bounce2::Button button = Bounce2::Button();

static bool isRunning   = 0;
static bool isInForward = 0;
static bool zeroWasCalled = 0;
int         oldPosition;
int         position;


// ===== Runtime dead-time =====
static uint32_t       deadTimeRed      = 0;
static uint32_t       deadTimeFed      = 0;

// ===== MCPWM handles =====
mcpwm_timer_handle_t pwm_timer;
mcpwm_oper_handle_t  pwm_oper;
mcpwm_cmpr_handle_t  cmprA, cmprB;
mcpwm_gen_handle_t   genA, genB;

// ===== Encoder pins & state =====
static const int     enc_pin_A        = 15;
static const int     enc_pin_B        = 16;
volatile int32_t     encCountA        = 0;
volatile int32_t     encCountB        = 0;

// ===== Debounce settings =====
static bool          debounceEnabled  = true;
static const uint32_t debounceUs      = 100;  // µs
volatile uint32_t    lastEncATime     = 0;
volatile uint32_t    lastEncBTime     = 0;

// —— Encoder ISRs with optional debounce ——
void IRAM_ATTR onEncA()
{
    uint32_t t = micros();
    if (!debounceEnabled || (t - lastEncATime > debounceUs))
    {
        encCountA++;
        lastEncATime = t;
    }
}

void IRAM_ATTR onEncB()
{
    uint32_t t = micros();
    if (!debounceEnabled || (t - lastEncBTime > debounceUs))
    {
        encCountB++;
        lastEncBTime = t;
    }
}

// —— Encoder getters ——
int32_t getEncCountA(bool resetAfterRead = false)
{
    noInterrupts();
    int32_t c = encCountA;
    if (resetAfterRead)
    {
        encCountA = 0;
    }
    interrupts();
    return c;
}

int32_t getEncCountB(bool resetAfterRead = false)
{
    noInterrupts();
    int32_t c = encCountB;
    if (resetAfterRead)
    {
        encCountB = 0;
    }
    interrupts();
    return c;
}

// —— PWM channel initializer ——
void initChannel(int gpio_num, mcpwm_cmpr_handle_t &cmpr, mcpwm_gen_handle_t &gen)
{
    mcpwm_comparator_config_t ccfg =
    {
        .flags = { .update_cmp_on_tez = true }
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(pwm_oper, &ccfg, &cmpr));

    mcpwm_generator_config_t gcfg =
    {
        .gen_gpio_num = gpio_num
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(pwm_oper, &gcfg, &gen));

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

// —— Raw-duty setter ——
void setDutyCycle(mcpwm_cmpr_handle_t cmpr, uint32_t level)
{
    if (level > RESOLUTION)
    {
        level = RESOLUTION;
    }
    ESP_ERROR_CHECK(
        mcpwm_comparator_set_compare_value(cmpr, level)
    );
}

// —— Dead-time setter ——
void setDeadTime(uint32_t red_ticks, uint32_t fed_ticks)
{
    mcpwm_dead_time_config_t dt_cfg =
    {
        .posedge_delay_ticks = red_ticks,
        .negedge_delay_ticks = 0,
        .flags               = { .invert_output = false }
    };
    ESP_ERROR_CHECK(
        mcpwm_generator_set_dead_time(genA, genA, &dt_cfg)
    );

    dt_cfg.posedge_delay_ticks = 0;
    dt_cfg.negedge_delay_ticks = fed_ticks;
    dt_cfg.flags.invert_output = true;
    ESP_ERROR_CHECK(
        mcpwm_generator_set_dead_time(genA, genB, &dt_cfg)
    );
}

// —— L298 helpers ——
void setMotorDirection(uint8_t motor, uint8_t direction)
{
    // guard against out-of-range
    if (motor > MOTOR_B || direction > BACKWARD)
    {
        return;
    }

    int p0 = pin_map[motor][0];
    int p1 = pin_map[motor][1];

    digitalWrite(p0, dir_map[direction][0]);
    digitalWrite(p1, dir_map[direction][1]);
}

void setWindDirection()
{
    setMotorDirection(MOTOR_A, FORWARD);
    setMotorDirection(MOTOR_B, BACKWARD);
}

void setRewindDirection()
{
    setMotorDirection(MOTOR_A, BACKWARD);
    setMotorDirection(MOTOR_B, FORWARD);
}



// —— Setup ——
void setup()
{
    // 1) Enable quadrature encoder & it's button:
    pinMode(quadrature_vcc_pin, OUTPUT);
    digitalWrite(quadrature_vcc_pin, HIGH);

    button.attach(quadrature_button_pin, INPUT_PULLDOWN);
    button.interval(5);
    button.setPressedState(HIGH);

    // 2) MCPWM Timer
    mcpwm_timer_config_t tcfg =
    {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_HZ * RESOLUTION,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = PWM_PERIOD_TICKS
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &pwm_timer));

    // 3) Operator
    mcpwm_operator_config_t ocfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &pwm_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(pwm_oper, pwm_timer));

    // 4) PWM Channels
    initChannel(pwm_pin_A, cmprA, genA);
    initChannel(pwm_pin_B, cmprB, genB);

    // 5) Start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop( pwm_timer, MCPWM_TIMER_START_NO_STOP ));

    // 6) Initial duties (0%)
    setDutyCycle(cmprA, 0);
    setDutyCycle(cmprB, 0);

    // 7) Encoders
    pinMode(enc_pin_A, INPUT);
    pinMode(enc_pin_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(enc_pin_A), onEncA, RISING);
    attachInterrupt(digitalPinToInterrupt(enc_pin_B), onEncB, RISING);

    // 8) Set dead-time:
    setDeadTime(deadTimeRed, deadTimeFed);

    // 9) L298
    pinMode(IN1_pin, OUTPUT);
    pinMode(IN2_pin, OUTPUT);
    pinMode(IN3_pin, OUTPUT);
    pinMode(IN4_pin, OUTPUT);

    // 10) Establish correct direction
    setWindDirection();

    //rgbLedWrite(rgb_pin, 100, 0, 100);
    oldPosition = encoder.position();
}


void loop()
{
    // store current time & update button state:
    uint32_t now = millis(); 
    button.update();

    // possibly act on button being pressed:
    if(button.pressed())
    {
        encoder.zero();
        zeroWasCalled = true;
    }

    // Determine if encoder have moved:
    position = encoder.position();
    if(position != oldPosition)
    {
        if(zeroWasCalled)
        {
            int diff = position - oldPosition;
            if(diff > 0)
            {

            }
            else if(diff < 0)
            {

            }
        }
    }
    // Update new duty cycle value.
        Serial.println(position);
        oldPosition = position;
    

    // 1) Control at fixed interval
    if (now - lastControlTime >= CONTROL_INTERVAL_MS)
    {
        lastControlTime = now;

        // a) Pot → baseline duty (0…RESOLUTION)
        uint32_t raw       = analogRead(pot_pin);  
        uint32_t baseDuty  = map(raw, 0, 4095, 0, MAX_DUTY);

        // b) Apply to Motor A
        setDutyCycle(cmprA, baseDuty);

        // c) Measure pulses this interval
        int32_t cntA = getEncCountA(true);
        int32_t cntB = getEncCountB(true);

        // d) PI on Motor B: error = pulsesA − pulsesB
        float   error    = (float)(cntA - cntB);
        errorInt        += error;
        int32_t delta    = (int32_t)(Kp * error + Ki * errorInt);

        // e) Compute Motor B duty = base + delta
        int32_t dutyB    = (int32_t)baseDuty + delta;
        dutyB            = constrain(dutyB, 0, (int32_t)MAX_DUTY);

        setDutyCycle(cmprB, dutyB);

        // f) Debug
        printf(
            "A→%u ticks, B→%u ticks, err=%d\n",
            cntA,
            cntB,
            cntA - cntB
        );
    }
}
*/
{
    #include "driver/mcpwm_prelude.h"
}


