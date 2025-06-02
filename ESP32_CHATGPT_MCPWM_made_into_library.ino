#include <RotaryEncoderPCNT.h>
#include <Bounce2.h>
#include "MCPWM_PWM_ENCODER.h"

//extern "C"
//{
    #include "driver/mcpwm_prelude.h"
//}

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

// ===== quadrature encoder & Button =====
static const int      quadrature_encoder_pin_A = 10;
static const int      quadrature_encoder_pin_B = 9;
static const int      quadrature_vcc_pin       = 8;
static const int      quadrature_button_pin    = 7;
static const int      rgb_pin                  = 21;

MCPWM_PWM_ENCODER pwm(17, 18, 15, 16, 21000, 10);
RotaryEncoderPCNT encoder(quadrature_encoder_pin_A, quadrature_encoder_pin_B);// Constructor args are: pinA/Clock, pinB/Data, start position(defaults=0), glitch filter time in ns(defaults=1000).
Bounce2::Button button = Bounce2::Button();

static bool isRunning   = 0;
static bool isInForward = 0;
static bool zeroWasCalled = 0;
int         oldPosition;
int         position;


// ===== Debounce settings =====
static bool          debounceEnabled  = true;
static const uint32_t debounceUs      = 100;  // µs
volatile uint32_t    lastEncATime     = 0;
volatile uint32_t    lastEncBTime     = 0;



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

void setSpoolDirection()
{
    setMotorDirection(MOTOR_A, FORWARD);
    setMotorDirection(MOTOR_B, BACKWARD);
}

void setUnspoolDirection()
{
    setMotorDirection(MOTOR_A, BACKWARD);
    setMotorDirection(MOTOR_B, FORWARD);
}



// —— Setup ——
void setup()
{
    // 1) MCPWM setup:
    pwm.initPWM(0);
    // 21) Enable quadrature encoder & it's button:
    pinMode(quadrature_vcc_pin, OUTPUT);
    digitalWrite(quadrature_vcc_pin, HIGH);

    // 3) Button setup:
    button.attach(quadrature_button_pin, INPUT_PULLDOWN);
    button.interval(5);
    button.setPressedState(HIGH);

    // 4) L298 setup:
    pinMode(IN1_pin, OUTPUT);
    pinMode(IN2_pin, OUTPUT);
    pinMode(IN3_pin, OUTPUT);
    pinMode(IN4_pin, OUTPUT);

    // 5) Establish correct direction
    setSpoolDirection();

    // 6) Misc:
    //rgbLedWrite(rgb_pin, 100, 0, 100);
    oldPosition = encoder.position();
    //Serial.begin(115200);
}


void loop()
{
    // store current time & update button state:
    //uint32_t now = millis(); 
    button.update();

    // possibly act on button being pressed:
    if(button.pressed())
    {
        encoder.zero();
        oldPosition = encoder.position();
        pwm.setDutyCycle(pwm.cmprA, 0);
        pwm.setDutyCycle(pwm.cmprB, 0);
        //zeroWasCalled = true;
        printf("Button was pressed\n");
    }

    // Determine if encoder have moved:
    position = encoder.position();
    if(position != oldPosition)
    {
        if(position < 0)
        {
            setUnspoolDirection();
        }
        else
        {
            setSpoolDirection();
        }
        pwm.setDutyCycle(pwm.cmprA, abs(position));
        pwm.setDutyCycle(pwm.cmprB, abs(position));
        printf("encoder value = %d \n", position);
        oldPosition = position;
    }
    

   
}
