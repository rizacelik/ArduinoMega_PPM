#include <Servo_Rc_PWM.h>

// https://github.com/GaloisInc/ardupilot-mega/blob/master/libraries/AP_HAL_AVR/RCInput_APM2.cpp

Servo servo_roll, servo_pitch;

#define AVR_RC_INPUT_NUM_CHANNELS 10

volatile uint16_t _pulse_capt[AVR_RC_INPUT_NUM_CHANNELS] = {0};
volatile uint8_t _valid = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo_pitch.attach(12, 1200, 1800, 1500);
  servo_pitch.writeMicroseconds(1500);
  inits();
}

void loop() {
  // put your main code here, to run repeatedly:
  servo_pitch.writeMicroseconds(_pulse_capt[0]);
  Serial.println(_pulse_capt[0]);
}


void inits(void) {
  /* Arduino Mega pin 48 is ICP5 / PL1,  timer 5 input capture */
  pinMode(48, INPUT);
  /**
     WGM: 1 1 1 1. Fast WPM, TOP is in OCR5A
     COM all disabled
     CS51: prescale by 8 => 0.5us tick
     ICES5: input capture on rising edge
     OCR5A: 40000, 0.5us tick => 2ms period / 50hz freq for outbound
     fast PWM.
  */
  TCCR5A = _BV(WGM50) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS51) | _BV(ICES5);
  OCR5A  = 40000;

  /* OCR5B and OCR5C will be used by RCOutput_APM2. init to nil output */
  OCR5B  = 0xFFFF;
  OCR5C  = 0xFFFF;

  /* Enable input capture interrupt */
  TIMSK5 |= _BV(ICIE5);
}


/* private callback for input capture ISR */
ISR(TIMER5_CAPT_vect) {
  static uint16_t icr5_prev;
  static uint8_t  channel_ctr;

  const uint16_t icr5_current = ICR5;
  uint16_t pulse_width;
  if (icr5_current < icr5_prev) {
    /* ICR5 rolls over at TOP=40000 */
    pulse_width = icr5_current + 40000 - icr5_prev;
  } else {
    pulse_width = icr5_current - icr5_prev;
  }

  if (pulse_width > 8000) {
    /* sync pulse detected */
    channel_ctr = 0;
  } else {
    if (channel_ctr < AVR_RC_INPUT_NUM_CHANNELS) {
      _pulse_capt[channel_ctr] = (pulse_width >> 1);
      channel_ctr++;
      if (channel_ctr == AVR_RC_INPUT_NUM_CHANNELS) {
        _valid = AVR_RC_INPUT_NUM_CHANNELS;
      }
    }
  }
  icr5_prev = icr5_current;
}
