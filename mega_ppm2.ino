#include <Servo_Rc_PWM.h>
Servo servo_roll, servo_pitch;

// https://github.com/collin80/AltSoftSerial/blob/master/config/AltSoftSerial_Timers.h

volatile unsigned long Start_Pulse = 0;
volatile unsigned long Stop_Pulse = 0;
volatile unsigned long Pulse_Width = 0;
volatile byte PPM_Counter = 0;
volatile unsigned long PWM_RAW[8] = {2400, 2400, 2400, 2400, 2400, 2400, 2400, 2400};

volatile uint8_t _radio_status = 0;

void setup() {
  Serial.begin(9600);
  servo_roll.attach(46, 1200, 1800, 1500);
  servo_roll.writeMicroseconds(1500);
  servo_pitch.attach(12, 1200, 1800, 1500);
  servo_pitch.writeMicroseconds(1500);
  Init_PPM_PWM4();
}

void loop() {
  servo_roll.writeMicroseconds(limit(0));
  servo_pitch.writeMicroseconds(limit(0));
  Serial.println(channel(0));
  delay(10);
}


unsigned long channel(byte ch)
{
  return ((PWM_RAW[ch] + 800) / 2);
}

unsigned long limit(byte ch)
{
  return constrain(channel(ch), 1200, 1800);
}



void Init_PPM_PWM4(void)
{
  //pinMode(49, INPUT); // Arduino Mega PIN 49
  // (ICP4) PL0 giriş olarak ayarlayın, RC alıcı PPM çıkışı Arduino Mega PIN 49'u bağlayın
  DDRL |= ( 0 << PL0 ); // Set (ICP4) PL0 as an input, RC receiver PPM output connect Arduino Mega PIN 49

  // Timer4 kayıtlarını sıfırla.
  TCCR4A = 0x0;
  TCCR4B = 0x0;
  TCCR4C = 0x0;

  // Prescaler değeri 8 olarak ayarlandı, 0.5us çözünürlük sağlayacak
  // Sinyal yakalama RISING (yükselen kenar) olacak
  TCCR4A |= (1 << WGM41) | (1 << WGM40); // | (1 << COM4A1)
  TCCR4B |= (1 << WGM42) | (1 << WGM43) | (1 << CS41) | (1 << ICES4);

  OCR4A = 40000;
  OCR4B = 0xFFFF; // OCR kayıtlarını sıfır çıkış sinyalinde başlat
  OCR4C = 0xFFFF;
  ICR4 = 40000;
  TIMSK4 |= (1 << ICIE4); //Timer kesinti maskesi
  sei();
}


ISR(TIMER4_CAPT_vect)//interrupt.
{
  if (((1 << ICES4)& TCCR4B) >= 0x01)
  {
    if (Start_Pulse > Stop_Pulse) //Checking if the Stop Pulse overflow the register, if yes i normalize it.
    {
      Stop_Pulse += 40000; //Nomarlizing the stop pulse.
    }
    Pulse_Width = Stop_Pulse - Start_Pulse; //Calculating pulse
    if (Pulse_Width > 5000) //Verify if this is the sync pulse
    {
      PPM_Counter = 0; //If yes restart the counter
    }
    else
    {
      PWM_RAW[PPM_Counter] = Pulse_Width; //Saving pulse.
      PPM_Counter++;
    }
    Start_Pulse = ICR4;
    TCCR4B &= (~(1 << ICES4)); //Changing edge detector.
  }
  else
  {
    Stop_Pulse = ICR4; //Capturing time stop of the drop edge
    TCCR4B |= (1 << ICES4); //Changing edge detector.
    //TCCR4B &=(~(1<<ICES4));
  }
}
