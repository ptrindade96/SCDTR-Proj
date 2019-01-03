#define LED 3
#define SENSOR0 0
#define MAX_INTENSITY 255
#define STEP_FRAC 0.2
#define STEP_TIME 1000000

long unsigned int Time_0;
long unsigned int Time_1;
long unsigned int Time;
int climb = 1;
int intensity = 0;
int read0 = 0;
float mean = 0;

/**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setPwmFrequency(LED,1);
  pinMode(LED, OUTPUT);
//  delay(2000);
  Time_0 = micros();
  Time_1 = Time_0;
}

void loop(){
  Time = micros();
  analogWrite(LED,intensity);


  if(Time-Time_0 > STEP_TIME){
    Time_0 = micros();
    intensity += MAX_INTENSITY*STEP_FRAC*climb;
    
    if(intensity >= MAX_INTENSITY){
      climb = -1;
      intensity = MAX_INTENSITY;  }
      
    if(intensity <= 0){
      climb = 1;
      intensity = 0;  }
    analogWrite(LED,intensity);  
  }

  read0 = analogRead(SENSOR0);
  
  Serial.print(Time-Time_1);
  Serial.print(",");
  Serial.print(intensity);
  Serial.print(",");
  Serial.println(read0);
}
