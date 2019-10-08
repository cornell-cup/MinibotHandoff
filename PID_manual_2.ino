#include <elapsedMillis.h>

elapsedMillis timeElapsed;

int val;
int encoder0PinA = 12; //18; //interrupt pin
//int encoder0PinB = 4;
int encoder0Pos = 0;
int encoder0PinALast = LOW;

int encoder1PinA = 10; //19;//interrupt pin
//int encoder1PinB = 8;
int encoder1Pos = 0;
int encoder1PinALast = LOW;

int encoderCountpRev = 360;
int setpoint = 200; //(degrees/sec)
double Integral0 = 0;
double Integral1 = 0;
int n = LOW;
int m = LOW;

int motor0pin1 = 2; //for uno change to 10
int motor0pin2 = 3; // for uno change to 9
int pwm0 = 123; //123
int digital0 = 1;

int motor1pin1 = 4;
int motor1pin2 = 5;
int pwm1 = 123; //123
int digital1 = 0;

int encoder0PrevCount = 0;
int lastSpeed0 = 0;
int encoder1PrevCount = 0;
int lastSpeed1 = 0;

double timeSec = .5;

double kP = 0.20; //.15
double kI = 0.01; //.05
double kD = 0.01; //.01

void setup() {
  pinMode (encoder0PinA, INPUT);
  //  pinMode (encoder0PinB, INPUT);
  pinMode (motor0pin1, OUTPUT);
  pinMode (motor0pin2, OUTPUT);

  pinMode (encoder1PinA, INPUT);
  //pinMode (encoder1PinB, INPUT);
  pinMode (motor1pin1, OUTPUT);
  pinMode (motor1pin2, OUTPUT);

  // attachInterrupt(0, countInterrupt0, CHANGE);
  // attachInterrupt(1, countInterrupt1, CHANGE);
  Serial.begin (9600);
}

void loop() {
  if (digital0 == 1)
    digitalWrite( motor0pin1, HIGH);
  else digitalWrite( motor0pin1, LOW);
  analogWrite( motor0pin2, pwm0);

  digital1 = 0;
  if (digital1 == 1)
    digitalWrite( motor1pin1, HIGH);
  else digitalWrite( motor1pin1, LOW);
  analogWrite( motor1pin2, pwm1);


  timeElapsed = 0;
  //calculateSpeed();
  while ( timeElapsed < 500 ) {

    n = digitalRead(encoder0PinA);
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
      encoder0Pos++;
    }
    //Serial.print (encoder0Pos);
    //Serial.print ("/");

    encoder0PinALast = n;

    m = digitalRead(encoder1PinA);
    if ((encoder1PinALast == LOW) && (m == HIGH)) {
      encoder1Pos++;
    }
    // Serial.print (encoder1Pos);
    //Serial.print ("/");

    encoder1PinALast = m;

  }

  //unsigned long CurrentTime = millis();
  //unsigned long ElapsedTime = CurrentTime - StartTime;
  timeSec = 1.0 ;//(double)( ElapsedTime * .001);
  Serial.print( "Time: ");
  Serial.println(timeSec); // time needs to be fixed
  adjustPWM();
  Serial.println(" ");

}

void adjustPWM() {
  int speedNow0 = calculateSpeed0();
  int error0 = setpoint - speedNow0;
  double dError0 = ((double)speedNow0 - (double)lastSpeed0) / timeSec;
  Integral0 += (double) error0;

  int speedNow1 = calculateSpeed1();
  int error1 = setpoint - speedNow1;
  double dError1 = ((double)speedNow1 - (double)lastSpeed1) / timeSec;
  Integral1 += (double) error1;

  if (Integral0 > 255) Integral0 = 255;
  else if (Integral0 < 0) Integral0 = 0;
  Serial.print("Integral0: ");
  Serial.println( Integral0);

  if (Integral1 > 255) Integral1 = 255;
  else if (Integral1 < 0) Integral1 = 0;
  Serial.print("Integral1: ");
  Serial.println( Integral1);

  int adjust0 = (kP * (double)error0) + kI * Integral0 + kD * dError0;
  int adjust1 = (kP * (double)error1) + kI * Integral1 + kD * dError1;

  if (digital0 == 0) pwm0 -= adjust0;
  else pwm0 += adjust0;

  if (digital1 == 0) pwm1 += adjust1;    
  else pwm1 -= adjust1;

  if (pwm0 > 255) pwm0 = 255;
  else if (pwm0 < 0) pwm0 = 0;
  
  if (pwm1 > 255) pwm1 = 255;
  else if (pwm1 < 0) pwm1 = 0;

  lastSpeed0 = speedNow0;
  lastSpeed1 = speedNow1;

  Serial.print("adjustment0: ");
  Serial.println( adjust0);
  Serial.print("PWM0: ");
  Serial.println( pwm0 );

  Serial.print("adjustment1: ");
  Serial.println( adjust1);
  Serial.print("PWM1: ");
  Serial.println( pwm1 );
}


int calculateSpeed0() {
  int speedDetect = (encoder0Pos - encoder0PrevCount) / timeSec;
  Serial.print( encoder0Pos );
  Serial.print("  ");
  Serial.println( encoder0PrevCount);
  encoder0PrevCount = encoder0Pos;
  Serial.print( "Speed: ");
  Serial.println( speedDetect);
  return speedDetect;
}

int calculateSpeed1() {
  int speedDetect = (encoder1Pos - encoder1PrevCount) / timeSec;
  Serial.print( encoder1Pos);
  Serial.print("  ");
  Serial.println( encoder1PrevCount);
  encoder1PrevCount = encoder1Pos;
  Serial.print( "Speed: ");
  Serial.println( speedDetect);
  return speedDetect;
}

/*void countInterrupt0() {
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
    //Serial.print (encoder0Pos);
    //Serial.print ("/");
  }
  encoder0PinALast = n;
  }

  void countInterrupt1() {
  n = digitalRead(encoder1PinA);
  if ((encoder1PinALast == LOW) && (n == HIGH)) {
    if (digitalRead(encoder1PinB) == LOW) {
      encoder1Pos--;
    } else {
      encoder1Pos++;
    }
    // Serial.print (encoder1Pos);
    //Serial.print ("/");
  }
  encoder1PinALast = n;
  }
*/

