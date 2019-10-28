#include <SPI.h>
#include <elapsedMillis.h>

elapsedMillis timeElapsed;

int encoder0PinA = A1; //J3 motor on board
//int encoder0PinB = 4;
int encoder0Pos = 0;
int encoder0PinALast = LOW;

int encoder1PinA = A2; //J4 motor on board
//int encoder1PinB = 8;
int encoder1Pos = 0;
int encoder1PinALast = LOW;

int encoderCountpRev = 360;
int setpoint = 200; //(degrees/sec) 
double Integral0 = 0;
double Integral1 = 0;
int n = LOW;
int m = LOW;

int motor0pin1 = 2; // J3 on Board
int motor0pin2 = 3; //pwm pin
int pwm0 = 123; //123
int digital0 = 1; //0?

int motor1pin1 = 8; // J4 on Board
int motor1pin2 = 5; //pwm pin
int pwm1 = 123; //123
int digital1 = 1; //0?

int encoder0PrevCount = 0;
int lastSpeed0 = 0;
int encoder1PrevCount = 0;
int lastSpeed1 = 0;

double timeSec = .5;

double kP = 0.20;//0.20 or .15
double kI = 0.01;//0.01 or .05
double kD = 0.01;//0.01 or .01

int val;
int test;
char buff [50];
volatile byte indx;
volatile boolean process;
int  interruptPin = 10;

int IRPin = 4; //S4 on J5
int in; 
int trigPin = 9; //J10 on board
int echoPin = A3; //this is the ADC pin
long duration,cm;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode (encoder0PinA, INPUT);
  //  pinMode (encoder0PinB, INPUT);
  pinMode (motor0pin1, OUTPUT);
  pinMode (motor0pin2, OUTPUT);

  pinMode (encoder1PinA, INPUT);
  //pinMode (encoder1PinB, INPUT);
  pinMode (motor1pin1, OUTPUT);
  pinMode (motor1pin2, OUTPUT);

  pinMode(MISO,OUTPUT); //init spi
  pinMode(3,OUTPUT);

  pinMode(20,OUTPUT);
  pinMode(22,OUTPUT);
  

  
  SPCR |= bit (SPE); // slave control register
  indx = 0; //buffer empty
  process = false;

  
  pinMode(interruptPin,INPUT);
  int val = digitalRead(interruptPin);

  delay(1000);
  SPI.attachInterrupt();
 // attachInterrupt(digitalPinToInterrupt(interruptPin), tester, LOW); //enable interrupt
  //val = 1;
  //Serial.print(val);
  
  pinMode(IRPin, INPUT); 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  test = 0;
  }

ISR (SPI_STC_vect) //SPI Interrupt Service Routine
//void tester() 
{
 // digitalWrite(3,1);
  //Serial.println("work");
  byte c = SPDR; //read byte from SPI data register
  if (indx < sizeof (buff) - 1) 
    buff [indx++] = c;// save data in the next index in the array buff
 // if (c == '\r') //check for the end of the word
    process = true;
  
}  

//Adjust PWM FOR PID CODE
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

void PID() {
  Serial.println(digital0);
  Serial.println(digital1);
  
  if (digital0 == 1)
    digitalWrite( motor0pin1, HIGH);
  else digitalWrite( motor0pin1, LOW);
  analogWrite( motor0pin2, pwm0);

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

void moveForward() {
  Serial.println("forward code");
    //low is nearby, high is far
  in = digitalRead(IRPin);
  Serial.println(in);
  digitalWrite(trigPin,LOW);
  delayMicroseconds(30);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(30);
  pinMode(echoPin,INPUT);
  duration = pulseIn(echoPin,HIGH);
  //convert the time into distance: 29ms per cm
  cm = (duration/2)/29.1;
  Serial.println(cm);
  if (in == 1 || cm < 10){
    //stop
    digitalWrite(motor0pin2, HIGH);//1 high 2 low is clockwise
    digitalWrite(motor0pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor1pin1, LOW);
    delay(400);
    digitalWrite(motor0pin2, LOW);
    digitalWrite(motor1pin2, LOW);
    delay(1000);
    Serial.println("stop");
  }
  else {
    //move
   // PID();
    digitalWrite(motor0pin2, LOW);//1 high 2 low is clockwise
    digitalWrite(motor0pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor1pin1, HIGH);
    Serial.println("move");
  }
}

void LineFollow() {
  
}

void loop() {
  Serial.print(buff);
  Serial.println(" test");
 // moveForward();


  if (process) {
    buff[indx] = 0;
    process = false; //reset flag
//    digitalWrite(3,0);
 //   Serial.println("work");
    Serial.print(buff); //print to serial monitor
    int i = 0;
    if (i < sizeof(buff)) {
      switch(buff[i]) {
        case 'F' : //fwd
          Serial.println("moving forward");
          test++;
          moveForward();
          //delay(6000);
          break;
        case 'B' : //Backwards (back())
          Serial.println("back");
          digitalWrite(motor0pin2, HIGH);
          digitalWrite(motor0pin1, LOW);
          digitalWrite(motor1pin2, HIGH);
          digitalWrite(motor1pin1, LOW);
         // delay(6000);
          break;
        case 'L' : //left
          Serial.println("Left");
          digitalWrite(motor0pin2, LOW);
          digitalWrite(motor0pin1, HIGH);
          digitalWrite(motor1pin2, HIGH);
          digitalWrite(motor1pin1, LOW);
          //delay(6000);
          break;
        case 'R' : //right
          digitalWrite(motor0pin2, HIGH);
          digitalWrite(motor0pin1, LOW);
          digitalWrite(motor1pin2, LOW);
          digitalWrite(motor1pin1, HIGH);
          //delay(6000);
          break; 
        case 'S' : //stop
          digitalWrite(motor0pin2, LOW);
          digitalWrite(motor0pin1, LOW);
          digitalWrite(motor1pin2, LOW);
          digitalWrite(motor1pin1, LOW);
          //delay(6000);
          break; 
        case 'T' : //Line Follow mode
          LineFollow();
          break; 
        
         
        default:
          i++;
          break; 
        
      }
    }
  }

    //SPDR = data  //sends value to master via SPDR
    indx = 0; //reset button to zero
}

  
