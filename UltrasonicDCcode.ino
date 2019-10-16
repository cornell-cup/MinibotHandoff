#include <Servo.h>
//Servo myservo;
int motorPin1 = 2;//J3 on board
int motorPin2 = 7;
int motorPin3 = 5; //5 is a PWM pin
int motorPin4 = 8; // J4 on board
int val;
int trigPin = 9; //J8 on board
int echoPin = A3; //this is the ADC pin
long duration,cm;
void setup() {
 Serial.begin(9600);
//myservo.attach(5);
pinMode(motorPin1, OUTPUT);
pinMode(motorPin2, OUTPUT);
pinMode(motorPin3, OUTPUT);
pinMode(motorPin4, OUTPUT);
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
}
void loop() {
 //this is the motor code
//this is the ultrasonic code
 digitalWrite(trigPin,LOW);
 delayMicroseconds(30);
 digitalWrite(trigPin,HIGH);
 delayMicroseconds(30);
 pinMode(echoPin,INPUT);
 duration = pulseIn(echoPin,HIGH);
 //convert the time into distance: 29ms per cm
 cm = (duration/2)/29.1;
 Serial.println(cm);
 if(cm < 10){
    digitalWrite(motorPin1, LOW);//1 high 2 low is clockwise
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
 }
 else{
    digitalWrite(motorPin1, HIGH);//1 high 2 low is clockwise
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
 }
 //servo code
 //myservo.write(140);
 //delay(300);
 //myservo.write(100);
 delay(300);
}
