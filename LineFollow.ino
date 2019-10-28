#include <Wire.h>
#include "Adafruit_TCS34725.h"
int Rmotor= 2;
int RmotorPWM= 3;
int Lmotor= 8;
int LmotorPWM= 5;

int left_Q = A3;
int right_Q = A5;

int left_calib;
int right_calib;

int left_threshold;
int right_threshold;


int left_read;
int right_read;

//CONSTANT TO GET THE REFLECTANCE OF A LINE
int line_refl=980;

int left_line=false;
int right_line=false;

void setup() {
 
  
  // put your setup code here, to run once:
 Serial.begin(9600);
  pinMode(Rmotor,OUTPUT);
  pinMode(Lmotor,OUTPUT);
  pinMode(RmotorPWM,OUTPUT);
  pinMode(LmotorPWM, OUTPUT);
  
  pinMode(left_Q, INPUT);
  pinMode(right_Q, INPUT);

  left_calib=analogRead(left_Q);
  right_calib=analogRead(right_Q);

  left_threshold = abs((left_calib-line_refl)/2);
  right_threshold = abs((right_calib-line_refl)/2);
}

void loop() {
    readSensors();
    if(!left_line && !right_line){
      drive_forward();
      Serial.println("forward no line");
    }
   else if(!left_line && right_line){
      veer_left();
      Serial.println("veer left");
    }
    else if(left_line && !right_line){
      veer_right();
      Serial.println("veer right");
    }
    else {
      drive_forward();
      Serial.println("forward else");
    }
}

void readSensors(){
  left_line= (((analogRead(left_Q)-line_refl)<left_threshold) ||(line_refl-analogRead(left_Q))>left_threshold);
  right_line=((analogRead(right_Q)-line_refl<right_threshold) || (line_refl-analogRead(right_Q))>right_threshold);
}
void drive_forward() {
  digitalWrite(Rmotor,HIGH); 
  digitalWrite(Lmotor,HIGH);
  analogWrite(RmotorPWM,100);
  analogWrite(LmotorPWM, 100);
}
void stop(){
  digitalWrite(Rmotor,LOW);
  digitalWrite(Lmotor,LOW);
  analogWrite(RmotorPWM,0);
  analogWrite(LmotorPWM, 0);
}
void veer_left(){
  digitalWrite(Rmotor,HIGH);
  digitalWrite(Lmotor,HIGH);
  analogWrite(RmotorPWM,150);
  analogWrite(LmotorPWM, 50);
}

void veer_right(){
  digitalWrite(Rmotor,HIGH);
  digitalWrite(Lmotor,HIGH);
  analogWrite(RmotorPWM,50);
  analogWrite(LmotorPWM, 150);
}
