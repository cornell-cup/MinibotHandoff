/*
  IR sensor + Ultrasonic
*/
int motorPin1 = 2;//J6 on board
int motorPin2 = 3;
int motorPin3 = 5; //5 is a PWM pin
int motorPin4 = 8; // J7 on board
int IRPin = 4; //S4 on J5
int in; 
int val;
int trigPin = 9; //J10 on board
int echoPin = A3; //this is the ADC pin
long duration,cm;

void setup() {
  Serial.begin(9600);
  pinMode(IRPin, INPUT); 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
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
  //Serial.println(cm);
  if (in == 1 || cm < 10){
    //stop
    digitalWrite(motorPin1, LOW);//1 high 2 low is clockwise
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    delay(300);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    Serial.println("stop");
  }
  else {
    //move
    digitalWrite(motorPin1, HIGH);//1 high 2 low is clockwise
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    Serial.println("move");
  }
  
}
  
