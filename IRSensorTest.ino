/*
  IR sensor
*/
int motorPin1 = 2;//J3 on board
int motorPin2 = 7;
int motorPin3 = 5; //5 is a PWM pin
int motorPin4 = 8; // J4 on board
int IRPin = 4; //S4 on J2
int val; 

void setup() {
  Serial.begin(9600);
  pinMode(IRPin, INPUT); 
  val = 0;
}

void loop() {
  //low is nearby, high is far
  val = digitalRead(IRPin);
  Serial.println(val);
  if (val){
    //stop
    digitalWrite(motorPin1, LOW);//1 high 2 low is clockwise
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
  }
  else {
    //move
    digitalWrite(motorPin1, HIGH);//1 high 2 low is clockwise
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
  }
}
  
