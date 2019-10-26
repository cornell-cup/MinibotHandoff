#include <SPI.h>

#define PN532_IRQ   (9)   //set up the interrupt and reset
#define PN532_RESET (17)  
int val;
char buff [50];
volatile byte indx;
volatile boolean process;
int  interruptPin = 10;
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
  // put your setup code here, to run once:
  Serial.begin(115200);
  myservo.attach(6);//servo pin
  pinMode(MISO,OUTPUT); //init spi
  pinMode(3,OUTPUT);
  //Init DC pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(20,OUTPUT);
  pinMode(22,OUTPUT);
  
  //LEDs setup fir RFID
  pinMode(LED1,OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  
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

void moveForward() {
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

void loop() {
//
          digitalWrite(motorPin1, LOW);
          digitalWrite(motorPin2, LOW);
          digitalWrite(motorPin3, LOW);
          digitalWrite(motorPin4, LOW);  

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
          moveForward();
          delay(6000);
          break;
        case 'B' : //Backwards (back())
          digitalWrite(motorPin1, LOW);
          digitalWrite(motorPin2, HIGH);
          digitalWrite(motorPin3, LOW);
          digitalWrite(motorPin4, HIGH);
          delay(6000);
          break;
        case 'L' : //left
          digitalWrite(motorPin1, HIGH);
          digitalWrite(motorPin2, LOW);
          digitalWrite(motorPin3, LOW);
          digitalWrite(motorPin4, HIGH);
          delay(6000);
          break;
        case 'R' : //right
          digitalWrite(motorPin1, LOW);
          digitalWrite(motorPin2, HIGH);
          digitalWrite(motorPin3, HIGH);
          digitalWrite(motorPin4, LOW);
          delay(6000);
          break; 
        case 'S' : //stop
          digitalWrite(motorPin1, LOW);
          digitalWrite(motorPin2, LOW);
          digitalWrite(motorPin3, LOW);
          digitalWrite(motorPin4, LOW);
          delay(6000);
          break; 
        case 'T' : //Line Follow mode
          lineFollow();
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

  
}
