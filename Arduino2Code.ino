#include <SPI.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_PN532.h>


Servo myservo;

#define PN532_IRQ   (9)   //set up the interrupt and reset
#define PN532_RESET (17)  
int val;
char buff [50];
volatile byte indx;
volatile boolean process;
int  interruptPin = 10;
int motorPin1 = 2;
int motorPin2 = 3;
int motorPin3 = 8;
int motorPin4 = 5;
//LEDs for RFID
int LED1 = 14;    //Green LED
int LED2 = 6;    //Yellow LED
int LED3 = 4;    //Red LED
//initialize the RFID
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

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
  //LEDs setup
  pinMode(LED1,OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  SPCR |= bit (SPE); // slave control register
  indx = 0; //buffer empty
  process = false;
  
  nfc.begin();
  
  pinMode(interruptPin,INPUT);
  int val = digitalRead(interruptPin);
  //if (!val)
    //Serial.println ("wtf");
  delay(1000);
  SPI.attachInterrupt();
 // attachInterrupt(digitalPinToInterrupt(interruptPin), tester, LOW); //enable interrupt
  //val = 1;
  //Serial.print(val);
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
  //Insert IR+US+DC code here
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
          
        case 'O': //object detection
         
         
        default:
          i++;
          break; 
        
      }
    }
  }

    //SPDR = data  //sends value to master via SPDR
    indx = 0; //reset button to zero
}
