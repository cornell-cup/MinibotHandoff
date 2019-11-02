#include <SPI.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_PN532.h>


Servo myservo;

#define PN532_IRQ   (17)   //set up the interrupt and reset
#define PN532_RESET (9)  
int val;
char buff [50];
volatile byte indx;
volatile boolean process;
int  interruptPin = 10;
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

  pinMode(20,OUTPUT);
  pinMode(22,OUTPUT);
  
  //LEDs setup fir RFID
  pinMode(LED1,OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  
  SPCR |= bit (SPE); // slave control register
  indx = 0; //buffer empty
  process = false;
  
  //RFID reader
  nfc.begin(); 
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  // If find a board, print out the version
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);
  // configure board to read RFID tags
  nfc.SAMConfig();
  
  pinMode(interruptPin,INPUT);
  int val = digitalRead(interruptPin);

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


  if (process) {
    buff[indx] = 0;
    process = false; //reset flag
//    digitalWrite(3,0);
 //   Serial.println("work");
    Serial.print(buff); //print to serial monitor
    int i = 0;
    if (i < sizeof(buff)) {
      switch(buff[i]) {         
        case 'O': //object detection
          RFID();         
        default:
          i++;
          break; 
      }
    }
  }

    //SPDR = data  //sends value to master via SPDR
    indx = 0; //reset button to zero
}

void RFID(){
  // RFID 
  Serial.println("Hello");
  boolean detector;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID from tag
  uint8_t obj1[] = {0xE9, 0xDF, 0xE, 0xF4}; //the tag on the LED panel
  uint8_t obj2[] = {0x79, 0x5C, 0xE, 0xF4}; //the tag on the micrstickII
  uint8_t obj3[] = {0x1D, 0xEC, 0xB7, 0xC3};  //for the blue key tags
  uint8_t uidLength;

Serial.println("Sensor ready!");
  detector = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
      
      if(detector){
      Serial.println("Found a tag!");
      Serial.println("This is .....");
      if(memcmp(obj1, uid, 4) == 0){
        Serial.println("Object 1");
        LED_Blink_3(LED1);
        delay(1000);
      }
      
      else if (memcmp(obj2, uid, 4) == 0){
        Serial.println("Object 2");
        LED_Blink_3(LED2);
        delay(1000);
      }

      else if(memcmp(obj3, uid, 4) == 0){
        Serial.println("Object 3");
        LED_Blink_3(LED3);
        delay(1000); 
      }
      
      else{
        Serial.println("Not in database");
        delay(1000);
      }
    
      delay(1000); 
  
    }
    else{
      Serial.println("No Objects in Range");    
    }
  
}

void LED_Blink_3(int LED){
    int counter = 3;
    while (counter>0,counter--){
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED,LOW);
    delay(1000);
    }
  }
