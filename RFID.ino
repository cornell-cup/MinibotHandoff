//this is using I2C communication
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>


// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
//use J11 on the PCB
#define PN532_IRQ   (17)
#define PN532_RESET (9)  // Not connected by default on the NFC Shield
int LED1 = 14;    //Green LED
int LED2 = 6;    //Yellow LED
int LED3 = 4;    //Red LED


// Or use this line for a breakout or shield with an I2C connection:
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

//#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
// also change #define in Adafruit_PN532.cpp library file
//   #define Serial SerialUSB
//   #endif

// Ultrasonic Initialization
// defines pins numbers
//const int trigPin = 9;
//const int echoPin = 10;
// const int LED = 7;
// defines variables
//long duration;
//int distance;


void setup() {
  
  // Ultrasonic 
//  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
//  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(LED1,OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // RFID
//  #ifndef ESP8266
//    while (!Serial); // for Leonardo/Micro/Zero
//  #endif
  
  Serial.begin(9600);

  nfc.begin();

 uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);

  // configure board to read RFID tags
  nfc.SAMConfig();

  // Serial.println("Show us your card!");

}

void loop() {
  // RFID 
  Serial.println("Hello");
  boolean detector;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID from tag
  
  uint8_t obj1[] = {0xB9, 0xC3, 0xD, 0xF4}; //Green Tower
  uint8_t obj2[] = {0xB9, 0x12, 0xD, 0xF4}; //Yellow Tower
  uint8_t obj3[] = {0xC9, 0x82, 0xC, 0xF4};  //Red Tower
  uint8_t uidLength;

Serial.println("Sensor ready!");
// Ultrasonic
  /*
//  // Clears the trigPin
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//// Sets the trigPin on HIGH state for 10 micro seconds
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//// Reads the echoPin, returns the sound wave travel time in microseconds
//  duration = pulseIn(echoPin, HIGH);
//// Calculating the distance
//  distance= duration*0.034/2;
//  Serial.print("Distance: ");
//  Serial.println(distance);


  
  if(distance<=20){
   //flash the light a bunch of time to indicate that an object is close
    for(int8_t x=0; x>=-20; x--){

      delay(100);     
      delay(100);
    }

    delay(100);
   // Serial.println("Object detected!");
    // Serial.println("Moving closer....");

    while(distance>=2){

      distance = (pulseIn(echoPin, HIGH))*0.034/2;
      Serial.print("Distance: ");
      Serial.println(distance);
      reading the tags*/
      detector = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
      Serial.println(detector);
      
//      if(detector){
//        break;
//      }
   // } // while loop closed
    
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
