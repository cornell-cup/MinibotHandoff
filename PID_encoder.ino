#include <Encoder.h>
#include <Motor.h>
#include <SpeedControl.h>

#define RH_ENCODER_A 3 //Interrupt pin
#define RH_ENCODER_B 5
#define LH_ENCODER_A 2 //Interrupt pin
#define LH_ENCODER_B 4 //Marketed as 12 counts per revolution by polulu
int deltaT = 500000;
int countPerRev = 600;

Motor* motorL = new Motor( 9, 10 );
Encoder* encoderL = new Encoder( LH_ENCODER_A, LH_ENCODER_B, deltaT, countPerRev);
SpeedControl speedL = SpeedControl(motorL, encoderL);
double kP = 1.0;
double kI = .5;
double kD = .3;
double minSpeed = 40; 

void setup() {
  // put your setup code here, to run once:
  //motorL->setPWM(180);
  speedL.setGains( kP, kI, kD);
  speedL.setMinSpeed(minSpeed);
  speedL.speedSet(minSpeed);
  
  // initialize hardware interrupts
  attachInterrupt(0, leftEncoderEvent, CHANGE);
  
  //attachInterrupt(1, rightEncoderEvent, CHANGE);
  
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.print("Right Count: ");
 // Serial.println(rightCount);
  Serial.print("Left distance: ");
  Serial.println(encoderL->getDistance());
  Serial.print( "Left speed: ");
  Serial.println( encoderL->getSpeed());
  
  Serial.print( "Set Point: ");
  Serial.println( speedL.getSetPoint()); 
  Serial.print( "iTerm: ");
  Serial.println( speedL.getiTerm()); 
  
  speedL.adjustPWM();
  Serial.println("adjusted");
  
  Serial.println();
  delay(500);

}

void leftEncoderEvent() {
  encoderL->updateCount();
}
 
/*// encoder event for the interrupt call
void rightEncoderEvent() { //encoder.updateCount()
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}
*/

//Do PID stuff outside of interrupt



