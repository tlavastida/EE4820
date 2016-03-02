/***********************************************************************************
 *     -(] [)-          RobotGeek Gripper  
 *   / /     \ \              Automatic Test
 *  | |       | |
 *   \ \_____/ /
 *    |  ___  |
 *    |_|___|_|
 *    | _____ |
 *     |     |
 *     |     |
 *     |_____|
 *
 *  The following sketch will move each servo in the RobotGeek Gripper. It will repeat the following cycle
 * 
 *  1)Open gripper, wait 3 seconds
 *  2)Close Gripper, wait 3 seconds
 *  3)Lower Wrist Servo ~ 90 degrees, wait 3 seconds
 *  4)Raise Wrist Servo ~ 90 degrees, wait 3 seconds
 *  5)Level Wrist Servo, wait 3 seconds
 *
 *  First it will open and close the gripper
 *  Then it will raise and lower the wrist servo.
 *  
 *    
 *  Wiring
 *
 *  DIO 9 - Blue Micro Servo (FT-FS90MG)      Orange - 'S' Brown -'G'
 *  DIO 11 - Black RobotGeek 180 Degree Servo  White -'S' Black -'G'
 *  
 *  Use an external power supply and set the jumper for pins 9/10/11 to 'VIN'
 *   
 *  For more information and wiring diagrams see
 *  http://learn.trossenrobotics.com/30-robotgeek-getting-started-guides/dev-kits/55-robotgeek-gripper-kit-tips
 *
 ***********************************************************************************/
#include <Servo.h>   //include the servo library to control the RobotGeek Servos

#define MICRO_SERVOPIN 9  //pin that the micro servo will be attached to
#define LARGE_SERVOPIN 11  //pin that the large servo will be attached to

Servo microServo;   //create an servo object for the 9g FT-FS90MG micro servo
Servo largeServo;   //create an servo object for the RobotGeek 180 degree serco

const int timeDelay = 500;
const int microDelay = 5;
const int gripOpen = 0;
const int gripClosed = 130;
const int wristDown = 150;
const int wristUp = 0;
const int wristLevel = 90;
const int gripGrab = 84;
const int wristPickup = 10;
const int wristLower = 110;

//declare pins for Infrared sensors
const int analogPinIR1 = A0;
const int analogPinIR2 = A7;

//Variables for IR sensors
long analogValue,distanceValue;
long distanceIR1,distanceIR2;  
//int loopControl = 0;

//setup servo objects and set initial position
void setup()
{ 
  microServo.attach(MICRO_SERVOPIN);
  microServo.write(130);    // sets the servo position to 150 degress, positioning the servo for the gripper closed
  largeServo.attach(LARGE_SERVOPIN);
  largeServo.write(90);    // sets the servo position to 90 degress, centered
  //pinMode(analogPinIR1,OUTPUT);   //Commented out because the reads were interfering with one another
  //pinMode(analogPinIR2,OUTPUT);
  Serial.begin(9600);
}
 
//repeat test process 
void loop()
{
  //while (loopControl < 1)
  //{
    distanceIR1 = checkIR(analogPinIR1); 
    delay(100);
    distanceIR2 = checkIR(analogPinIR2); 
    delay(100);
    if ((distanceIR1 == 4)&&(distanceIR2 == 4))
    {
      grabVictim();
    }
    delay(3000);
    //loopControl = loopControl + 1;
  //}
}

int checkIR (int pinNumIR)
{
  analogValue = analogRead(pinNumIR);     // Read current analog value from pin
  distanceValue = (4000/analogValue)-(42/100);
  Serial.print(analogValue);
  Serial.println();
  return distanceValue;         //Should be cm I believe
}

void grabVictim()
{
  //Pickup victim
  microServo.write(gripOpen);    //set gripper to 0 degrees = fully open
  Serial.println("Gripper Open");
  delay(timeDelay);            //wait 3 seconds
  largeServo.write(wristLower);   //Lower wrist to grab victim
  Serial.println("Wrist Level");
  delay(timeDelay);            //wait 3 seconds
  microServo.write(gripGrab);    //set gripper to grab victim
  Serial.println("Grab Victim");
  delay(timeDelay);            //wait 3 seconds
  for (int i=wristLower;i>wristPickup;i=i-1)
  {
    largeServo.write(i);   //Pickup victim
    delay(microDelay);
  }
  Serial.println("Wrist Up");
  delay(timeDelay);            //wait 3 seconds
  for (int i=wristPickup;i<wristLower;i=i+1)
  {
    largeServo.write(i);   //Lower wrist to drop victim
    delay(microDelay);
  }
  Serial.println("Wrist lowered");
  delay(timeDelay);            //wait 3 seconds
  microServo.write(gripOpen);    //set gripper to 0 degrees = fully open
  Serial.println("Gripper Open");
  delay(timeDelay);            //wait 3 seconds
} 
 
 
 
