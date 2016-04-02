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

#define MICRO_SERVOPIN 5  //pin that the micro servo will be attached to
#define LARGE_SERVOPIN 6  //pin that the large servo will be attached to

Servo microServo;   //create an servo object for the 9g FT-FS90MG micro servo
Servo largeServo;   //create an servo object for the RobotGeek 180 degree serco

const int timeDelay = 3000;
const int microDelay = 5;
const int gripOpen = 0;
const int gripClosed = 130;
const int wristDown = 150;
const int wristUp = 0;
const int wristLevel = 90;
const int gripGrab = 85;
const int wristPickup = 20;
const int wristLower = 110;

//setup servo objects and set initial position
void setup()
{ 
  microServo.attach(MICRO_SERVOPIN);
  microServo.write(gripOpen);    // sets the servo position to 150 degress, positioning the servo for the gripper closed
  largeServo.attach(LARGE_SERVOPIN);
  largeServo.write(wristLevel);    // sets the servo position to 90 degress, centered
  Serial.begin(9600);
}
 
//repeat test process 
void loop()
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
  

  //Test gripper servo
  /*microServo.write(gripOpen);    //set gripper to fully open
  Serial.println("Gripper Open");
  delay(timeDelay);            //wait 
  microServo.write(gripClosed);    //set gripper to fully closed
  Serial.println("Gripper Closed");
  delay(timeDelay);            //wait 
  */
  //Test wrist servo
  /*largeServo.write(wristDown);   //Lower wrist 
  Serial.println("Wrist Down");
  delay(timeDelay);            //wait 
  largeServo.write(wristUp);   //Raise wrist 
  Serial.println("Wrist Up");
  delay(timeDelay);            //wait 
  largeServo.write(wristLevel);   //Level out wrist
  Serial.println("Wrist Level");
  delay(timeDelay);            //wait 
  */
  
  //microServo.write(gripClosed);  //set gripper to 130 degrees = fully closed
  //Serial.println("Gripper Closed");
  //delay(timeDelay);            //wait 3 seconds
  //largeServo.write(wristDown);    //set wrist servo to 0 degrees = wrist down
  //Serial.println("Wrist Down");
  //delay(timeDelay);            //wait 3 seconds
  //largeServo.write(wristUp);  //set wrist servo to 180 degrees = wrist up
  //Serial.println("Wrist Up");
  //delay(timeDelay);            //wait 3 seconds
  //largeServo.write(wristLevel);   //set wrist servo to 90 degrees = wrist level
  //Serial.println("Wrist Level");
  //delay(timeDelay);            //wait 3 seconds
  //Serial.println("Starting Over");
}
 
 
 
 
 
