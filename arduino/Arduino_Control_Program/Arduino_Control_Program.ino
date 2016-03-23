/************************************************************************************
 *  Wiring for RobotGeek Gripper
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
/************************************************************************************
*  References: 
* 1) http://learn.parallax.com/KickStart/28015 for Ultrasonic example code
* 2) http://www.instructables.com/id/Get-started-with-distance-sensors-and-Arduino/?ALLSTEPS for example code for IR/US
* 3) http://www.robotshop.com/blog/en/arduino-5-minute-tutorials-lesson-4-ir-distance-sensor-push-button-2-3637 for IR example code
* 4) http://www.robotoid.com/appnotes/circuits-quad-encoding.html
* 5) http://learn.robotgeek.com/getting-started/30-dev-kits/55-robotgeek-gripper-getting-started-guide.html
* 6) http://learn.trossenrobotics.com/30-robotgeek-getting-started-guides/dev-kits/55-robotgeek-gripper-kit-tips for gripper sample code
************************************************************************************/
#include <Servo.h>   //include the servo library to control the RobotGeek Servos
#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

#define STOP 0  
#define SLOW 40     
#define SPEED 80

#define MICRO_SERVOPIN 9    //pin that the micro servo will be attached to
#define LARGE_SERVOPIN 11   //pin that the large servo will be attached to

Servo microServo;   //create an servo object for the 9g FT-FS90MG micro servo
Servo largeServo;   //create an servo object for the RobotGeek 180 degree serco

//PIN ASSIGNMENTS
//declare pins for Infrared sensors
const int analogPinIR_L = A1;     
const int analogPinIR_R = A2;    

//declare pins for Ultrasonic sensors
const int digPinUS_L = 24;
const int digPinUS_R = 26;
const int digPinUS_F = 28;

//declare pins for LS7184 chips
const int clk1 = 25;          //interrupt will be attached for this (chip1)
const int direction1 = 27;        //requires digital pin (chip1)
const int clk2 = 29;          //interrupt will be attached for this (chip2)
const int direction2 = 31;        //requires digital pin (chip2)

//Time delays for gripper movement
const int timeDelay = 20;
const int microDelay = 5;

//Angle values for gripper
const int gripOpen = 0;
const int gripClosed = 130;
const int gripGrab = 84;

//Angle values for wrist
const int wristDown = 150;
const int wristUp = 0;
const int wristLevel = 90;
const int wristPickup = 10;
const int wristLower = 110;

//distance Thresholds
const int pickupVictim = 67;                //pickup victim at this distance (mm)
const int victimAhead = 110;                //victim is ahead of robot, start aligning (mm)
const int pickupTolerance = 2;              //use threshold - measurement and check tolerance (mm)
const int dangerZone = pickupVictim-pickupTolerance;    //Stop Immediately

//Variables for IR sensors
long analogValue,distanceValue;
volatile long distanceIR_L,distanceIR_R;  
int threshold = 5;                    //used to check if IR sensors approx. equal (mm)

//Variables for US sensors
long pulseWidth,cm; 
volatile long distanceUS_L,distanceUS_R,distanceUS_F; 

//Variables for LS7184 chips
volatile int count1 = 0;                //tick count chip 1
volatile int count2 = 0;                //tick count chip 2

//Variables for encoders 
volatile int distance = 0;              //distance to travel in cm, given by Pi?
const int convFactor = 13;              //counts per cm, 13 for cm 33 for in at 1x resolution
volatile int totalTicks = distance*convFactor;  //total number of ticks to travel
const int turnTickCount = 239;          //number of ticks for a turn


volatile bool targetAcquired = FALSE;     //if target is in tow or not


//setup servo objects and set initial position
void setup()
{ 
  microServo.attach(MICRO_SERVOPIN);
  microServo.write(gripClosed);    // sets the servo position to 150 degress, positioning the servo for the gripper closed
  largeServo.attach(LARGE_SERVOPIN);
  largeServo.write(wristPickup);    // sets the servo position to 90 degress, centered
  pinMode(clk1,INPUT);
  pinMode(direction1,INPUT);
  attachInterrupt(digitalPinToInterrupt(clk1),encoderInterrupt1,RISING);
  pinMode(clk2,INPUT);
  pinMode(direction2,INPUT);
  attachInterrupt(digitalPinToInterrupt(clk2),encoderInterrupt2,RISING);
  Serial.begin(9600);
  md.init();
}
 
//repeat test process 
void loop()
{
  
}

int checkIR (int pinNumIR)
{
  analogValue = analogRead(pinNumIR);                 //Read current analog value from pin
  distanceValue = (400000-(42*analogValue))/(10*analogValue);   //conversion based on one from ref 
  return distanceValue;                         //mm
}

void grabVictim()
{
  //Pickup victim
  microServo.write(gripOpen);           //set gripper to fully open
  delay(timeDelay);                     //wait 
  largeServo.write(wristLower);           //Lower wrist to grab victim
  delay(timeDelay);                     //wait 
  microServo.write(gripGrab);           //set gripper to grab victim
  delay(timeDelay);                     //wait 
  for (int i=wristLower;i>wristPickup;i=i-1)
  {
    largeServo.write(i);              //Pickup victim
    delay(microDelay);
  }
} 

void dropVictim()
{
  //Drop victim
  for (int i=wristPickup;i<wristLower;i=i+1)
  {
    largeServo.write(i);              //Lower wrist to drop victim
    delay(microDelay);
  }
  delay(timeDelay);                     //wait 
  microServo.write(gripOpen);           //set gripper to 0 degrees = fully open
  delay(timeDelay);                     //wait
} 

void raiseGripper()
{
  for (int i=wristLower;i>wristPickup;i=i-1)
  {
    largeServo.write(i);   
    delay(microDelay);
  }
  microServo.write(gripClosed);    
}

//Checks US sensors - Code based on ref 1,2
long checkUS (int pinNumUS)
{
  digitalWrite(pinNumUS,LOW);               // make sure pin is low before pulsing
  pinMode(pinNumUS,OUTPUT);               // set up pin to initiate pulse
  delayMicroseconds(2);                   // for 2 microseconds 
  digitalWrite(pinNumUS,HIGH);              // Start pulse
  delayMicroseconds(5);                   // for 5 microseconds
  digitalWrite(pinNumUS,LOW);               // set pin back to low to ready for return pulse
  pinMode(pinNumUS,INPUT);                // change pin to Input mode for return pulse
  pulseWidth = pulseIn(pinNumUS,HIGH,18500);    // wait for return pulse. Timeout after 18.5 milliseconds
  cm = pulseWidth/88;                     // Convert to centimeters, use 58 for Mega, 53 for Due, jk it's 88 for Due
  return cm;
}
 
 //Interrupt routine for LS7184 chip 1 - Code based on ref 4
void encoderInterrupt1()
{
  count1 = digitalRead(direction1) ? count1 + 1: count1 - 1;
  flag1 = true;
}

//Interrupt routine for LS7184 chip 2 - Code based on ref 4
void encoderInterrupt2()
{
  count2 = digitalRead(direction2) ? count2 + 1: count2 - 1;
  flag2 = true;
}

void checkSensors()
{
  distanceIR_L = checkIR(analogPinIR_L);
  distanceIR_R = checkIR(analogPinIR_R);
  distanceUS_L = checkUS(digPinUS_L);
  distanceUS_R = checkUS(digPinUS_R);
  distanceUS_F = checkUS(digPinUS_F);
}

void printSensorValues()
{
  Serial.print("IR_L:");
  Serial.print(IR_L,DEC);
  Serial.print(",");
  Serial.print("IR_R:");
  Serial.print(IR_R,DEC);
  Serial.print(",");
  Serial.print("US_L:");
  Serial.print(US_L,DEC);
  Serial.print(",");
  Serial.print("US_R:");
  Serial.print(US_R,DEC);
  Serial.print(",");
  Serial.print("US_F:");
  Serial.print(US_F,DEC);
  Serial.print(",");
  Serial.print("Encoder1:");
  Serial.print(count1,DEC);
  Serial.print(",");
  Serial.print("Encoder2:");
  Serial.print(count2,DEC);
  Serial.println();
}

void turn(int numTurns,char turnDirection)
{
  md.setM1Speed(STOP);
  md.setM2Speed(STOP);
  for (int i=0;i<numTurns;i=i+1;)
  {
    count1 = 0;
    count2 = 0;
    if (turnDirection == 'L')
    {
      while ((count1<turnTickCount)&&(count2<turnTickCount))
      {
        md.setM1Speed(-1*SPEED);
        md.setM2Speed(SPEED);
      }
      md.setM1Speed(STOP);
      md.setM2Speed(STOP);
    }
    else
    {
      while ((count1<turnTickCount)&&(count2<turnTickCount))
      {
        md.setM1Speed(SPEED);
        md.setM2Speed(-1*SPEED);
      }
      md.setM1Speed(STOP);
      md.setM2Speed(STOP);
    }
    
  }
  
}

void acquireTarget()
{
  int drift = 1;
  while (targetAcquired==FALSE)
  {
    checkSensors();
    if ((distanceIR_L < dangerZone)|| (distanceIR_R < dangerZone))
    {
      //STOP
      md.setM1Speed(STOP);
      md.setM2Speed(STOP);
    }
    else if ((abs(distanceIR_L - pickupVictim)<=pickupTolerance) && (abs(distanceIR_R - pickupVictim)<=pickupTolerance))
    {
      //Stop and pick up target
      md.setM1Speed(STOP);
      md.setM2Speed(STOP);
      grabVictim();
      targetAcquired = TRUE;
    }
    else if ((distanceIR_L <= victimAhead) || (distanceIR_R <= victimAhead))
    {
      if ((abs(distanceIR_L - distanceIR_R) <= threshold) && (distanceIR_L > pickupVictim) && (distanceIR_R > pickupVictim))
      {
      //Target Aligned, Keep Going Slowly
      md.setM1Speed(SLOW);
      md.setM2Speed(SLOW);
      }
      else if (distanceIR_R < distanceIR_L)
      {
      //Turn Left Slowly
      md.setM1Speed(SLOW-drift);
      md.setM2Speed(SLOW);
      }
      else
      {
      //Turn Right Slowly
      md.setM1Speed(SLOW);
      md.setM2Speed(SLOW-drift);
      }
    }
    else
    {
      md.setM1Speed(SLOW);
      md.setM2Speed(SLOW);
    }
  }
}
 
void travelDistance()
{
  checkSensors();
  int rightSet = distanceRightUS;

  //assume going forward

  int err,rtol,ltol;
  //int tol = 3;
  while ( count1 < totalTicks && count2 < totalTicks )
  {
  checkSensors();

  //check error
  err = rightSet - distanceRightUS;
  if( err > 0)
  {
    //adjust rtol
    rtol = 2;
    ltol = 0;
  }
  else if(err < 0)
  {
    //adjust ltol
    rtol = 0;
    ltol = 2;
  }
  else
  {
    rtol = 0;
    ltol = 0;
  }

  
  //md.setM1Speed(SPEED+tol);

  md.setM1Speed(SPEED + ltol);
  md.setM2Speed(SPEED + rtol);
  
    }

    md.setM1Speed(STOP);
    md.setM2Speed(STOP);
    
}
 
