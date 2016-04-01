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
* 2) http://www.instructables.com/id/Get-started-with-Distance-sensors-and-Arduino/?ALLSTEPS for example code for IR/US
* 3) http://www.robotshop.com/blog/en/arduino-5-minute-tutorials-lesson-4-ir-Distance-sensor-push-button-2-3637 for IR example code
* 4) http://www.robotoid.com/appnotes/circuits-quad-encoding.html
* 5) http://learn.robotgeek.com/getting-started/30-dev-kits/55-robotgeek-gripper-getting-started-guide.html
* 6) http://learn.trossenrobotics.com/30-robotgeek-getting-started-guides/dev-kits/55-robotgeek-gripper-kit-tips for gripper sample code
* 7) https://gist.github.com/philippbosch/5395696 for sprintf example code
************************************************************************************/
/*****************************************************************************************
*  Notes:
* Constants are all capital
* Global volatile variables start with a capital letter and camelcase for the remaining
* Local variables are camelcase
*****************************************************************************************/
#include <Servo.h>   //include the servo library to control the RobotGeek Servos
#include "DualMC33926MotorShield.h"

DualMC33926MotorShield mtr_ctrl;

//for instructions from Pi
const char GO = 'G';
const char TURN = 'T';
const char PICKUP = 'P';
const char DROPOFF = 'D';
const char RECOVER = 'R';

//Motor speeds
const int STOP = 0;  
const int SLOW = 40;     
const int SPEED = 80;

//Controls
const int KP = 1;

Servo microServo;   //create an servo object for the 9g FT-FS90MG micro servo
Servo largeServo;   //create an servo object for the RobotGeek 180 degree servo

//*********************************************************************************
//                PIN ASSIGNMENTS
//*********************************************************************************
const int MICRO_SERVOPIN = 5;    //pin that the micro servo will be attached to
const int LARGE_SERVOPIN = 6;   //pin that the large servo will be attached to

//declare pins for Infrared sensors
const int ANALOG_PIN_IR_L = A8;     
const int ANALOG_PIN_IR_R = A9;    

//declare pins for Ultrasonic sensors
const int DIG_PIN_US_L = 31;
const int DIG_PIN_US_R = 33;
const int DIG_PIN_US_F = 35;        

//declare pins for LS7184 chips
const int DIG_PIN_CLK_1 = 20;             //interrupt will be attached for this (chip1)   //Change back to 30 for Due
const int DIG_PIN_DIR_1 = 32;             //requires digital pin (chip1)
const int DIG_PIN_CLK_2 = 21;             //interrupt will be attached for this (chip2)   //Change back to 34 for Due
const int DIG_PIN_DIR_2 = 36;             //requires digital pin (chip2)
//*********************************************************************************

//Time delays for gripper movement
const int TIME_DELAY = 20;
const int MICRO_DELAY = 5;

//Angle values for gripper
const int GRIP_OPEN = 0;
const int GRIP_CLOSED = 130;
const int GRIP_GRAB = 84;

//Angle values for wrist
//const int wristDown = 150;
const int WRIST_UP = 0;
const int WRIST_LEVEL = 90;
const int WRIST_PICKUP = 10;
const int WRIST_LOWER = 110;

//Distance Thresholds
const int PICKUP_VICTIM_THRESHOLD = 67;                       //pickup victim at this Distance (mm)
const int VICTIM_AHEAD_THRESHOLD = 110;                       //victim is ahead of robot, start aligning (mm)
const int PICKUP_TOLERANCE = 2;                         //use threshold - measurement and check tolerance (mm)
const int DANGER_ZONE = PICKUP_VICTIM_THRESHOLD-PICKUP_TOLERANCE;   //Stop Immediately

//Variables for IR sensors
long analogValue,distanceValue;
volatile long Distance_IR_L,Distance_IR_R;  
const int THRESHOLD = 5;                                //used to check if IR sensors approx. equal (mm)

//Variables for US sensors
long pulseWidth,cm,mm;                        //Added mm in case we use instead of cm
volatile long Distance_US_L,Distance_US_R,Distance_US_F; 

//Variables for LS7184 chips
volatile long Count1 = 0;                             //tick count chip 1
volatile long Count2 = 0;                             //tick count chip 2

//Variables for encoders 
const int CONV_FACTOR = 52;                   //counts per cm, 13 for cm 33 for in at 1x resolution, modified to 4x
const int TURN_TICK_COUNT = 1016;               //number of ticks for a turn (sqrt((3.875^2)+(3^2))*(pi/2)*2.54 = cm

//Variables used in main loop
volatile long TurnNum;
volatile char TurnDir;
volatile long TickGoal;
volatile long Distance;

volatile bool TargetAcquired = false;         //if target is in tow or not
volatile bool AtStart = true;

//Threshold variables
volatile long US_HALL_THRESHOLD;        //Volatile but treat as a constant
volatile long US_DANGER_THRESHOLD;        //Volatile but treat as a constant

volatile char instruction; 


//setup servo objects and set initial position
void setup()
{ 
  microServo.attach(MICRO_SERVOPIN);
  microServo.write(GRIP_CLOSED);        // sets the gripper servo position to closed 
  largeServo.attach(LARGE_SERVOPIN);
  largeServo.write(WRIST_PICKUP);     // sets the wrist servo position to travel position
  pinMode(DIG_PIN_CLK_1,INPUT);
  pinMode(DIG_PIN_DIR_1,INPUT);
  attachInterrupt(digitalPinToInterrupt(DIG_PIN_CLK_1),encoderInterrupt1,RISING);
  pinMode(DIG_PIN_CLK_2,INPUT);
  pinMode(DIG_PIN_DIR_2,INPUT);
  attachInterrupt(digitalPinToInterrupt(DIG_PIN_CLK_2),encoderInterrupt2,RISING);
  Serial.begin(250000);
  mtr_ctrl.init();
}
 
//repeat test process 
void loop()
{
  /* char buffer[128];
  char instruction; */
  if (AtStart)
  {
    //Set Threshold to maintain distance from objects
    //Take US L/R measurements at start
      
      Distance_US_L = checkUS(DIG_PIN_US_L);
      Distance_US_R = checkUS(DIG_PIN_US_R);
//      if (Distance_US_L > Distance_US_R)
//      {
//        Distance = Distance_US_R;
//      }
//      else
//      {
//        Distance = Distance_US_L;
//      }
      //US_HALL_THRESHOLD = Distance;
      //US_DANGER_THRESHOLD = Distance/2; 
      
      Distance = (Distance_US_L + Distance_US_R);
      US_HALL_THRESHOLD = Distance/3;
      US_DANGER_THRESHOLD = Distance/5; 
      //delay(3000);      
      
      //Distance = 121;   //cm, 4 feet
      //TickGoal = CONV_FACTOR * Distance;
      //travelDistance(TickGoal);
      
//      TurnNum = 1;
//      TurnDir = 'L';
//      turn(TurnNum,TurnDir);
//      
//      TurnNum = 1;
//      TurnDir = 'R';
//      turn(TurnNum,TurnDir);
//      
//      TurnNum = 2;
//      TurnDir = 'L';
//      turn(TurnNum,TurnDir);
      
      AtStart = false;

      //checkSensors();
      //Serial.println();
      //delay(200);
  }
  
  
 if(Serial.available() > 0)
  {
    instruction = Serial.read();
    switch (instruction) {
      case GO:
        Distance = Serial.parseInt();
        TickGoal = CONV_FACTOR * Distance;
        travelDistance(TickGoal);
        taskComplete();
      break;
      case TURN:
        TurnNum = Serial.parseInt();
        TurnDir = Serial.read();
        turn(TurnNum,TurnDir);
        taskComplete();
      break;
      case PICKUP:
        acquireTarget();
        taskComplete();
      break;
      case DROPOFF:
        dropVictim();
        raiseGripper();
        taskComplete();
      break;
      case RECOVER:
          taskComplete();
      break;
      default: 
     
      break;
    }
    //taskComplete();
  } 
}

long checkIR (int pinNumIR)
{
  analogValue = analogRead(pinNumIR);                       //Read current analog value from pin
  //distanceValue = (400000-(42*analogValue))/(10*analogValue);   //conversion based on one from ref //Due conversion
  distanceValue = (264000-(42*analogValue))/(10*analogValue);   //conversion based on one from ref //Mega conversion
  return distanceValue;                                 //mm
}

void grabVictim()
{
  //Pickup victim
  microServo.write(GRIP_OPEN);              //set gripper to fully open
  delay(TIME_DELAY);                        //wait 
  largeServo.write(WRIST_LOWER);            //Lower wrist to grab victim
  delay(TIME_DELAY);                        //wait 
  microServo.write(GRIP_GRAB);              //set gripper to grab victim
  delay(TIME_DELAY);                        //wait 
  for (int i=WRIST_LOWER;i>WRIST_PICKUP;i=i-1)
  {
    largeServo.write(i);                    //Pickup victim
    delay(MICRO_DELAY);
  }
} 

void dropVictim()
{
  //Drop victim
  for (int i=WRIST_PICKUP;i<WRIST_LOWER;i=i+1)
    {
    largeServo.write(i);                //Lower wrist to drop victim
    delay(MICRO_DELAY);
    }
  delay(TIME_DELAY);                     //wait 
  microServo.write(GRIP_OPEN);           //set gripper to 0 degrees = fully open
  TargetAcquired = false;
} 

void raiseGripper()
{
  for (int i=WRIST_LOWER;i>WRIST_PICKUP;i=i-1)
  {
    largeServo.write(i);   
    delay(MICRO_DELAY);
  }
  delay(TIME_DELAY); 
  microServo.write(GRIP_CLOSED);    
}

//Checks US sensors - Code based on ref 1,2
long checkUS (int pinNumUS)
{
  digitalWrite(pinNumUS,LOW);                 // make sure pin is low before pulsing
  pinMode(pinNumUS,OUTPUT);                   // set up pin to initiate pulse
  delayMicroseconds(2);                       // for 2 microseconds 
  digitalWrite(pinNumUS,HIGH);                // Start pulse
  delayMicroseconds(5);                     // for 5 microseconds
  digitalWrite(pinNumUS,LOW);                 // set pin back to low to ready for return pulse
  pinMode(pinNumUS,INPUT);                    // change pin to Input mode for return pulse
  pulseWidth = pulseIn(pinNumUS,HIGH,18500);    // wait for return pulse. Timeout after 18.5 milliseconds
  //cm = pulseWidth/88;                       // Convert to centimeters, use 58 for Mega, 53 for Due, jk it's 88 for Due
  //mm = (pulseWidth*10)/88;                      // Check using mm instead of cm //Due
  mm = (pulseWidth*10)/58;                      // Check using mm instead of cm //Mega
  //return cm;
  //Serial.print(mm);
 //Serial.print(" ");
  return mm;
}
 
 //Interrupt routine for LS7184 chip 1 - Code based on ref 4
void encoderInterrupt1()
{
  Count1 = digitalRead(DIG_PIN_DIR_1) ? Count1 + 1: Count1 - 1;
}

//Interrupt routine for LS7184 chip 2 - Code based on ref 4
void encoderInterrupt2()
{
  Count2 = digitalRead(DIG_PIN_DIR_2) ? Count2 + 1: Count2 - 1;
}

void checkSensors()
{
  Distance_IR_L = checkIR(ANALOG_PIN_IR_L);
  Distance_IR_R = checkIR(ANALOG_PIN_IR_R);
  Distance_US_L = checkUS(DIG_PIN_US_L);
  Distance_US_R = checkUS(DIG_PIN_US_R);
  Distance_US_F = checkUS(DIG_PIN_US_F);
}

//Used Ref 7 for example sprintf code
void printSensorValues()
{
  char buffer[128];    //2 ints,5 longs,49 chars
  //sprintf(buffer, "IR_L:%d,IR_R:%d,US_L:%d,US_R:%d,US_F:%d,Encoder1:%d,Encoder2:%d\n",Distance_IR_L,Distance_IR_R,Distance_US_L,Distance_US_R,Distance_US_F,Count1,Count2);
  sprintf(buffer, "Encoder1:%d,Encoder2:%d\n",Count1,Count2); //We can use this if we only want to send encoder counts
  Serial.print(buffer);
}

void taskComplete()
{
  char buffer[128];  
  sprintf(buffer,"TaskComplete:%c\n",'Y');
  Serial.print(buffer);
}

void turn(long numTurns,char turnDirection)
{
  long count_L;// = Count1 + TURN_TICK_COUNT;
  long count_R;// = Count2 + TURN_TICK_COUNT;
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);
  for (int i=0;i<numTurns;i=i+1)
  {
    if (turnDirection == 'L')
    {
      count_L = Count1 + TURN_TICK_COUNT;
      count_R = Count2 - TURN_TICK_COUNT; 
      accelFromStop(SPEED,2); //left
      while ((Count1 < count_L)&&(Count2 > count_R))
      {
        mtr_ctrl.setM1Speed(SPEED);
        mtr_ctrl.setM2Speed(-1*SPEED);
      }
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
    }
    else if (turnDirection == 'R')
    {
      count_L = Count1 - TURN_TICK_COUNT;
      count_R = Count2 + TURN_TICK_COUNT;
      accelFromStop(SPEED,3); //right
      while ((Count1 > count_L)&&(Count2 < count_R))
      {
        mtr_ctrl.setM1Speed(-1*SPEED);
        mtr_ctrl.setM2Speed(SPEED);
      }
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
    }
    else 
    {
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
    }
  }
}

void acquireTarget()
{
  int drift = 3;                                //Used to slow one motor to align by drifting
  int recoveryDistance = VICTIM_AHEAD_THRESHOLD - PICKUP_VICTIM_THRESHOLD;  //Distance to back up if recovery is needed 
  int recoveryTickCount = recoveryDistance * CONV_FACTOR;           //ticks to travel for recovery  
  TargetAcquired = false;
  
  //Start travelling forward slowly
  //mtr_ctrl.setM1Speed(SLOW);
  //mtr_ctrl.setM2Speed(SLOW);
  accelFromStop(SLOW,1); //reverse because gripper on back
  
  while (TargetAcquired==false)
  {
	bool dangerZoneReached = false;  
    checkSensors();
    if ((Distance_IR_L < DANGER_ZONE) || (Distance_IR_R < DANGER_ZONE))
    {
    //STOP
    mtr_ctrl.setM1Speed(STOP);
    mtr_ctrl.setM2Speed(STOP);
    //need to recover from here, back up slowly, should check sensors
    long count_L = Count1 + recoveryTickCount;
    long count_R = Count2 + recoveryTickCount;
    //This may need to be checked to see if works, going from stop to slow speed
    accelFromStop(SLOW,0); //forward
    //Above may need to be checked to see if works, going from stop to slow speed
    while ((Count1 < count_L) && (Count2 < count_R)&&(dangerZoneReached==false))
    {
      checkSensors();
      if ((Distance_US_F > US_DANGER_THRESHOLD) && (Distance_US_L > US_DANGER_THRESHOLD) && (Distance_US_R > US_DANGER_THRESHOLD))
      {
        mtr_ctrl.setM1Speed(SLOW);
        mtr_ctrl.setM2Speed(SLOW);
      }
      else
      {
        mtr_ctrl.setM1Speed(STOP);
        mtr_ctrl.setM2Speed(STOP);
        dangerZoneReached=true;
      }
    }
    }
    else if ((abs(Distance_IR_L - PICKUP_VICTIM_THRESHOLD)<=PICKUP_TOLERANCE) && (abs(Distance_IR_R - PICKUP_VICTIM_THRESHOLD)<=PICKUP_TOLERANCE) && (abs(Distance_IR_L-Distance_IR_R) <= THRESHOLD))
    {
      //Stop and pick up target
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
      grabVictim();
      TargetAcquired = true;
    }
	else if ((Distance_IR_L <= VICTIM_AHEAD_THRESHOLD) && (Distance_IR_R <= VICTIM_AHEAD_THRESHOLD))
	{
		if ((abs(Distance_IR_L - Distance_IR_R) <= THRESHOLD) && (Distance_IR_L > PICKUP_VICTIM_THRESHOLD) && (Distance_IR_R > PICKUP_VICTIM_THRESHOLD))
		{
		//Target Aligned, Keep Going Slowly
		  mtr_ctrl.setM1Speed(-1*SLOW);
		  mtr_ctrl.setM2Speed(-1*SLOW);
		}
		else if ((Distance_IR_R < Distance_IR_L) && (Distance_IR_L > PICKUP_VICTIM_THRESHOLD) && (Distance_IR_R > PICKUP_VICTIM_THRESHOLD))
		{
		//Drift Left Slowly
		  mtr_ctrl.setM1Speed(-1*SLOW-drift);
		  mtr_ctrl.setM2Speed(-1*SLOW);
		}
		else if ((Distance_IR_L < Distance_IR_R) && (Distance_IR_L > PICKUP_VICTIM_THRESHOLD) && (Distance_IR_R > PICKUP_VICTIM_THRESHOLD))
		{
		//Drift Right Slowly
		  mtr_ctrl.setM1Speed(-1*SLOW);
		  mtr_ctrl.setM2Speed(-1*SLOW-drift);
		}
		else
		{
		  mtr_ctrl.setM1Speed(-1*SLOW);
		  mtr_ctrl.setM2Speed(-1*SLOW);
		}
	}
	else
	{
		mtr_ctrl.setM1Speed(-1*SLOW);
		mtr_ctrl.setM2Speed(-1*SLOW);
	}
  }
}
 
void travelDistance(long numTicks)
{
  long count_L = Count1 + numTicks;
  long count_R = Count2 + numTicks;
  //checkSensors();
  //int rightSet = Distance_US_R;
  
  //assume going forward
  
  int err,rtol,ltol;
  //int tol = 3;
  accelFromStop(SPEED,0);
  
  while ( Count1 < count_L && Count2 < count_R )
  {
    checkSensors();
    
    //check error
    //err = rightSet - Distance_US_R;
    if (Distance_US_L > (2*US_HALL_THRESHOLD))
    {
      err = US_HALL_THRESHOLD - Distance_US_R;
      if( err > 0)
      {
        //adjust rtol

        //original
        //rtol = KP*err;    //switch this to ltol
        //ltol = 0;

        //first change
        ltol = KP*err;
        rtol = 0;

        //second change
        //ltol = 5;
        //rtol = 0;
      }
      else if(err < 0)
      {
        //adjust ltol

        //original
        //rtol = 0;
        //ltol = -1*KP*err;  //switch this to rtol

        //first change
        ltol = 0;
        rtol = -1*KP*err;

        //2nd change
        //ltol = 0;
        //rtol = 5;
      }
      else
      {
        rtol = 0;
        ltol = 0;
      }
    }
    else
    {
      err = US_HALL_THRESHOLD - Distance_US_L;
      if( err > 0)
      {
        //adjust rtol

        //original
        //rtol = 0;
        //ltol = KP*err;

        //first change
        rtol = KP*err;
        ltol = 0;
      }
      else if(err < 0)
      {
        //adjust ltol

        //original
        //rtol = -1*KP*err;
        //ltol = 0;

        //first change
        rtol = 0;
        ltol = -1*KP*err;
      }
      else
      {
        rtol = 0;
        ltol = 0;
      }
    }
    
    //mtr_ctrl.setM1Speed(SPEED+tol);
    if (Distance_US_F > US_DANGER_THRESHOLD)
    {
      mtr_ctrl.setM1Speed(SPEED + ltol);
      mtr_ctrl.setM2Speed(SPEED + rtol);
    }
    else
    {
      //Stop and Recover
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
    }
  }
  
    //Stop at location
    mtr_ctrl.setM1Speed(STOP);
    mtr_ctrl.setM2Speed(STOP);
    
}

//int code
// 0 ---- forward
// 1 ---- backward
// 2 ---- left
// 3 ---- right
// default ---- return? 
void accelFromStop(int input , int code)
{
  //assert input > 0
  int upper = (220*input)/100;
  //If input is positive do this
  if (upper > 0)

  
  {
    
    for(int i = 1; i <= upper; ++i)
    {
      switch(code) {
        case 0:
          mtr_ctrl.setSpeeds(i,i);
          break;
        case 1:
          mtr_ctrl.setSpeeds(-1*i,-1*i);
          break;
        case 2:
          mtr_ctrl.setSpeeds(i,-1*i);
          break;
        case 3:
          mtr_ctrl.setSpeeds(-1*i,i);
      }
    }
    delay(20);
    for(int i = upper; i >= input; --i)
    {
       switch(code) {
        case 0:
          mtr_ctrl.setSpeeds(i,i);
          break;
        case 1:
          mtr_ctrl.setSpeeds(-1*i,-1*i);
          break;
        case 2:
          mtr_ctrl.setSpeeds(i,-1*i);
          break;
        case 3:
          mtr_ctrl.setSpeeds(-1*i,i);
      }
    }
  }
  else //If zero, stop
  {
    mtr_ctrl.setSpeeds(STOP,STOP);
  }
}
 
