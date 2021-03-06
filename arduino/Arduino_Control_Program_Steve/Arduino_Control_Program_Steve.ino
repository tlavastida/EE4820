/*	SMS 4/5/16 11:27AM - ADDED ADJUST FUNCTION TO STRAIGHTEN OUT ROBOT AFTER MOVEMENTS
*/

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
const char MANIPULATE = 'M';

//for gripper commands
//const char OPEN = 'O';
//const char CLOSE = 'C';
//const char UP = 'U';
//const char DOWN = 'D';

//Motor speeds
const int STOP = 0;  
const int SLOW = 55;     
const int SPEED = 100; //90;
const int TURN_SPEED = 130;    //added a faster speed to test zero turn
const int LEFT_MOTOR_ADJ = -4;  //SMS was -6, testing -5// -15

//Controls
const int KP_INV = 10; //Kp = 1/10 so Kp^-1 = 10
const int KPL = 10;       //60     
const int KPR = 10;       //60
const int KD = 1;

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
const int DIG_PIN_CLK_R = 20;             //interrupt will be attached for this (chip1)   //Change back to 30 for Due
const int DIG_PIN_DIR_R = 32;             //requires digital pin (chip1)
const int DIG_PIN_CLK_L = 21;             //interrupt will be attached for this (chip2)   //Change back to 34 for Due
const int DIG_PIN_DIR_L = 36;             //requires digital pin (chip2)
//*********************************************************************************

//Time delays for gripper movement
const int TIME_DELAY = 2000;       //ms, used for delay between movements of gripper
const int MICRO_DELAY = 500;      //microseconds, used to slow the movement of the gripper

//Angle values for gripper
const int GRIP_OPEN = 0;
const int GRIP_CLOSED = 130;
const int GRIP_GRAB = 88;

//Angle values for wrist
//const int wristDown = 150;
const int WRIST_UP = 0;
const int WRIST_LEVEL = 90;
const int WRIST_PICKUP = 10;
const int WRIST_LOWER = 115;

//Distance Thresholds
const int PICKUP_VICTIM_THRESHOLD = 67;                       //pickup victim at this Distance (mm)
const int VICTIM_AHEAD_THRESHOLD = 110;                       //victim is ahead of robot, start aligning (mm)
const int PICKUP_TOLERANCE = 5;                         //use threshold - measurement and check tolerance (mm)
const int DANGER_ZONE = PICKUP_VICTIM_THRESHOLD-PICKUP_TOLERANCE;   //Stop Immediately

//Variables for IR sensors
long analogValue,distanceValue;
volatile long Distance_IR_L,Distance_IR_R;  
const int THRESHOLD = 5;                                //used to check if IR sensors approx. equal (mm)

//Variables for US sensors
long pulseWidth,cm,mm;                        //Added mm in case we use instead of cm
volatile long Distance_US_L,Distance_US_R,Distance_US_F; 

//Variables for LS7184 chips
volatile long Count_Encoder_Left = 0;                             //tick count chip 1
volatile long Count_Encoder_Right = 0;                             //tick count chip 2

//Variables for encoders 
const int CONV_FACTOR = 52;                     //counts per cm, 13 for cm 33 for in at 1x resolution, modified to 4x
const int TURN_TICK_COUNT = 1016;               //number of ticks for a turn (sqrt((3.875^2)+(3^2))*(pi/2)*2.54 = cm

//Variables used in main loop
volatile long TurnNum;
volatile char TurnDir;
volatile long TickGoal;
volatile long Distance;
volatile char instruction; 

volatile bool TargetAcquired = false;         //if target is in tow or not
volatile bool AtStart = true;

//Threshold variables
volatile long US_HALL_THRESHOLD;          //Volatile but treat as a constant
const long US_DANGER_THRESHOLD = 9;         //Volatile but treat as a constant

//Options to manipulate gripper
volatile char GripOption;
volatile int GripPosition;
volatile int WristPosition;

//Variables for serial
char Buffer[128]; 

//setup servo objects and set initial position
void setup()
{ 
  microServo.attach(MICRO_SERVOPIN);
  microServo.write(GRIP_OPEN);        // sets the gripper servo position to closed 
  GripPosition = GRIP_OPEN;
  largeServo.attach(LARGE_SERVOPIN);
  largeServo.write(WRIST_PICKUP);     // sets the wrist servo position to travel position
  WristPosition = WRIST_PICKUP;
  pinMode(DIG_PIN_CLK_R,INPUT);     //changing DIG_PIN_CLK_1 to say right
  pinMode(DIG_PIN_DIR_R,INPUT);     //changing DIG_PIN_DIR_1 to say right
  attachInterrupt(digitalPinToInterrupt(DIG_PIN_CLK_R),encoderInterruptRight,RISING);
  pinMode(DIG_PIN_CLK_L,INPUT);       //changing DIG_PIN_CLK_2 to say left
  pinMode(DIG_PIN_DIR_L,INPUT);       //changing DIG_PIN_DIR_2 to say left
  attachInterrupt(digitalPinToInterrupt(DIG_PIN_CLK_L),encoderInterruptLeft,RISING);
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
      US_HALL_THRESHOLD =   6;//Distance/3;   //changed to cm, set value
      //US_DANGER_THRESHOLD = 4; //Distance/5; //changed to cm, set value 
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
  
  //microServo.write(GripPosition);				//SMS 4/6/16 1:01PM commenting out. Shouldn't need it the main loop
  //largeServo.write(WristPosition);
 if(Serial.available() > 0)
  {
    delayMicroseconds(MICRO_DELAY);
    instruction = Serial.read();
    switch (instruction) {
      case GO:
        Distance = Serial.parseInt();
        TickGoal = CONV_FACTOR * Distance;
        //travelDistance(TickGoal);
        travelDistance_Enc(TickGoal);
        //travelDistance_timed(TickGoal);     //SMS Used for testing
        taskComplete();
      break;
      case TURN:
        TurnNum = Serial.parseInt();
        TurnDir = Serial.read();
        for(int i = 0; i < TurnNum; ++i)
        {
          switch(TurnDir)
          {
            case 'L':
              turnLeft_P();
              break;
            case 'R':
              turnRight_P();
              break;
            default:
              break;
          }
          delay(2000);
        }
        //turn(TurnNum,TurnDir);
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
      case MANIPULATE:
        GripOption = Serial.read();     //send C for closed, O for open, U for up, D for down
        manipulateGripper(GripOption);
        taskComplete();
      break;
      case RECOVER:
		    //adjust();
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

void manipulateGripper(char option)
{
  if (option == 'C')
  {
    microServo.write(GRIP_GRAB);              //set gripper to grab victim
    GripPosition = GRIP_GRAB;
  }
  else if (option == 'O')
  {
    microServo.write(GRIP_OPEN);              //set gripper to fully open
    GripPosition = GRIP_OPEN;
  }
  else if (option == 'U')
  {
    for (int i=WRIST_LOWER;i>WRIST_PICKUP;i=i-1)
    {
      largeServo.write(i);                    //Pickup victim
      delayMicroseconds(MICRO_DELAY);
    }
  }
  else if (option == 'D')
  {
    for (int i=WRIST_PICKUP;i<WRIST_LOWER;i=i+1)
    {
      largeServo.write(i);                //Lower wrist to drop victim
      delayMicroseconds(MICRO_DELAY);
    }
  }
  else
  {
    delayMicroseconds(MICRO_DELAY);
  }
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
    delayMicroseconds(MICRO_DELAY);
  }
} 

//SMS - This is the function that we were testing last 4/7/16 8:37AM  
void travelDistance_Enc(int numTicks)
{
   int totalTicks = 0;
   int followerSpeed = 85;//+5;   //right motor is the follower
   int error = 0;
   int Kp = 30;             //10    
   int dangerCounter = 0;
   int dangerCountThresh = 2;
   Count_Encoder_Left = 0;
   Count_Encoder_Right = 0;
   int leftSpeed = 85;
   int danger_US = 3; 		//US danger zone (cm)
   int Kp_US = 2;     //10

   accelFromStop(85,0);
   //checkSensors();
   while (abs(totalTicks) < numTicks)
   {
      checkSensors();
	  
	    if(Distance_US_F > US_DANGER_THRESHOLD)
      {
		
		    if ((Distance_US_L <= danger_US) && (Distance_US_L != 0))
		    {
		      error = danger_US - Distance_US_L;
		      followerSpeed -= error*Kp_US;
		    }
		    else if ((Distance_US_R <= danger_US) && (Distance_US_R != 0))
		    {
		      error = danger_US - Distance_US_R;
		      followerSpeed += error*Kp_US;
		    }		
		    //else  
		    //{
		      dangerCounter = 0;
          error = Count_Encoder_Left - Count_Encoder_Right;
          followerSpeed += error/Kp; 
		    //}

        mtr_ctrl.setM2Speed(leftSpeed+LEFT_MOTOR_ADJ);  //was 2
        mtr_ctrl.setM1Speed(followerSpeed);//+8); //was 1
        //delay(100);
        //delay(20);
      }
      else
      {
        dangerCounter += 1;
        if(dangerCounter >= dangerCountThresh)
        {
//          leftSpeed = STOP;
//          followerSpeed = STOP;
          //maybe break?
          break;
        } 
      }
    Count_Encoder_Left = 0;
    Count_Encoder_Right = 0;

    delay(20);
    //100-60 
    totalTicks += Count_Encoder_Left;
   } //end while loop
   
   mtr_ctrl.setM2Speed(STOP);
   mtr_ctrl.setM1Speed(STOP); 
}

//Don't use this, will not work
/*void travelDistance_Enc(int numTicks)
{
   int followerSpeed = TURN_SPEED;//+5; //right motor is the follower
   int minFollowerSpeed = TURN_SPEED - 30;
   int maxFollowerSpeed = TURN_SPEED + 30;
   int error = 0;
   int Kp_inv = 2;
   int dangerCounter = 0;
   int dangerCountThresh = 2;
   Count_Encoder_Left = 0;
   Count_Encoder_Right = 0;
   int leftSpeed = TURN_SPEED;

   long us_l, us_r, us_f;

   int dangerZoneThresh = 4;
    int dl = 0;
    int dr = 0;
    int err_us = 0;
    int Kp_us = 8;  //YOLO #fuckit
   //checkSensors();

   us_l = checkUS_mm(DIG_PIN_US_L);
   us_r = checkUS_mm(DIG_PIN_US_R);
   Distance_US_F = checkUS(DIG_PIN_US_F);
   
   int prevLeft = us_l;
   int prevRight = us_r;
 

   while (abs(Count_Encoder_Left)<numTicks)
   {
      //checkSensors();
      us_l = checkUS_mm(DIG_PIN_US_L);
      us_r = checkUS_mm(DIG_PIN_US_R);
      Distance_US_F = checkUS(DIG_PIN_US_F);
      
      dr = us_r - prevRight;
      dl = us_l - prevLeft;

      Serial.println(dl);
      Serial.println(dr);
      Serial.println();
      printSensorValues();

      if ( dr > 0 || dl < 0)
      {
         err_us = (abs(dl) + abs(dr))/2;
      }
      else if( dr < 0 || dl > 0 )
      {
        err_us = -1*((abs(dl) + abs(dr))/2);
      }
      if(Distance_US_R > 21 && abs(dl) > 0) 
      {
        err_us = -1*dl;
      }
      else if(Distance_US_L > 21 && abs(dr) > 0)
      {
        err_us = dr;
      }
      
      Count_Encoder_Right += err_us/Kp_us;
      
      if(Distance_US_F > US_DANGER_THRESHOLD)
      {
        dangerCounter = 0;
        error = Count_Encoder_Left - Count_Encoder_Right;
        followerSpeed = (followerSpeed+error*Kp_inv <= SLOW) ? SLOW : (followerSpeed + error*Kp_inv >= maxFollowerSpeed) ? maxFollowerSpeed : followerSpeed +error*Kp_inv; 
      }
      else
      {
        dangerCounter += 1;
        if(dangerCounter >= dangerCountThresh)
        {
//          leftSpeed = STOP;
//          followerSpeed = STOP;
          //maybe break?
          break;
        } 
      }

      prevLeft = us_l;
      prevRight = us_r;
      mtr_ctrl.setM2Speed(leftSpeed+LEFT_MOTOR_ADJ-7);  //was 2
      mtr_ctrl.setM1Speed(followerSpeed);//+8); //was 1
     Serial.print(leftSpeed+LEFT_MOTOR_ADJ-7);
     Serial.print(" ");
     Serial.print(followerSpeed);
     Serial.println();
      delay(60);    //100-60 
   }
   mtr_ctrl.setM2Speed(STOP);
   mtr_ctrl.setM1Speed(STOP); 
}*/

//backup
/*
void travelDistance_Enc(int numTicks)
{
   int followerSpeed = SPEED;//+5;   //right motor is the follower
   int error = 0;
   int Kp = 10;
   int dangerCounter = 0;
   int dangerCountThresh = 3;
   Count_Encoder_Left = 0;
   Count_Encoder_Right = 0;
   int leftSpeed = SPEED;
   checkSensors();
   while (abs(Count_Encoder_Left)<numTicks)
   {
      checkSensors();
      if(Distance_US_F > US_DANGER_THRESHOLD)
      {
        dangerCounter = 0;
        error = Count_Encoder_Left - Count_Encoder_Right;
        followerSpeed += error/Kp; 
      }
      else
      {
        dangerCounter += 1;
        if(dangerCounter >= dangerCountThresh)
        {
//          leftSpeed = STOP;
//          followerSpeed = STOP;
          //maybe break?
          break;
        } 
      }
      mtr_ctrl.setM2Speed(leftSpeed+LEFT_MOTOR_ADJ);  //was 2
      mtr_ctrl.setM1Speed(followerSpeed);//+8); //was 1
      delay(100);    //100-60 
   }
   mtr_ctrl.setM2Speed(STOP);
   mtr_ctrl.setM1Speed(STOP); 
}*/

void dropVictim()
{
  //Drop victim
  for (int i = WRIST_PICKUP; i < WRIST_LOWER; i = i+1)
    {
    largeServo.write(i);                //Lower wrist to drop victim
    delayMicroseconds(MICRO_DELAY);       //maybe delayMicroseconds?
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
    delayMicroseconds(MICRO_DELAY);
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
  cm = pulseWidth/58;                       // Convert to centimeters, use 58 for Mega, 53 for Due, jk it's 88 for Due
  //mm = (pulseWidth*10)/88;                      // Check using mm instead of cm //Due
  //mm = (pulseWidth*10)/58;                      // Check using mm instead of cm //Mega
  return cm;
  //Serial.print(mm);
  //Serial.print(" ");
  //return mm;
}

long checkUS_mm (int pinNumUS)
{
  digitalWrite(pinNumUS,LOW);                 // make sure pin is low before pulsing
  pinMode(pinNumUS,OUTPUT);                   // set up pin to initiate pulse
  delayMicroseconds(2);                       // for 2 microseconds 
  digitalWrite(pinNumUS,HIGH);                // Start pulse
  delayMicroseconds(5);                     // for 5 microseconds
  digitalWrite(pinNumUS,LOW);                 // set pin back to low to ready for return pulse
  pinMode(pinNumUS,INPUT);                    // change pin to Input mode for return pulse
  pulseWidth = pulseIn(pinNumUS,HIGH,18500);    // wait for return pulse. Timeout after 18.5 milliseconds
  //cm = pulseWidth/58;                       // Convert to centimeters, use 58 for Mega, 53 for Due, jk it's 88 for Due
  //mm = (pulseWidth*10)/88;                      // Check using mm instead of cm //Due
  mm = (pulseWidth*10)/58;                      // Check using mm instead of cm //Mega
  //return cm;
  //Serial.print(mm);
  //Serial.print(" ");
  return mm;
}

 //Interrupt routine for LS7184 chip 1 - Code based on ref 4
void encoderInterruptLeft()
{
  Count_Encoder_Left = digitalRead(DIG_PIN_DIR_L) ? Count_Encoder_Left + 1: Count_Encoder_Left - 1;
}

//Interrupt routine for LS7184 chip 2 - Code based on ref 4
void encoderInterruptRight()
{
  Count_Encoder_Right = digitalRead(DIG_PIN_DIR_R) ? Count_Encoder_Right - 1: Count_Encoder_Right + 1;      //SMS changed from +/-
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
  //char buffer[128];    //2 ints,5 longs,49 chars
  sprintf(Buffer, "IR_L:%ld,IR_R:%ld,US_L:%ld,US_R:%ld,US_F:%ld,Encoder_L:%ld,Encoder_R:%ld\n",Distance_IR_L,Distance_IR_R,Distance_US_L,Distance_US_R,Distance_US_F,Count_Encoder_Left,Count_Encoder_Right);
  //sprintf(Buffer, "Encoder_L:%d,Encoder_R:%d\n",Count_Encoder_Left,Count_Encoder_Right); //We can use this if we only want to send encoder counts
  Serial.print(Buffer);
}

void taskComplete()
{
  //char buffer[128];  
  //sprintf(Buffer,"TaskComplete:%c\n",'Y');
  sprintf(Buffer,"Encoder_L: %ld,\tEncoder_R: %ld\n",Count_Encoder_Left,Count_Encoder_Right);
  Serial.print(Buffer);
}

void turn(long numTurns,char turnDirection)
{
  long count_L;// = Count_Encoder_Left + TURN_TICK_COUNT;
  long count_R;// = Count_Encoder_Right + TURN_TICK_COUNT;
  long right_turn_adjust = -56;    // -236;  //for variations in encoder readings //-240 old
  long left_turn_adjust = 32;   //for variations in encoder readings   // 4 old // 8 old
  
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);

  switch (turnDirection) 
  {
    case 'L':
    {
      for (int i=0;i<numTurns;i=i+1)
      {
        //count_L = Count_Encoder_Left - (TURN_TICK_COUNT + left_turn_adjust); 
        //count_R = Count_Encoder_Right - (TURN_TICK_COUNT + left_turn_adjust); //SMS changing logic for left turn
		count_L = Count_Encoder_Left - (TURN_TICK_COUNT + left_turn_adjust); 
        count_R = Count_Encoder_Right + (TURN_TICK_COUNT + left_turn_adjust); 
        accelFromStop(SPEED,2); //left
        //while ((Count_Encoder_Left < count_L)&&(Count_Encoder_Right > count_R))
        //while( Count_Encoder_Left > count_L  && Count_Encoder_Right > count_R )  //first change/idea first comparison was >
		while( Count_Encoder_Left > count_L  && Count_Encoder_Right < count_R )  //SMS changed logic for turns
        {
          mtr_ctrl.setM1Speed(TURN_SPEED);      //switching to turn speed for testing, change 1
          mtr_ctrl.setM2Speed(-1*(TURN_SPEED+LEFT_MOTOR_ADJ));
        }
        mtr_ctrl.setM1Speed(STOP);
        mtr_ctrl.setM2Speed(STOP);
      }
      break;
    }
    case 'R':
    {
      for (int i=0; i<numTurns;i=i+1)
      {
        //count_L = Count_Encoder_Left + (TURN_TICK_COUNT + right_turn_adjust);
        //count_R = Count_Encoder_Right + (TURN_TICK_COUNT + right_turn_adjust);
		count_L = Count_Encoder_Left + (TURN_TICK_COUNT + right_turn_adjust);
        count_R = Count_Encoder_Right - (TURN_TICK_COUNT + right_turn_adjust);
        accelFromStop(SPEED,3); //right
        //while ((Count_Encoder_Left > count_L)&&(Count_Encoder_Right < count_R))
        //while( Count_Encoder_Left < count_L  && Count_Encoder_Right < count_R )  //first change/idea
		while( Count_Encoder_Left < count_L  && Count_Encoder_Right > count_R )  //SMS changed logic for turns
        {
          mtr_ctrl.setM1Speed(-1*TURN_SPEED);     //switching to turn speed for testing, change 1
          //mtr_ctrl.setM2Speed(TURN_SPEED);        //switching to turn speed for testing, change 1 //SMS 4/5/16 2:55PM commented out for change
          mtr_ctrl.setM2Speed(TURN_SPEED+LEFT_MOTOR_ADJ);  
        }
        mtr_ctrl.setM1Speed(STOP);
        mtr_ctrl.setM2Speed(STOP);
      }
      break;
    }
    default:
    {
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
      break;
    }
  }
}

void turnLeft_P()
{
  long currentTime = 0;
  
	int Kp = 3;

  int leftTickCount = 950; //individually tuned for left turns
	//Set points
  int count_L = Count_Encoder_Left - leftTickCount;
  int count_R = Count_Encoder_Right + leftTickCount;

  //Serial.print("Left target: ");
  //Serial.print(count_L);
  //Serial.print(", Right target: ");
  //Serial.print(count_R);
  //Serial.println();

  long startTime = millis();

  int topSpeed = 150;

  int el = count_L - Count_Encoder_Left; 
  int er = count_R - Count_Encoder_Right;
  int tol = 18;
  
  //compute error
	  el = count_L - Count_Encoder_Left;
	  er = count_R - Count_Encoder_Right;
	  
	  int leftInput = el*Kp;
	  int rightInput = er*Kp;
	  //compute inputs with saturation
    if(el >= 0)
    {
      leftInput = (leftInput <= SLOW) ? SLOW : (leftInput >= topSpeed) ? topSpeed : leftInput;
    }
    else 
    {
      leftInput = (leftInput >= -1*SLOW) ? -1*SLOW : (leftInput <= -1*topSpeed) ? -1*topSpeed : leftInput;
    }

    if(er >= 0)
    {
      rightInput = (rightInput <= SLOW) ? SLOW : (rightInput >= topSpeed) ? topSpeed : rightInput;
    }
    else
    {
      rightInput = (rightInput >= -1*SLOW) ? -1*SLOW : (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
    }
	  
	accelFromStop( (abs(leftInput)+abs(rightInput))/2 ,2);
  
  while ((abs(el) >= tol) || (abs(er) >= tol))
  {
	  //compute error
	  el = count_L - Count_Encoder_Left;
	  er = count_R - Count_Encoder_Right;
	  
	  leftInput = el*Kp;
	  rightInput = er*Kp;

    if(el >= 0)
    {
      leftInput = (leftInput <= SLOW) ? SLOW : (leftInput >= topSpeed) ? topSpeed : leftInput;
    }
    else 
    {
      leftInput = (leftInput >= -1*SLOW) ? -1*SLOW :(leftInput <= -1*topSpeed) ? -1*topSpeed : leftInput;
    }

    if(er >= 0)
    {
      rightInput = (rightInput <= SLOW) ? SLOW : (rightInput >= topSpeed) ? topSpeed : rightInput;
    }
    else
    {
      rightInput = (rightInput <= -1*SLOW) ? -1*SLOW : (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
    }
    
    //Serial.println(leftInput);
    //Serial.println(rightInput);
	  
	  //input the inputs
	  mtr_ctrl.setM2Speed(leftInput);
	  mtr_ctrl.setM1Speed(rightInput);

    //printSensorValues();
    if (( millis() - startTime) > 1200)
    {
      break;
    }
  }
  
  mtr_ctrl.setSpeeds(STOP,STOP);
  
}

void turnRight_P()
{
  {
  int Kp = 2; //1
  //1016,900,958,929,
  int rightTickCount = 943; //individually tuned for right turns
  //Set points
  int count_L = Count_Encoder_Left + rightTickCount;
  int count_R = Count_Encoder_Right - rightTickCount;

//  Serial.print("Left target: ");
//  Serial.print(count_L);
//  Serial.print(", Right target: ");
//  Serial.print(count_R);
//  Serial.println();

  long startTime = millis();
  int topSpeed = 160;

  int el = count_L - Count_Encoder_Left; 
  int er = count_R - Count_Encoder_Right;
  int tol = 17; //10
  
  //compute error
    el = count_L - Count_Encoder_Left;
    er = count_R - Count_Encoder_Right;
    
    int leftInput = el*Kp;
    int rightInput = er*Kp;
    //compute inputs with saturation
    if(el >= 0)
    {
      leftInput = (leftInput <= SLOW) ? SLOW : (leftInput >= topSpeed) ? topSpeed : leftInput;
    }
    else 
    {
      leftInput = (leftInput >= -1*SLOW) ? -1*SLOW : (leftInput <= -1*topSpeed) ? -1*topSpeed : leftInput;
    }

    if(er >= 0)
    {
      rightInput = (rightInput <= SLOW) ? SLOW : (rightInput >= topSpeed) ? topSpeed : rightInput;
    }
    else
    {
      rightInput = (rightInput >= -1*SLOW) ? -1*SLOW : (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
    }
    
  accelFromStop( (abs(leftInput)+abs(rightInput))/2 ,3);
  
  while ((abs(el) >= tol) || (abs(er) >= tol))
  {
    //compute error
    el = count_L - Count_Encoder_Left;
    er = count_R - Count_Encoder_Right;
    
    leftInput = Kp*el;
    rightInput = Kp*er;

    if(el >= 0)
    {
      leftInput = (leftInput >= topSpeed) ? topSpeed : leftInput;
    }
    else 
    {
      leftInput = (leftInput <= -1*topSpeed) ? -1*topSpeed : leftInput;
    }

    if(er >= 0)
    {
      rightInput = (rightInput >= topSpeed) ? topSpeed : rightInput;
    }
    else
    {
      rightInput = (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
    }
    
    //Serial.println(leftInput);
    //Serial.println(rightInput);
    
    //input the inputs
    mtr_ctrl.setM2Speed(leftInput);
    mtr_ctrl.setM1Speed(rightInput);

    //printSensorValues();

    if (( millis() - startTime) > 1200)
    {
      break;
    }
  }

  
  mtr_ctrl.setSpeeds(STOP,STOP);
  
}  
}

//Proposed solution to straightening out: SMS 4/5/16 11:00am
/* void adjust()		//removed inputs from original turn function
{	
  long count_L;// = Count_Encoder_Left + TURN_TICK_COUNT;
  long count_R;// = Count_Encoder_Right + TURN_TICK_COUNT;
  long right_turn_adjust = 0; // -236;  //for variations in encoder readings //-240 old
  long left_turn_adjust = 0;   //for variations in encoder readings   // 4 old
  
  //These are added to original turn function SMS 4/5/16 11:16am
  long diff = abs(Count_Encoder_Left) - abs(Count_Encoder_Right);		//Add the 2 together since one is negative and the other positive
  long tickAdjust = abs(diff) % TURN_TICK_COUNT;					//Remove turns from the difference, whatever is left we need to adjust
  Serial.println(diff);
  Serial.println(tickAdjust);
  int tickTol = 4;											//THIS WILL NEED TO BE ADJUSTED THROUGH TESTING
  char adjustDirection;
  if ( tickAdjust <= tickTol)
  {
	  adjustDirection = 'N';
  }
  else if ( diff > 0) 
  {
	  adjustDirection = 'L';
  }
  else if (diff < 0)
  {
	  adjustDirection = 'R';
  }
  
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);

  switch (adjustDirection) 
  {
    case 'L':
    {
      //for (int i=0;i<numTurns;i=i+1)
      //{
        //count_L = Count_Encoder_Left - (TURN_TICK_COUNT + left_turn_adjust);			//SMS 4/5/16 11:18am commented out original lines
        //count_R = Count_Encoder_Right + (TURN_TICK_COUNT + left_turn_adjust); 		//SMS 4/5/16 11:18am commented out original lines
		    count_L = Count_Encoder_Left - (tickAdjust + left_turn_adjust);					//SMS 4/5/16 11:18am changed TURN_TICK_COUNT to tickAdjust
        count_R = Count_Encoder_Right - (tickAdjust + left_turn_adjust); 				//SMS 4/5/16 11:18am changed TURN_TICK_COUNT to tickAdjust
        accelFromStop(SPEED,2); //left
        //while ((Count_Encoder_Left < count_L)&&(Count_Encoder_Right > count_R))
        while( Count_Encoder_Left > count_L  && Count_Encoder_Right > count_R )  //first change/idea
        {
          mtr_ctrl.setM1Speed(TURN_SPEED);      //switching to turn speed for testing, change 1
          mtr_ctrl.setM2Speed(-1*(TURN_SPEED+LEFT_MOTOR_ADJ));
        }
        mtr_ctrl.setM1Speed(STOP);
        mtr_ctrl.setM2Speed(STOP);
      //}
      break;
    }
    case 'R':
    {
      //for (int i=0; i<numTurns;i=i+1)
      //{
        //count_L = Count_Encoder_Left + (TURN_TICK_COUNT + right_turn_adjust);			//SMS 4/5/16 11:18am commented out original lines
        //count_R = Count_Encoder_Right - (TURN_TICK_COUNT + right_turn_adjust);		//SMS 4/5/16 11:18am commented out original lines
		    count_L = Count_Encoder_Left + (tickAdjust + right_turn_adjust);			//SMS 4/5/16 11:18am changed TURN_TICK_COUNT to tickAdjust
        count_R = Count_Encoder_Right + (tickAdjust + right_turn_adjust);			//SMS 4/5/16 11:18am changed TURN_TICK_COUNT to tickAdjust
        accelFromStop(SPEED,3); //right
        //while ((Count_Encoder_Left > count_L)&&(Count_Encoder_Right < count_R))
        while( Count_Encoder_Left < count_L  && Count_Encoder_Right < count_R )  //first change/idea
        {
          mtr_ctrl.setM1Speed(-1*TURN_SPEED);     //switching to turn speed for testing, change 1
          mtr_ctrl.setM2Speed(TURN_SPEED+LEFT_MOTOR_ADJ);        //switching to turn speed for testing, change 1
        }
        mtr_ctrl.setM1Speed(STOP);
        mtr_ctrl.setM2Speed(STOP);
      //}
      break;
    }
    default:
    {
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
      break;
    }
  }
  Serial.println(Count_Encoder_Left);
  Serial.println(Count_Encoder_Right);
} */

//SMS 4/5/16 1:00PM: Proposed solution to acquiring the target
void acquireTarget()
{
    //travelToTarget_US();
    travelToTarget_IR();
    align();
  
//  int acquireSlow = 35;
//	checkSensors();
//	int distance = Distance_US_F - 18;		//approximate distance to go to get closer to target
//  Serial.println(distance);
//	long tickGoal = 0;
//	bool objectReached = false;
//		
//	if (distance > 0 && distance < 80)
//  {
//		tickGoal = distance * CONV_FACTOR;	
//		travelDistance(tickGoal);				//travel closer to target
//	}
//	
//	
//	//Make sure safe on the sides
//	if (Distance_US_L > US_DANGER_THRESHOLD)			
//	{
//		turn(2,'R');			//Do a 180																						
//	}
//	else if (Distance_US_R > US_DANGER_THRESHOLD)
//	{
//		turn(2,'L');			//Do a 180
//	}
//	
//	
//	accelFromStop(acquireSlow,1);		//go in reverse slowly
//	while (objectReached == false)
//	{
//		if ((abs(Distance_IR_L-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE) && (abs(Distance_IR_R-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE)) //at target
//		{
//			mtr_ctrl.setM1Speed(STOP);
//			mtr_ctrl.setM2Speed(STOP);
//			objectReached = true;
//		}
//		else// if ((Distance_IR_L <= VICTIM_AHEAD_THRESHOLD) && (Distance_IR_L > (PICKUP_VICTIM_THRESHOLD+PICKUP_TOLERANCE)) 
//				//&& (Distance_IR_R <= (VICTIM_AHEAD_THRESHOLD)) && (Distance_IR_R > (PICKUP_VICTIM_THRESHOLD+PICKUP_TOLERANCE)))
//			{
//        accelFromStop(acquireSlow,1);
//				mtr_ctrl.setM1Speed(-1*acquireSlow);
//				mtr_ctrl.setM2Speed(-1*(acquireSlow+LEFT_MOTOR_ADJ));
//			}
//	}
// 
//	Serial.println("object reached");
// 
//	TargetAcquired = false;
//	int currentTime = millis();
//	while (TargetAcquired == false)
//	{
//		checkSensors();
//		if (abs(Distance_IR_L-Distance_IR_R) <= PICKUP_TOLERANCE)
//		{
//			//pickup target
//     Serial.println("I'm in goo");
//			grabVictim();
//			TargetAcquired = true;
//		}
//		else if((Distance_IR_L < Distance_IR_R) && (abs(Distance_IR_L-Distance_IR_R) >= PICKUP_TOLERANCE))
//		{
//			//turn right slightly
//			accelFromStop(acquireSlow,1); //get started backwards
//			mtr_ctrl.setM1Speed(-1*acquireSlow);
//			mtr_ctrl.setM2Speed(acquireSlow+LEFT_MOTOR_ADJ);
//      delay(100);
//		}
//		else if ((Distance_IR_L > Distance_IR_R) && (abs(Distance_IR_L-Distance_IR_R) >= PICKUP_TOLERANCE))
//		{
//			//turn LEFT slightly
//		  accelFromStop(acquireSlow,1); //get started backwards
//			mtr_ctrl.setM1Speed(acquireSlow);
//			mtr_ctrl.setM2Speed(-1*(acquireSlow+LEFT_MOTOR_ADJ));
//     delay(100);
//		}
//		
//		if ((millis()-currentTime) >= 5000)
//		{
//			break;
//		}	
//		
//	}
//	    mtr_ctrl.setM1Speed(STOP);
//      mtr_ctrl.setM2Speed(STOP);
}

void align()
{
  int acquireSlow = 40;
  TargetAcquired = false;
  int currentTime = millis();
  while (TargetAcquired == false)
  {
    checkSensors();
    if (abs(Distance_IR_L-Distance_IR_R) <= PICKUP_TOLERANCE)
    {
      //pickup target
      //Serial.println("I'm in goo");
      grabVictim();
      TargetAcquired = true;
    }
    else if((Distance_IR_L < Distance_IR_R) && (abs(Distance_IR_L-Distance_IR_R) >= PICKUP_TOLERANCE))
    {
      //turn right slightly
      accelFromStop(acquireSlow,1); //get started backwards
      mtr_ctrl.setM1Speed(-1*acquireSlow);
      mtr_ctrl.setM2Speed(acquireSlow+LEFT_MOTOR_ADJ);
    }
    else if ((Distance_IR_L > Distance_IR_R) && (abs(Distance_IR_L-Distance_IR_R) >= PICKUP_TOLERANCE))
    {
      //turn LEFT slightly
      accelFromStop(acquireSlow,1); //get started backwards
      mtr_ctrl.setM1Speed(acquireSlow);
      mtr_ctrl.setM2Speed(-1*(acquireSlow+LEFT_MOTOR_ADJ));
    }
    
    if ((millis()-currentTime) >= 5000)
    {
      break;
    } 
    
  }
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
}

/* void travelToTarget_US()
{
  int acquireSlow = 35;
  checkSensors();
  int distance = Distance_US_F - 18;    //approximate distance to go to get closer to target
  Serial.println(distance);
  long tickGoal = 0;
  bool objectReached = false;
    
  if (distance > 0 && distance < 80)
  {
    tickGoal = distance * CONV_FACTOR;  
    travelDistance(tickGoal);       //travel closer to target
  }
} */

void travelToTarget_IR()
{
    bool objectReached = false;
    int acquireSlow = 55;
    accelFromStop(acquireSlow,1);    //go in reverse slowly
    objectReached = (abs(Distance_IR_L-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE) || (abs(Distance_IR_R-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE);
    while (objectReached == false)
    {
      checkSensors();
      //printSensorValues();
      if ((abs(Distance_IR_L-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE) || (abs(Distance_IR_R-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE)) //at target
      {
        mtr_ctrl.setM1Speed(STOP);
        mtr_ctrl.setM2Speed(STOP);
        objectReached = true;
      }
      else// if ((Distance_IR_L <= VICTIM_AHEAD_THRESHOLD) && (Distance_IR_L > (PICKUP_VICTIM_THRESHOLD+PICKUP_TOLERANCE)) 
          //&& (Distance_IR_R <= (VICTIM_AHEAD_THRESHOLD)) && (Distance_IR_R > (PICKUP_VICTIM_THRESHOLD+PICKUP_TOLERANCE)))
        {
          //accelFromStop(acquireSlow,1);
          mtr_ctrl.setM1Speed(-1*acquireSlow);
          mtr_ctrl.setM2Speed(-1*(acquireSlow+LEFT_MOTOR_ADJ));
        }
    }
   
    //Serial.println("object reached");
}

/*
void acquireTarget_original()							//SMS 4/5/16 12:20: Changing to _original
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
    /*if ((Distance_IR_L < DANGER_ZONE) || (Distance_IR_R < DANGER_ZONE))
    {
    //STOP
    mtr_ctrl.setM1Speed(STOP);
    mtr_ctrl.setM2Speed(STOP);
    //need to recover from here, back up slowly, should check sensors
    long count_L = Count_Encoder_Left + recoveryTickCount;
    long count_R = Count_Encoder_Right + recoveryTickCount;
    //This may need to be checked to see if works, going from stop to slow speed
    accelFromStop(SLOW,0); //forward
    //Above may need to be checked to see if works, going from stop to slow speed
    while ((Count_Encoder_Left < count_L) && (Count_Encoder_Right < count_R)&&(dangerZoneReached==false))
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
    else*//* if ((abs(Distance_IR_L - PICKUP_VICTIM_THRESHOLD)<=PICKUP_TOLERANCE) && (abs(Distance_IR_R - PICKUP_VICTIM_THRESHOLD)<=PICKUP_TOLERANCE) && (abs(Distance_IR_L-Distance_IR_R) <= THRESHOLD))
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
}*/

//Test function for motors 
/*void travelDistance_timed(long numTicks)
{
  checkSensors();
  //accelFromStop(SPEED,0);
  
  mtr_ctrl.setM1Speed(TURN_SPEED);
  delay(2000);
  mtr_ctrl.setM1Speed(STOP);
  printSensorValues();
  
  mtr_ctrl.setM2Speed(TURN_SPEED + LEFT_MOTOR_ADJ);        //Change 1, remove later. Test using one motor
  delay(2000);
  mtr_ctrl.setM2Speed(STOP);
  printSensorValues();
}*/

/* void travelDistance_simple(long numTicks)
{
  long count_L = Count_Encoder_Left + numTicks;
  long count_R = Count_Encoder_Right + numTicks;
 
  
  accelFromStop(SPEED,0);
  
  while ( Count_Encoder_Left < count_L && Count_Encoder_Right < count_R )
  {
    checkSensors();

    if(Distance_US_F > US_DANGER_THRESHOLD)
    {
    
      mtr_ctrl.setM1Speed(SPEED);
      mtr_ctrl.setM2Speed(SPEED + LEFT_MOTOR_ADJ);        //Change 1, remove later. Test using one motor
    
    }
    else
    {
      mtr_ctrl.setSpeeds(STOP,STOP);
    }
  }

   //Stop at location
   mtr_ctrl.setM1Speed(STOP);
   mtr_ctrl.setM2Speed(STOP);
}  */

void danielCorrect()
{
  long count_L = Count_Encoder_Left;
  long count_R = Count_Encoder_Right;
  int Kp = 1;
  int err = 0;
  int motorSpeed = 0;
  while (abs(count_L) - abs(count_R) < 100)
  {
    
    err = abs(count_R) - abs(count_L);
    motorSpeed = (Kp*abs(err) <= SLOW) ? SLOW : (Kp*abs(err) >= TURN_SPEED) ? TURN_SPEED : Kp*abs(err);
    
    if ((err > 0) && (count_R < 0))
    {
      mtr_ctrl.setM1Speed(-1*motorSpeed);
    }
    else if ((err > 0) && (count_R > 0))
    {
      mtr_ctrl.setM1Speed(motorSpeed);
    }
    else if ((err < 0) && (count_R < 0))
    {
      mtr_ctrl.setM1Speed(motorSpeed);
    }
    else if ((err < 0) && (count_R > 0))
    {
      mtr_ctrl.setM1Speed(-1*motorSpeed);
    }
    
  }
  printSensorValues();
}
 
void travelDistance(long numTicks)
{

    Distance_US_L = checkUS_mm(DIG_PIN_US_L);
    Distance_US_R = checkUS_mm(DIG_PIN_US_R);
    Distance_US_F = checkUS_mm(DIG_PIN_US_F);
  
  int dangerCounter = 0;
  int dangerCountThresh = 3;

  int rightSpeed = STOP,leftSpeed = STOP;
  
  long count_L = Count_Encoder_Left + numTicks;
  long count_R = Count_Encoder_Right + numTicks;
  int prevLeftDist = Distance_US_L, prevRightDist = Distance_US_R;
  int dr[4] = {0,0,0,0}, dl[4] = {0,0,0,0};

  int e_r,e_l;
  int e_r_prev = 0, e_l_prev = 0;
  int de_l, de_r;
  
  //int prevErr = 0;
  accelFromStop(SPEED,0);
  
  while ( Count_Encoder_Left < count_L && Count_Encoder_Right < count_R )
  {
    Distance_US_L = checkUS_mm(DIG_PIN_US_L);
    Distance_US_R = checkUS_mm(DIG_PIN_US_R);
    Distance_US_F = checkUS_mm(DIG_PIN_US_F);
  
    //compute derivatives
    dl[0] = Distance_US_L - prevLeftDist;     
    dr[0] = Distance_US_R - prevRightDist;
  
    //here we do the filtering
    if (Distance_US_L > 2*US_HALL_THRESHOLD*10)   //change to mm
    {
      dl[0] = 0; 
    }
    if (Distance_US_R > 2*US_HALL_THRESHOLD*10)   //change to mm
    {
      dr[0] = 0;
    }
   

    //once we figure that
    
    //compute errors
//    e_r = (dr[0] + dr[1] + dr[2] + dr[3]) >> 2;
//    e_l = (dl[0] + dl[1] + dl[2] + dl[3]) >> 2; //4th order avg

      e_r = dr[0];
      e_l = dl[0];
      
    //e_r = (e_r >= 0) ? e_r : 0;
    //e_l = (e_l >= 0) ? e_l : 0;
  
    if(Distance_US_R <= US_DANGER_THRESHOLD*10)
    {
      e_r = (e_r > 0) ? 0 : e_r;
      e_l = (e_l <  0) ? 0 : e_l;
    }
  
    if(Distance_US_L <= US_DANGER_THRESHOLD*10)
    {
      e_r = (e_r < 0) ? 0 : e_r;
      e_l = (e_r > 0) ? 0 : e_l;
    }
  
    if(Distance_US_F > US_DANGER_THRESHOLD || Distance_US_F == 0)
    {
      dangerCounter = 0;
      //rightSpeed = SPEED - e_r*KPR;                       //original
      //leftSpeed = (SPEED + LEFT_MOTOR_ADJ) - e_l*KPL;     //original
      rightSpeed = SPEED - e_r*KPR;         
      leftSpeed = (SPEED + LEFT_MOTOR_ADJ) - e_l*KPL;

      rightSpeed = (rightSpeed <= SLOW) ? SLOW : rightSpeed;
      leftSpeed = (leftSpeed <= SLOW) ? SLOW : leftSpeed;
    
    }
    else
    {
      dangerCounter += 1;
      if(dangerCounter >= dangerCountThresh)
      {
        leftSpeed = STOP;
        rightSpeed = STOP;
      }
    }
    Serial.print("error ");
    Serial.print(e_l);
    Serial.print(" ");
    Serial.print(e_r);
    Serial.println();
    mtr_ctrl.setM1Speed(rightSpeed);
    mtr_ctrl.setM2Speed(leftSpeed);

    Serial.print("Speeds ");
    Serial.print(leftSpeed);
    Serial.print(" ");
    Serial.print(rightSpeed);
    Serial.println();

  
    //update vars
    prevLeftDist = Distance_US_L; 
    prevRightDist = Distance_US_R;

    dl[1] = dl[0];
    dl[2] = dl[1];
    dl[3] = dl[2];

    dr[1] = dr[0];
    dr[2] = dr[1];
    dr[3] = dr[2];

    Serial.print("d's ");
    Serial.print(dl[1]);
    Serial.print(" ");
    Serial.print(dr[1]);
    Serial.println();
    
    printSensorValues();
  }

   //Stop at location
   mtr_ctrl.setM1Speed(STOP);
   mtr_ctrl.setM2Speed(STOP);

   //danielCorrect();
} 
 
/*void travelDistance_old(long numTicks)
{
  long count_L = Count_Encoder_Left + numTicks;
  long count_R = Count_Encoder_Right + numTicks;
  //checkSensors();
  //int rightSet = Distance_US_R;

  int prevLeftDist = Distance_US_L, prevRightDist = Distance_US_R;
  int dr, dl;
  bool leftGap = false;
  //assume going forward
  
  int err,rtol,ltol;
  int prevErr = 0;
  //int tol = 3;
  accelFromStop(SPEED,0);
  
  bool useRight = Distance_US_L > (2*US_HALL_THRESHOLD);
  
  while ( Count_Encoder_Left < count_L && Count_Encoder_Right < count_R )
  {
    checkSensors();
    dl = abs(Distance_US_L - prevLeftDist);
    dr = abs(Distance_US_R - prevRightDist);
  //if leftGap becomes true then hold its value to true
    leftGap = dl > 254 || leftGap;  //approximately 10 inches in mm, refactor later    
    //check error
    //err = rightSet - Distance_US_R;
    if (useRight || leftGap)
    {
      err = US_HALL_THRESHOLD - Distance_US_R;
      if( err > 0)
      {
        //adjust rtol

        //original
        //rtol = KP*err;    //switch this to ltol
        //ltol = 0;

        //first change
        ltol = err/KP_INV;
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
        rtol = -1*err/KP_INV;

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
        rtol = err/KP_INV;
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
        ltol = -1*err/KP_INV;
      }
      else
      {
        rtol = 0;
        ltol = 0;
      }

      prevLeftDist = Distance_US_L; 
      prevRightDist = Distance_US_R;
      prevErr = err;
    }
    //end while loop
    
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
    
}*/

//int code
// 0 ---- forward
// 1 ---- backward
// 2 ---- left
// 3 ---- right
// default ---- return? 
void accelFromStop(int input , int code)
{
  //assert input > 0
  int upper = (300*input)/100;
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
 

