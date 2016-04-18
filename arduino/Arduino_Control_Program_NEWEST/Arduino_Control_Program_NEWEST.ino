#include <Servo.h>   //include the servo library to control the RobotGeek Servos
#include "DualMC33926MotorShield.h"

DualMC33926MotorShield mtr_ctrl;

Servo microServo;   //create an servo object for the 9g FT-FS90MG micro servo
Servo largeServo;   //create an servo object for the RobotGeek 180 degree servo

//*********************************************************************************
//                PIN ASSIGNMENTS
//*********************************************************************************
//declare pins for the gripper servos
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

//Variables for IR sensors
long analogValue,distanceValue;
volatile long Distance_IR_L,Distance_IR_R;  
const int THRESHOLD = 5;                         //used to check if IR sensors approx. equal (mm)

//Variables for US sensors
long pulseWidth,cm;                             
volatile long Distance_US_L,Distance_US_R,Distance_US_F; 
const long US_DANGER_THRESHOLD = 9;         //Volatile but treat as a constant

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

//Variables for encoders 
const int CONV_FACTOR = 52;                     //counts per cm, 13 for cm 33 for in at 1x resolution, modified to 4x
const int TURN_TICK_COUNT = 1016;               //number of ticks for a turn (sqrt((3.875^2)+(3^2))*(pi/2)*2.54 = cm

//Motor speeds
const int STOP = 0;       
const int SPEED = 110; 
const int TURN_SPEED = 130;    
const int LEFT_MOTOR_ADJ = -4;  

//Accel from stop constants
const int FORWARD = 0;
const int BACKWARD = 1;
const int LEFT = 2;
const int RIGHT = 3;

//Variables for LS7184 chips
volatile long Count_Encoder_L = 0;                             //tick count chip 1
volatile long Count_Encoder_R = 0;                             //tick count chip 2

//loop variables
char instruction;
char gripOption;
long turnNum = 0;
char turnDir;
long distance = 0;
int tickGoal = 0; 
int distanceTravelled = 0;

//Variables for serial
char Buffer[128];

void setup() {
  Serial.begin(115200);
  delay(1000); 
  microServo.attach(MICRO_SERVOPIN);
  microServo.write(GRIP_OPEN);        // sets the gripper servo position to closed 
  largeServo.attach(LARGE_SERVOPIN);
  largeServo.write(WRIST_PICKUP);     // sets the wrist servo position to travel position
  pinMode(DIG_PIN_CLK_R,INPUT);     //changing DIG_PIN_CLK_1 to say right
  pinMode(DIG_PIN_DIR_R,INPUT);     //changing DIG_PIN_DIR_1 to say right
  attachInterrupt(digitalPinToInterrupt(DIG_PIN_CLK_R),encoderInterruptRight,RISING);
  pinMode(DIG_PIN_CLK_L,INPUT);       //changing DIG_PIN_CLK_2 to say left
  pinMode(DIG_PIN_DIR_L,INPUT);       //changing DIG_PIN_DIR_2 to say left
  attachInterrupt(digitalPinToInterrupt(DIG_PIN_CLK_L),encoderInterruptLeft,RISING);
  mtr_ctrl.init();
}

void loop() {
  // put your main code here, to run repeatedly:
    if(Serial.available() > 0)
    {
        delay(20);
        instruction = Serial.read();
        switch (instruction)
        {
          case 'I':
            checkSensors_IR();
            printIRsensorValues();
            taskComplete(0);
          break;
          case 'U':
            checkSensors_US();
            printUSsensorValues();
            taskComplete(0);
          break;
          case 'M':
            delay(20);
            gripOption = Serial.read();
            manipulateGripper(gripOption);
            taskComplete(0);
          break;
          case 'L':
            Count_Encoder_L = 0;
            Count_Encoder_R = 0;
            mtr_ctrl.setM2Speed(SPEED+LEFT_MOTOR_ADJ);
            delay(2000);
            mtr_ctrl.setM2Speed(STOP);
            taskComplete(Count_Encoder_L/CONV_FACTOR);
          break;
          case 'R':
            Count_Encoder_L = 0;
            Count_Encoder_R = 0;
            mtr_ctrl.setM1Speed(SPEED);
            delay(2000);
            mtr_ctrl.setM1Speed(STOP);
            taskComplete(Count_Encoder_R/CONV_FACTOR);
          break;
          case 'B':
            Count_Encoder_L = 0;
            Count_Encoder_R = 0;
            mtr_ctrl.setM1Speed(SPEED);
            mtr_ctrl.setM2Speed(SPEED+LEFT_MOTOR_ADJ);
            delay(2000);
            mtr_ctrl.setM1Speed(STOP);
            mtr_ctrl.setM2Speed(STOP);
            taskComplete((Count_Encoder_R+Count_Encoder_L)/(2*CONV_FACTOR));
          break;
          case 'T':
              delay(20);
              turnNum = Serial.parseInt();
              delay(20);
              turnDir = Serial.read();
              if ((turnNum > 0 ) && ((turnDir == 'L')||(turnDir == 'R'))) 
              {
                for(int i = 0; i < turnNum; ++i)
                {
                    switch(turnDir)
                    {
                      case 'L':
                        turn_L(701);
                        break;
                      case 'R':
                        turn_R(701);
                        break;
                      default:
                        break;
                    }//switch end
                    delay(1000);
                }//for loop end
              }//if end
              turnNum = 0;
              turnDir = '0';
              taskComplete(0);
              delay(2000);
          break;
          case 'G':
            delay(20);
            distance = Serial.parseInt();       //make sure the variable is a long
            if (distance > 0)
            {
              tickGoal = CONV_FACTOR * distance;  //int
              distanceTravelled = travelDistance_Enc(tickGoal);
            }
            else
            {
              distanceTravelled = 0;
            }
            taskComplete(distanceTravelled);
            distanceTravelled = 0;
          break;
          default:
            taskComplete(0);
          break;
        }//end switch
    }//end serial check if
    delay(1000);
}

void taskComplete(int distance)
{
  Serial.flush();
  sprintf(Buffer,"DistTravelled: %d\n",distance);
  Serial.print(Buffer);
  Serial.flush();
}

//For debugging, remove later
void printUSsensorValues()
{
  Serial.print(Distance_US_L);
  Serial.print(" ");
  Serial.print(Distance_US_R);
  Serial.print(" ");
  Serial.print(Distance_US_F);
  Serial.println();
}

void printIRsensorValues()
{
  Serial.print(Distance_IR_L);
  Serial.print(" ");
  Serial.print(Distance_IR_R);
  Serial.println();
}

//Use this to measure the IR sensor distance
void checkSensors_IR()
{
  Distance_IR_L = checkIR(ANALOG_PIN_IR_L);
  Distance_IR_R = checkIR(ANALOG_PIN_IR_R);
}

//This calculates the distance based on the analog value read
long checkIR (int pinNumIR)
{
  analogValue = analogRead(pinNumIR);                         //Read current analog value from pin
  distanceValue = (264000-(42*analogValue))/(10*analogValue); //conversion based on one from ref //Mega conversion
  return distanceValue;                                       //mm
}

void checkSensors_US()
{
  Distance_US_L = checkUS(DIG_PIN_US_L);
  Distance_US_R = checkUS(DIG_PIN_US_R);
  Distance_US_F = checkUS(DIG_PIN_US_F);
}

long checkUS (int pinNumUS)
{
  digitalWrite(pinNumUS,LOW);                 // make sure pin is low before pulsing
  pinMode(pinNumUS,OUTPUT);                   // set up pin to initiate pulse
  delayMicroseconds(2);                       // for 2 microseconds 
  digitalWrite(pinNumUS,HIGH);                // Start pulse
  delayMicroseconds(5);                       // for 5 microseconds
  digitalWrite(pinNumUS,LOW);                 // set pin back to low to ready for return pulse
  pinMode(pinNumUS,INPUT);                    // change pin to Input mode for return pulse
  pulseWidth = pulseIn(pinNumUS,HIGH,18500);  // wait for return pulse. Timeout after 18.5 milliseconds
  cm = pulseWidth/58;                         // Convert to centimeters, use 58 for Mega
  return cm;                                  //returns cm distance
}

void manipulateGripper(char option)
{
  if (option == 'C')
  {
    microServo.write(GRIP_GRAB);              //set gripper to grab victim
  }
  else if (option == 'O')
  {
    microServo.write(GRIP_OPEN);              //set gripper to fully open
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

//Interrupt routine for LS7184 chip 1 - Code based on ref 4
void encoderInterruptLeft()
{
  Count_Encoder_L = digitalRead(DIG_PIN_DIR_L) ? Count_Encoder_L + 1: Count_Encoder_L - 1;
}

//Interrupt routine for LS7184 chip 2 - Code based on ref 4
void encoderInterruptRight()
{
  Count_Encoder_R = digitalRead(DIG_PIN_DIR_R) ? Count_Encoder_R - 1: Count_Encoder_R + 1;      
}

void turn_R(int tickGoal)
{
  Count_Encoder_L = 0;delayMicroseconds(100);
  Count_Encoder_R = 0;delayMicroseconds(100);
  int speed = 130;
  while (Count_Encoder_L < tickGoal)
  {
     mtr_ctrl.setM2Speed(speed);
     mtr_ctrl.setM1Speed(-1*speed); 
  }
  delay(100);
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);
  delay(100);
}

void turn_L(int tickGoal)
{
  Count_Encoder_L = 0;delayMicroseconds(100);
  Count_Encoder_R = 0;delayMicroseconds(100);
  int speed = 130;
  while (Count_Encoder_R < tickGoal)
  {
     mtr_ctrl.setM2Speed(-1*speed);
     mtr_ctrl.setM1Speed(speed); 
  }
  delay(100);
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);
  delay(100);
}

int travelDistance_Enc(int numTicks)
{
   int speed = 100;          //set speed to go
   int followerSpeed = speed; //right motor is the follower
   int leftAdjust = -6;
   int minSpeed = speed - 10;
   int maxSpeed = speed + 10 - leftAdjust;
   
   int error = 0;
   int Kp = 5; //10;
   int dangerCounter = 0;
   int dangerCountThresh = 2;
   Count_Encoder_L = 0;delayMicroseconds(100);
   Count_Encoder_R = 0;delayMicroseconds(100);
   int leftSpeed = speed;
   checkSensors_US();
   accelFromStop(speed, FORWARD);
   while (Count_Encoder_L <numTicks)
   {
      checkSensors_US();
      if(Distance_US_F > US_DANGER_THRESHOLD)
      {
        dangerCounter = 0;
        error = Count_Encoder_L - Count_Encoder_R;
        followerSpeed += error/Kp;
        if(followerSpeed >= maxSpeed) 
          followerSpeed = maxSpeed;
        else if(followerSpeed <= minSpeed) 
          followerSpeed = minSpeed; 
      }
      else
      {
        dangerCounter += 1;
        if(dangerCounter >= dangerCountThresh)
        {
          leftSpeed = STOP;
          followerSpeed = STOP;
          //maybe break?
          break;
        } 
      }
      mtr_ctrl.setM2Speed(leftSpeed+leftAdjust);
      mtr_ctrl.setM1Speed(followerSpeed);//+8); 
      delay(100);    //100-60 
   }
   mtr_ctrl.setM2Speed(STOP);
   mtr_ctrl.setM1Speed(STOP); 
   return Count_Encoder_L/CONV_FACTOR;
}

//int code
// FORWARD  ---- 0
// BACKWARD ---- 0
// LEFT     ---- 0
// RIGHT    ---- 0
// default ---- return? 
void accelFromStop(int input , int code)
{
  //assert input > 0
  int upper = (200*input)/100;
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
