#include <Servo.h>   //include the servo library to control the RobotGeek Servos
#include "DualMC33926MotorShield.h"

DualMC33926MotorShield mtr_ctrl;

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

//for instructions from Pi
const char GO = 'G';
const char TURN = 'T';
const char PICKUP = 'P';
const char DROPOFF = 'D';
const char RECOVER = 'R';
const char MANIPULATE = 'M';

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

//Variables for serial
char Buffer[128];

//Variables for encoders 
const int CONV_FACTOR = 52;                     //counts per cm, 13 for cm 33 for in at 1x resolution, modified to 4x
const int TURN_TICK_COUNT = 1016;               //number of ticks for a turn (sqrt((3.875^2)+(3^2))*(pi/2)*2.54 = cm

//Distance Thresholds
const int PICKUP_VICTIM_THRESHOLD = 67;                       //pickup victim at this Distance (mm)
const int VICTIM_AHEAD_THRESHOLD = 110;                       //victim is ahead of robot, start aligning (mm)
const int PICKUP_TOLERANCE = 5;                         //use threshold - measurement and check tolerance (mm)
const int DANGER_ZONE = PICKUP_VICTIM_THRESHOLD-PICKUP_TOLERANCE;   //Stop Immediately

//Motor speeds
const int STOP = 0;  
const int SLOW = 55;     
const int SPEED = 100; //90;
const int TURN_SPEED = 130;    //added a faster speed to test zero turn
const int LEFT_MOTOR_ADJ = -4;  //SMS was -6, testing -5// -15

//Threshold variables
const long US_DANGER_THRESHOLD = 9;         //Volatile but treat as a constant

//main loop variables
int distance = 0;
int tickGoal = 0;
int turnNum = 0;
char turnDir;
char gripOption;
char instruction;

//Variables for LS7184 chips
volatile long Count_Encoder_Left = 0;                             //tick count chip 1
volatile long Count_Encoder_Right = 0;                             //tick count chip 2
	
//Variables for IR sensors
long analogValue,distanceValue;
volatile long Distance_IR_L,Distance_IR_R;  
const int THRESHOLD = 5;                                //used to check if IR sensors approx. equal (mm)

//Variables for US sensors
long pulseWidth,cm,mm;                       			//Added mm in case we use instead of cm
volatile long Distance_US_L,Distance_US_R,Distance_US_F; 	
	
void setup()
{ 
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
  Serial.begin(250000);
  mtr_ctrl.init();

}

//repeat test process 
void loop()
{
	
	if(Serial.available() > 0)
	{
		delayMicroseconds(MICRO_DELAY);
		instruction = Serial.read();
		switch (instruction) {
		  case GO:
			distance = Serial.parseInt();
			tickGoal = CONV_FACTOR * distance;
			travelDistance_Enc(tickGoal);
			taskComplete();
		  break;
		  case TURN:
			turnNum = Serial.parseInt();
			turnDir = Serial.read();
			for(int i = 0; i < turnNum; ++i)
			{
			  switch(turnDir)
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
			gripOption = Serial.read();     //send C for closed, O for open, U for up, D for down
			manipulateGripper(gripOption);
			taskComplete();
		  break;
		  case RECOVER:
			taskComplete();
		  break;
		  default: 
		  break;
		  
		}
  } 
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

void taskComplete()
{
  sprintf(Buffer,"Encoder_L: %ld,\tEncoder_R: %ld\n",Count_Encoder_Left,Count_Encoder_Right);
  Serial.print(Buffer);
}

void acquireTarget()
{
    travelToTarget_IR();
    align();
}

void travelToTarget_IR()
{
    bool objectReached = false;
    int acquireSlow = 55;
    accelFromStop(acquireSlow,1);    //go in reverse slowly
    objectReached = (abs(Distance_IR_L-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE) || (abs(Distance_IR_R-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE);
    while (objectReached == false)
    {
      checkSensors();
      
      if ((abs(Distance_IR_L-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE) || (abs(Distance_IR_R-PICKUP_VICTIM_THRESHOLD) <= PICKUP_TOLERANCE)) //at target
      {
        mtr_ctrl.setM1Speed(STOP);
        mtr_ctrl.setM2Speed(STOP);
        objectReached = true;
      }
      else
      {
        mtr_ctrl.setM1Speed(-1*acquireSlow);
        mtr_ctrl.setM2Speed(-1*(acquireSlow+LEFT_MOTOR_ADJ));
      }
    }
   
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

void align()
{
  int acquireSlow = 40;
  bool targetAcquired = false;
  int currentTime = millis();
  while (targetAcquired == false)
  {
    checkSensors();
    if (abs(Distance_IR_L-Distance_IR_R) <= PICKUP_TOLERANCE)
    {
      //pickup target
      grabVictim();
      targetAcquired = true;
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

void checkSensors()
{
  Distance_IR_L = checkIR(ANALOG_PIN_IR_L);
  Distance_IR_R = checkIR(ANALOG_PIN_IR_R);
  Distance_US_L = checkUS(DIG_PIN_US_L);
  Distance_US_R = checkUS(DIG_PIN_US_R);
  Distance_US_F = checkUS(DIG_PIN_US_F);
}

long checkIR (int pinNumIR)
{
  analogValue = analogRead(pinNumIR);                       //Read current analog value from pin
  //distanceValue = (400000-(42*analogValue))/(10*analogValue);   //conversion based on one from ref //Due conversion
  distanceValue = (264000-(42*analogValue))/(10*analogValue);   //conversion based on one from ref //Mega conversion
  return distanceValue;                                 //mm
}

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
} 

void raiseGripper()
{
  for (int i=WRIST_LOWER;i>WRIST_PICKUP;i=i-1)
  {
    largeServo.write(i);   
    delayMicroseconds(MICRO_DELAY);
  }
  delay(TIME_DELAY); 
  microServo.write(GRIP_OPEN);    
}

void turnLeft_P()
{
  long currentTime = 0;
  
  int Kp = 3;

  int leftTickCount = 950; //individually tuned for left turns
  //Set points
  int count_L = Count_Encoder_Left - leftTickCount;
  int count_R = Count_Encoder_Right + leftTickCount;

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
    
    //input the inputs
    mtr_ctrl.setM2Speed(leftInput);
    mtr_ctrl.setM1Speed(rightInput);

    if (( millis() - startTime) > 1200)
    {
      break;
    }
  }
 
  mtr_ctrl.setSpeeds(STOP,STOP);
  
}  
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

void travelDistance_Enc(int numTicks)
{
   int speed = 100;					//set speed to go
   int followerSpeed = speed;//+5;   //right motor is the follower
   int leftAdjust = -6;
   int minSpeed = speed - 10;
   int maxSpeed = speed + 10 - leftAdjust;
   
   int error = 0;
   int Kp = 5; //10;
   int dangerCounter = 0;
   int dangerCountThresh = 2;
   Count_Encoder_Left = 0;
   Count_Encoder_Right = 0;
   int leftSpeed = speed;
   checkSensors();
   accelFromStop(speed, 0);
   while (abs(Count_Encoder_Left)<numTicks)
   {
      checkSensors();
      if(Distance_US_F > US_DANGER_THRESHOLD)
      {
        dangerCounter = 0;
        error = Count_Encoder_Left - Count_Encoder_Right;
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
//          leftSpeed = STOP;
//          followerSpeed = STOP;
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
}


