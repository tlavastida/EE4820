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
const int PICKUP_VICTIM_THRESHOLD = 70;                       //pickup victim at this Distance (mm)
const int VICTIM_AHEAD_THRESHOLD = 110;                       //victim is ahead of robot, start aligning (mm)
const int PICKUP_TOLERANCE = 6;                         //use threshold - measurement and check tolerance (mm)
const int DANGER_ZONE = PICKUP_VICTIM_THRESHOLD-PICKUP_TOLERANCE;   //Stop Immediately

//Motor speeds
const int STOP = 0;  
//const int SLOW = 55;     
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
int angle;

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
			travelDistance_revision(tickGoal);
     //adjust????
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
				  //turnLeft_P();
					turnLeft_tick(730);
				  break;
				case 'R':
					turnRight_tick(723);
				  break;
				default:
				  break;
			  }
			  delay(2000);
			}
      //adjust????
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
      alignRobot();
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
  int leftDistance = Count_Encoder_Left/CONV_FACTOR; //convert ticks to cm, driving motor
  sprintf(Buffer,"DistTravelled: %d\n",leftDistance);
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
  int PICKUP_TOLERANCE = 4;
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

//SMS made new function for just the US
void checkSensors_US()
{
	Distance_US_L = checkUS(DIG_PIN_US_L);
	Distance_US_R = checkUS(DIG_PIN_US_R);
	Distance_US_F = checkUS(DIG_PIN_US_F);
}

//SMS made new function for just the IR
void checkSensors_IR()
{
	Distance_IR_L = checkIR(ANALOG_PIN_IR_L);
	Distance_IR_R = checkIR(ANALOG_PIN_IR_R);
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
  int SLOW = 60;
  
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
  int SLOW = 60;
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
      rightInput = (rightInput >= -1*SLOW) ? -1*SLOW : (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
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

/*
void turnLeft_Experiment()
{
  int slow = 65;
  int top = 90;
  int Kp = 10;

  int Count_R = 1016;
  int Count_L = -1016;
  
  Count_Encoder_Left = 0;
  Count_Encoder_Right = 0;

  
}
*/

void turnLeft_P_city(int leftTickCount)
{
  int SLOW = 65;
  int Kp = 30;// 20;

  //int leftTickCount = 1016; // 950; //individually tuned for left turns     //commented out to give function tick count
  //Set points
  int count_L = Count_Encoder_Left - leftTickCount;
  int count_R = Count_Encoder_Right + leftTickCount;

  long startTime = millis();

  int topSpeed = 90;     //150

  int el = count_L - Count_Encoder_Left; 
  int er = count_R - Count_Encoder_Right;
  int tol = 15;
  
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
      //leftInput = (leftInput <= SLOW) ? SLOW : (leftInput >= topSpeed) ? topSpeed : leftInput;
      leftInput = (leftInput >= topSpeed) ? topSpeed : leftInput;
    }
    else 
    {
      //leftInput = (leftInput >= -1*SLOW) ? -1*SLOW :(leftInput <= -1*topSpeed) ? -1*topSpeed : leftInput;
      leftInput = (leftInput <= -1*topSpeed) ? -1*topSpeed : leftInput;
    }

    if(er >= 0)
    {
      //rightInput = (rightInput <= SLOW) ? SLOW : (rightInput >= topSpeed) ? topSpeed : rightInput;
      rightInput = (rightInput >= topSpeed) ? topSpeed : rightInput;
    }
    else
    {
      //rightInput = (rightInput <= -1*SLOW) ? -1*SLOW : (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
      rightInput = (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
    }
    
    
    //input the inputs
    mtr_ctrl.setM2Speed(leftInput);
    mtr_ctrl.setM1Speed(rightInput);

    //printSensorValues();
    if (( millis() - startTime) > 1200)
    {
      break;
    }
    Serial.print("Encoder Left: ");
  Serial.print(Count_Encoder_Left);
  Serial.print(" Encoder Right: ");
  Serial.print(Count_Encoder_Right);
  Serial.println();
  }

  //delay(1000);
  mtr_ctrl.setSpeeds(STOP,STOP);
  
}

void turnRight_P_city(int rightTickCount)
{
  int SLOW = 60;
  int Kp = 2; //1
  //1016,900,958,929,
  //int rightTickCount = 943; //individually tuned for right turns    //commented out to give function tick count
  //Set points
  int count_L = Count_Encoder_Left + rightTickCount;
  int count_R = Count_Encoder_Right - rightTickCount;

  long startTime = millis();
  int topSpeed = 140;     //160 orig

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
      rightInput = (rightInput >= -1*SLOW) ? -1*SLOW : (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
    }
    
    //input the inputs
    mtr_ctrl.setM2Speed(leftInput);
    mtr_ctrl.setM1Speed(rightInput);

    if (( millis() - startTime) > 1200)
    {
      break;
    }
  }
  //delay(1000);
  mtr_ctrl.setSpeeds(STOP,STOP);
  
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

//void travelDistance_Enc(int numTicks)
//{
//   int speed = 100;					//set speed to go
//   int followerSpeed = speed;//+5;   //right motor is the follower
//   int leftAdjust = -6;
//   int minSpeed = speed - 10;
//   int maxSpeed = speed + 10 - leftAdjust;
//   
//   int error = 0;
//   int Kp = 5; //10;
//   int dangerCounter = 0;
//   int dangerCountThresh = 2;
//   Count_Encoder_Left = 0;
//   Count_Encoder_Right = 0;
//   int leftSpeed = speed;
//   checkSensors();
//   accelFromStop(speed, 0);
//   while (abs(Count_Encoder_Left)<numTicks)
//   {
//      checkSensors();
//      if(Distance_US_F > US_DANGER_THRESHOLD)
//      {
//        dangerCounter = 0;
//        error = Count_Encoder_Left - Count_Encoder_Right;
//        followerSpeed += error/Kp;
//        if(followerSpeed >= maxSpeed) 
//          followerSpeed = maxSpeed;
//        else if(followerSpeed <= minSpeed) 
//          followerSpeed = minSpeed; 
//      }
//      else
//      {
//        dangerCounter += 1;
//        if(dangerCounter >= dangerCountThresh)
//        {
////          leftSpeed = STOP;
////          followerSpeed = STOP;
//          //maybe break?
//          break;
//        } 
//      }
//      mtr_ctrl.setM2Speed(leftSpeed+leftAdjust);
//      mtr_ctrl.setM1Speed(followerSpeed);//+8); 
//      delay(100);    //100-60 
//   }
//   mtr_ctrl.setM2Speed(STOP);
//   mtr_ctrl.setM1Speed(STOP); 
//}

void travelDistance_Enc_Steve(int numTicks)						//Added to test
{
   int speed = 100;          //set speed to go
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
   
   int prevL = 0;
   int prevR = 0;
   int diff = 0;
   int hallThreshold = 5;		//cm

checkSensors_US();
prevL = Distance_US_L;
prevR = Distance_US_R;

	if(numTicks > 1820)					//35*52
	{
		
		{
			while (abs(Count_Encoder_Left)<numTicks)
			{
				checkSensors_US();
				if (Distance_US_F > US_DANGER_THRESHOLD)
				{
					dangerCounter = 0;
					if (Distance_US_L < Distance_US_R)
					{
						diff = (Distance_US_L - prevL);
						error = abs(hallThreshold - Distance_US_L);
						Serial.print("diff = ");
						Serial.print(diff);
						Serial.print(" err = ");
						Serial.println(error);
						
					}
					else if (Distance_US_R < Distance_US_L)
					{
						diff = (Distance_US_R - prevR) * -1;
						error = abs(hallThreshold - Distance_US_R);
						Serial.print("diff = ");
						Serial.print(diff);
						Serial.print(" err = ");
						Serial.println(error);
					}
          else
          {
            error = 0;
          }
					followerSpeed += error*diff;
					Serial.print("follower = ");
					Serial.println(followerSpeed);
					mtr_ctrl.setM2Speed(leftSpeed+leftAdjust);
					mtr_ctrl.setM1Speed(followerSpeed);//+8); 
				}
				else
				{
					dangerCounter += 1;
					if(dangerCounter >= dangerCountThresh)
					{
						break;
					} 
				}
			}
			mtr_ctrl.setM2Speed(STOP);
			mtr_ctrl.setM1Speed(STOP); 
		}
	}
	else 
	{
	//do the encoder thing if the US are okay
		 while (abs(Count_Encoder_Left)<numTicks)
		 {
			checkSensors_US();
			
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
}

//CURRENT VERSION THAT WE WERE USING			
//SMS backup for current version of travel using encoders 4/11/16
void travelDistance_Enc(int numTicks)
{
   int speed = 100;          //set speed to go
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

   checkSensors();
//   int prevL = Distance_IR_L;
//   int prevR = Distance_IR_R;
//   int avgL  = 0;
//   int avgR = 0;
   
   //do the encoder thing if the US are okay
     while (abs(Count_Encoder_Left)<numTicks)
     {
        checkSensors();
//		avgL = (Distance_US_L + prevL)/2;
//		avgR = (Distance_US_R + prevR)/2;
        if(Distance_US_F > US_DANGER_THRESHOLD)
        {
          if (Distance_US_L <= 6)
          {
            error = 6 - Distance_US_L;
            followerSpeed -= error;
          }
          else if (Distance_US_R <= 6)
          {
            error = 6 - Distance_US_R;
            followerSpeed += error;
          }
          else
          {
            dangerCounter = 0;
            error = Count_Encoder_Left - Count_Encoder_Right;
            followerSpeed += error/Kp;
            if(followerSpeed >= maxSpeed) 
              followerSpeed = maxSpeed;
            else if(followerSpeed <= minSpeed) 
              followerSpeed = minSpeed; 
            else
            {
              dangerCounter += 1;
              if(dangerCounter >= dangerCountThresh)
              {
                break;
              } 
            }
          }
          mtr_ctrl.setM2Speed(leftSpeed+leftAdjust);
          mtr_ctrl.setM1Speed(followerSpeed);//+8); 
          delay(100);    //100-60 
//		  prevL = Distance_US_L;
//		  prevR = Distance_US_R;		  
       }
       
     }//end of while loop
     
     mtr_ctrl.setM2Speed(STOP);
     mtr_ctrl.setM1Speed(STOP); 
}

int dangerCorrect(int counter)
{
  int Kp = 2;
  int err = 0;
  int dangerThreshUS = 6;  //cm
  int countMin = 30;
  if (counter > countMin)
  {
    checkSensors(); 
    
    if ((Distance_US_L <= dangerThreshUS) && (Distance_US_L != 0))
    {
      err = Kp*(dangerThreshUS-Distance_US_L);
    }
    else if ((Distance_US_R <= dangerThreshUS) && (Distance_US_R != 0))
    {
      err = Kp*(dangerThreshUS-Distance_US_R);
    }
    return err;
  }
} 

//void turnLeft_tick()
//{
//  Count_Encoder_Right = 0;
//  Count_Encoder_Left = 0;
//  int tickGoal = 1016;
//  //int base = 130;
//  int Kp = 2;
//
//  int leftSpeed = 0;
//  int rightSpeed = 0;
//  int topSpeed = 100;
//  int tol = 10;
//
//  int error = tickGoal - Count_Encoder_Right;
//
//  while( abs(error) > tol )
//  {
//    rightSpeed = error*Kp;
//    rightSpeed = (leftSpeed >= topSpeed) ? topSpeed : leftSpeed;
//
//    leftSpeed = 0-rightSpeed;
//
//    mtr_ctrl.setM2Speed(leftSpeed);
//    mtr_ctrl.setM1Speed(rightSpeed);
//    delay(10);
//
//    error = tickGoal - Count_Encoder_Right;
//    
//  }
//
//  mtr_ctrl.setSpeeds(STOP,STOP);
//  
////  accelFromStop(speed,2);
////  while (Count_Encoder_Right < tickGoal)
////  {
////     mtr_ctrl.setM2Speed(-1*speed);
////     mtr_ctrl.setM1Speed(speed); 
////  }
////  delay(100);
////  mtr_ctrl.setM1Speed(STOP);
////  mtr_ctrl.setM2Speed(STOP);
//}

void turnRight_tick(int tickGoal)
{
  Count_Encoder_Left =0;
  Count_Encoder_Right =0;
  //int tickGoal = 900;
  int speed = 130;
  //accelFromStop(speed,3);
  while (Count_Encoder_Left < tickGoal)
  {
     mtr_ctrl.setM2Speed(speed);
     mtr_ctrl.setM1Speed(-1*speed); 
  }
  delay(100);
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);
}

void turnLeft_tick(int tickGoal)
{
  Count_Encoder_Left = 0;
  Count_Encoder_Right = 0;
  //int tickGoal = 1016;  //original 1016
  int speed = 130;
  //accelFromStop(speed,2);
  while (Count_Encoder_Right < tickGoal)
  {
     mtr_ctrl.setM2Speed(-1*speed);
     mtr_ctrl.setM1Speed(speed); 
  }
  delay(100);
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);
}

void alignRobot()
{
  int tol = 1;          //mm
  int goTicks = 0;
  int distanceAvg_L = 0;
  int distanceAvg_R = 0;
  int adjustIR_R = -2;

  checkSensors_IR();
  while (abs(Distance_IR_L - Distance_IR_R) > tol)
  {
    for (int i = 0;i<10;i++)
    {
      checkSensors_IR();
      distanceAvg_L += Distance_IR_L;
      distanceAvg_R += Distance_IR_R + adjustIR_R;
    } 
  
    distanceAvg_L = distanceAvg_L/10;
    distanceAvg_R = distanceAvg_R/10;
    
    bool thresh = abs(distanceAvg_L - distanceAvg_R) <= tol;
  
      if (distanceAvg_L < distanceAvg_R && !thresh)
      {
        //adjust left
        goTicks = (64687*(distanceAvg_R-distanceAvg_L))/(101*100);
        turnLeft_P_city(goTicks);
      }
      else if (distanceAvg_R < distanceAvg_L && !thresh)
      {
        //adjust right
    	  goTicks = (64687*(distanceAvg_L-distanceAvg_R))/(101*100);
        turnRight_P_city(goTicks);
      }
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
      delay(1000);
      distanceAvg_L = 0;
      distanceAvg_R = 0;
  }
  
  checkSensors_IR(); 
  Serial.print("Left = ");
  Serial.print(Distance_IR_L);
  Serial.print("Right = ");
  Serial.println(Distance_IR_R);
}

void travelDistance_revision(int numTicks)
{
	int err_r, err_l;
	int Kp = 2;
	int error = 0;

	int dl, dr;
	int l[2] = {0,0};
	int r[2] = {0,0};

	int travelSpeed = 90;  //Or whatever we want to move at
	int leftMotorIn = travelSpeed, rightMotorIn = travelSpeed;

	Count_Encoder_Left = 0;
	Count_Encoder_Right = 0;


	checkSensors_US();  //Or whatever this is supposed to be
	l[1] = Distance_US_L;
	r[1] = Distance_US_R;

	while(Count_Encoder_Left < numTicks)
	{
		//get current measurement
		checkSensors_US();
		l[0] = Distance_US_L;
		r[0] = Distance_US_R;

		//compute differences
		dl = l[0] - l[1];
		dr = r[0] - r[1];

		//compute errors
		//error = target - actual
		err_l = 0 - dl;
		err_r = 0 - dr;


		if(Distance_US_L < Distance_US_R) //left is closer, use the left, likely to be more reliable
		{
			error = 0 - err_l;
		}
		else if(Distance_US_R < Distance_US_L) //right is closer, use the right, likely to be more reliable
		{
			error = err_r;
		}
		else
		{
			error = 0; //if R = L then we are in the middle, maybe don't adjust?
		}

		rightMotorIn += error*Kp;

		mtr_ctrl.setM2Speed(leftMotorIn);
		mtr_ctrl.setM1Speed(rightMotorIn);


		//update previous measurement
		l[1] = l[0];
		r[1] = r[0];

	}

	mtr_ctrl.setSpeeds(STOP,STOP);

} 
