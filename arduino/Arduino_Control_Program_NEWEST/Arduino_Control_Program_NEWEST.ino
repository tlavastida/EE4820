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

//declare pins for Bottom Infrared sensors 
const int ANALOG_PIN_IR_L_B = A8;     
const int ANALOG_PIN_IR_R_B = A9;  

//declare pins for Top Infrared sensors 
const int ANALOG_PIN_IR_L_T = A10;     
const int ANALOG_PIN_IR_R_T = A11; 

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
volatile long Distance_IR_L_B,Distance_IR_R_B;        //These are for the bottom IR sensors
volatile long Distance_IR_L_T,Distance_IR_R_T;        //These are for the top IR sensors
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

//Distance Thresholds
const int PICKUP_VICTIM_THRESHOLD = 66;                       //pickup victim at this Distance (mm) //original value 70
const int VICTIM_AHEAD_THRESHOLD = 110;                       //victim is ahead of robot, start aligning (mm)
const int PICKUP_TOLERANCE = 6;                         //use threshold - measurement and check tolerance (mm)
const int DANGER_ZONE = PICKUP_VICTIM_THRESHOLD-PICKUP_TOLERANCE;   //Stop Immediately

//Variables for LS7184 chips
volatile long Count_Encoder_L = 0;                             //tick count chip 1
volatile long Count_Encoder_R = 0;                             //tick count chip 2

//loop variables
int tickGoal = 0; 
int distanceTravelled = 0;
char cmd;
long num = 0;
char mod;

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
        cmd = Serial.read();
        delay(20);
        num = Serial.parseInt();
        delay(20);
        mod = Serial.read();
        delay(20);

        cmd = (cmd == 'Y') ? 'Z':cmd;
        num = (num == 0) ? 0:num;
        mod = (mod == 'Y') ? 'Z':mod;
        
        switch (cmd)
        {
          case 'I':
            checkSensors_IR_B();
            printIRsensorValues_B();
            taskComplete(0);
          break;
          case 'J':
            checkSensors_IR_T();
            printIRsensorValues_T();
            taskComplete(0);
          break;
          case 'U':
            checkSensors_US();
            printUSsensorValues();
            taskComplete(0);
          break;
          case 'M':
            manipulateGripper(mod);
            taskComplete(0);
          break;
          case 'T':
              for(int i = 0; i < num; ++i)
              {
                  switch(mod)
                  {
                    case 'L':
                      turn_L(724);    //  Don't mess this number up, fully charged turn
                      break;
                    case 'R':
                      turn_R(724);    //  Don't mess this number up, fully charged turn
                      break;
                    default:
                      break;
                  }//switch end
                  delay(1000);
              }//for loop end
            taskComplete(0);
            delay(2000);
          break;
          case 'G':
            if (num > 0)
            {
              tickGoal = CONV_FACTOR * num;  //int
              distanceTravelled = travelDistance_Enc(tickGoal);
            }
            else
            {
              distanceTravelled = 0;
            }
            taskComplete(distanceTravelled);
            distanceTravelled = 0;
          break;
          case 'R':
            alignRobot();
            taskComplete(0);
          break;
          case 'P':
            travelToTarget_IR();
            delay(2000);
            align();
            distanceTravelled = Count_Encoder_L/CONV_FACTOR;
            taskComplete(distanceTravelled);
            distanceTravelled = 0;
          break;
          default:
            taskComplete(0);
          break;
        }//end switch

        cmd = 'Y';
        num = 0;
        mod = 'Y';
        
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

void printIRsensorValues_B()
{
  Serial.print(Distance_IR_L_B);
  Serial.print(" ");
  Serial.print(Distance_IR_R_B);
  Serial.println();
}

void printIRsensorValues_T()
{
  Serial.print(Distance_IR_L_T);
  Serial.print(" ");
  Serial.print(Distance_IR_R_T);
  Serial.println();
}

//Use this to measure the IR sensor distance
void checkSensors_IR_B()
{
  Distance_IR_L_B = checkIR(ANALOG_PIN_IR_L_B);
  Distance_IR_R_B = checkIR(ANALOG_PIN_IR_R_B);
}

void checkSensors_IR_T()
{
  Distance_IR_L_T = checkIR(ANALOG_PIN_IR_L_T);
  Distance_IR_R_T = checkIR(ANALOG_PIN_IR_R_T);
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
   int leftAdjust = -7;
   int minSpeed = speed - 10;
   int maxSpeed = speed + 10 + leftAdjust;
   
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
// BACKWARD ---- 1
// LEFT     ---- 2
// RIGHT    ---- 3
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

void alignRobot()
{
  int tol = 1;          //mm
  int goTicks = 0;
  int distanceAvg_L = 0;
  int distanceAvg_R = 0;
  int upperThresh = 110;

  checkSensors_IR_T();

  for (int i = 0;i<10;i++)
      {
        checkSensors_IR_T();
        distanceAvg_L += Distance_IR_L_T;
        distanceAvg_R += Distance_IR_R_T;
      } 
    
  distanceAvg_L = distanceAvg_L/10;
  distanceAvg_R = distanceAvg_R/10;
  
  if (distanceAvg_L < upperThresh && distanceAvg_R < upperThresh)
  {
    while (abs(Distance_IR_L_T - Distance_IR_R_T) > tol)
    {
      for (int i = 0;i<10;i++)
      {
        checkSensors_IR_T();
        distanceAvg_L += Distance_IR_L_T;
        distanceAvg_R += Distance_IR_R_T;
      } 
    
      distanceAvg_L = distanceAvg_L/10;
      distanceAvg_R = distanceAvg_R/10;
      
      bool thresh = abs(distanceAvg_L - distanceAvg_R) <= tol;
    
        if (distanceAvg_L < distanceAvg_R && !thresh)
        {
          //adjust left
          goTicks = (64687*(distanceAvg_R-distanceAvg_L))/(101*100);
          turn_L_P(goTicks);
        }
        else if (distanceAvg_R < distanceAvg_L && !thresh)
        {
          //adjust right
          goTicks = (64687*(distanceAvg_L-distanceAvg_R))/(101*100);
          turn_R_P(goTicks);
        }
        mtr_ctrl.setM1Speed(STOP);
        mtr_ctrl.setM2Speed(STOP);
        delay(1000);
        distanceAvg_L = 0;
        distanceAvg_R = 0;
    }//end while
  }//end if
  checkSensors_IR_T(); 
  printIRsensorValues_T();
}

void turn_L_P(int leftTickCount)
{
  int SLOW = 65;
  int Kp = 30;// 20;

  //Set points
  int count_L = Count_Encoder_L - leftTickCount;
  int count_R = Count_Encoder_R + leftTickCount;

  long startTime = millis();

  int topSpeed = 90;     //150

  int el = count_L - Count_Encoder_L; 
  int er = count_R - Count_Encoder_R;
  int tol = 15;
  
  //compute error
    el = count_L - Count_Encoder_L;
    er = count_R - Count_Encoder_R;
    
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
    el = count_L - Count_Encoder_L;
    er = count_R - Count_Encoder_R;
    
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

    if (( millis() - startTime) > 1200)
    {
      break;
    }
  }
  mtr_ctrl.setSpeeds(STOP,STOP);
  
}

void turn_R_P(int rightTickCount)
{
  int SLOW = 60;
  int Kp = 2; //1
 
  //Set points
  int count_L = Count_Encoder_L + rightTickCount;
  int count_R = Count_Encoder_R - rightTickCount;

  long startTime = millis();
  int topSpeed = 140;     //160 orig

  int el = count_L - Count_Encoder_L; 
  int er = count_R - Count_Encoder_R;
  int tol = 17; //10
  
  //compute error
    el = count_L - Count_Encoder_L;
    er = count_R - Count_Encoder_R;
    
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
    el = count_L - Count_Encoder_L;
    er = count_R - Count_Encoder_R;
    
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

void travelToTarget_IR()
{
    bool objectReached = false;
    int acquireSlow = 70;
    int distanceAvg_L = 0;
    int distanceAvg_R = 0;
    int thresh = 68;

    for (int i = 0;i<5;i++)
      {
        checkSensors_IR_B();
        distanceAvg_L += Distance_IR_L_B;
        distanceAvg_R += Distance_IR_R_B;
      } 
    
      distanceAvg_L = distanceAvg_L/5;
      distanceAvg_R = distanceAvg_R/5;
    
    accelFromStop(acquireSlow,BACKWARD);    //go in reverse slowly
    objectReached = (abs(distanceAvg_L-thresh) <= PICKUP_TOLERANCE) || (abs(distanceAvg_R-thresh) <= PICKUP_TOLERANCE);
    while (objectReached == false)
    {
      distanceAvg_L = 0;
      distanceAvg_R = 0;
      for (int i = 0;i<5;i++)
      {
        checkSensors_IR_B();
        distanceAvg_L += Distance_IR_L_B;
        distanceAvg_R += Distance_IR_R_B;
      } 
    
      distanceAvg_L = distanceAvg_L/5;
      distanceAvg_R = distanceAvg_R/5;
      
      if ((abs(distanceAvg_L-thresh) <= PICKUP_TOLERANCE) || (abs(distanceAvg_R-thresh) <= PICKUP_TOLERANCE)) //at target
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
      
    }//end while
    mtr_ctrl.setM1Speed(STOP);
    mtr_ctrl.setM2Speed(STOP);
}

void align()
{
  int acquireSlow = 70;
  bool targetAcquired = false;
  int tol = 5;
  int currentTime = millis();
  int distanceAvg_L = 0;
  int distanceAvg_R = 0;
  
  while (targetAcquired == false)
  {
    distanceAvg_L = 0;
    distanceAvg_R = 0;
    for (int i = 0;i<5;i++)
      {
        checkSensors_IR_B();
        distanceAvg_L += Distance_IR_L_B;
        distanceAvg_R += Distance_IR_R_B;
      } 
    
      distanceAvg_L = distanceAvg_L/5;
      distanceAvg_R = distanceAvg_R/5;
    
    if ((abs(PICKUP_VICTIM_THRESHOLD-distanceAvg_L) <= tol)&& (abs(PICKUP_VICTIM_THRESHOLD-distanceAvg_R)<= tol)) 
    {
      //pickup target
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
      delay(500);
      manipulateGripper('D');
      delay(500);
      manipulateGripper('C');
      delay(500);
      manipulateGripper('U');
      delay(500);
      targetAcquired = true;
    }
    else if((distanceAvg_L < distanceAvg_R) && (abs(distanceAvg_L-distanceAvg_R) >= tol))
    {
      //turn right slightly
      accelFromStop(acquireSlow,RIGHT); //get started backwards
      mtr_ctrl.setM1Speed(-1*acquireSlow);
      mtr_ctrl.setM2Speed(acquireSlow+LEFT_MOTOR_ADJ);
    }
    else if ((distanceAvg_L > distanceAvg_R) && (abs(distanceAvg_L-distanceAvg_R) >= tol))
    {
      //turn LEFT slightly
      accelFromStop(acquireSlow,LEFT); //get started backwards
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

