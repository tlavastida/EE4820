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
long pulseWidth,cm,mm;                             
volatile long Distance_US_L,Distance_US_R,Distance_US_F; 
const long US_DANGER_THRESHOLD = 14;         //Volatile but treat as a constant //9

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
const int WRIST_PICKUP = 0;		//original value = 10 4/27/16
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
const long TURN_LEFT_TICK = 910;    //950 original/tested 4/26 to tune value (( 895)) with dirty course
const long TURN_RIGHT_TICK = 873;   //761 original/tested 4/26 to tune value ((862)) with dirty course

volatile bool inDanger = false;

//Variables for serial
char Buffer[128];

void setup() {
  Serial.begin(115200);
  delay(1000); 
  microServo.attach(MICRO_SERVOPIN);
  microServo.write(GRIP_GRAB);        // sets the gripper servo position to closed 
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
			//Check bottom IR sensors - for testing purposes only
          case 'I':
            checkSensors_IR_B();
            Serial.println(Distance_IR_L_B);
            Serial.println(Distance_IR_R_B);
            taskComplete(0);
          break;
		  //Check top IR sensors - for testing purposes only
          case 'J':
            checkSensors_IR_T();
            Serial.println(Distance_IR_L_T);
            Serial.println(Distance_IR_R_T);
            taskComplete(0);
          break;
		  //check US sensors - for testing purposes only
          case 'U':
            checkSensors_US();
            Serial.println(Distance_US_L);
            Serial.println(Distance_US_R);
            Serial.println(Distance_US_F);
            taskComplete(0);
          break;
		  //Manipulate gripper arm
          case 'M':
            manipulateGripper(mod);
            taskComplete(0);
          break;
		  //Turn L/R
          case 'T':
              for(int i = 0; i < num; ++i)
              {
                  switch(mod)
                  {
                    case 'L':
                      //turn_L(TURN_LEFT_TICK);    //  Don't mess this number up, fully charged turn (902)
                      turn_L_P(TURN_LEFT_TICK);      //Testing Proportional turning
                      //turn_L_P(num);
                      break;
                    case 'R':
                      //turn_R(TURN_RIGHT_TICK);    //  Don't mess this number up, fully charged turn (761)
                      turn_R_P(TURN_RIGHT_TICK);      //Testing Proportional turning
                      //turn_R_P(num);
                      break;
                    default:
                      break;
                  }//switch end
                  delay(1000);
                  alignRobot();
                  delay(1000);
              }//for loop end
            taskComplete(0);
            delay(2000);
          break;
		  //Travel - for testing, need to update to move grid squares
          case 'G':
            distanceTravelled = moveDistanceWithAdjust(num);
            taskComplete(distanceTravelled);
            distanceTravelled = 0;
          break;
		  //Recover, uses the top IR sensors to align to wall
          case 'R':
            alignRobot();
            taskComplete(0);
          break;
		  //Pickup target after travelling and aligning
          case 'P':
            travelToTarget_IR();
            delay(2000);
            align();
            distanceTravelled = Count_Encoder_L/CONV_FACTOR;
            taskComplete(distanceTravelled);
            distanceTravelled = 0;
          break;
          case 'F':
            filtered_US();
            Serial.println(Distance_US_L);
            Serial.println(Distance_US_R);
            Serial.println(Distance_US_F);
            taskComplete(0);
          break;
          case 'B':
            travelDistance_Enc(num*CONV_FACTOR);
            taskComplete(Count_Encoder_L/CONV_FACTOR);
          break;
          case 'D':
            dropVictim();
            taskComplete(0);
          break;
          case 'A':
            aboutFace();
            taskComplete(0);
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

//Send distance travelled after an action is completed
//For handshaking with the pi, mostly useful for travel distance
void taskComplete(int distance)
{
  sprintf(Buffer,"DistTravelled: %d\n",distance);
  Serial.print(Buffer);
}




//Use this to measure the bottom IR sensor distance
void checkSensors_IR_B()
{
  Distance_IR_L_B = checkIR(ANALOG_PIN_IR_L_B);
  Distance_IR_R_B = checkIR(ANALOG_PIN_IR_R_B);
}

//Use this to measure the top IR sensor distance
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

//Use this to measure the US sensor distance 
void checkSensors_US()
{
  Distance_US_L = checkUS(DIG_PIN_US_L);
  Distance_US_F = checkUS(DIG_PIN_US_F);
  Distance_US_R = checkUS(DIG_PIN_US_R);
}

//This calculates the distance for the ultrasonic readings
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
  delayMicroseconds(300);                     //short delay before using another US
  return cm;                                  // returns cm distance
}

//Manipulates the gripper arm
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

//Turn Right
void turn_R(int tickGoal)
{
  int leftMotorAdj = -11;
  Count_Encoder_L = 0;delayMicroseconds(100);
  Count_Encoder_R = 0;delayMicroseconds(100);
  int speed = 130;
  while (Count_Encoder_L < tickGoal)
  {
     mtr_ctrl.setM2Speed(speed+leftMotorAdj);
     mtr_ctrl.setM1Speed(-1*speed); 
  }
  //delay(100);         //Don't know why there was a delay here
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);
  delay(100);
}

//Turn Left
void turn_L(int tickGoal)
{
  int leftMotorAdj = -11;
  Count_Encoder_L = 0;delayMicroseconds(100);
  Count_Encoder_R = 0;delayMicroseconds(100);
  int speed = 130;
  while (Count_Encoder_R < tickGoal)
  {
     mtr_ctrl.setM2Speed(-1*(speed+leftMotorAdj));
     mtr_ctrl.setM1Speed(speed); 
  }
  //delay(100);       //Don't know why there was a delay here
  mtr_ctrl.setM1Speed(STOP);
  mtr_ctrl.setM2Speed(STOP);
  delay(100);
}

//Travel a distance based on ticks, called by the go grid units function
int travelDistance_Enc(int numTicks)
{
   int speed = 100;          //set speed to go
   int followerSpeed = speed; //right motor is the follower
   int leftAdjust = -7;
   int minSpeed = speed - 10;
   int maxSpeed = speed + 10;
   
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
          inDanger = true;
          
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
//This function ramps up the speed so that there is enough 
//power to move the robot and then decreases speed to input
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
        case FORWARD:
          mtr_ctrl.setSpeeds(i,i);
          break;
        case BACKWARD:
          mtr_ctrl.setSpeeds(-1*i,-1*i);
          break;
        case LEFT:
          mtr_ctrl.setSpeeds(i,-1*i);
          break;
        case RIGHT:
          mtr_ctrl.setSpeeds(-1*i,i);
      }
    }
    delay(20);
    for(int i = upper; i >= input; --i)
    {
       switch(code) {
        case FORWARD:
          mtr_ctrl.setSpeeds(i,i);
          break;
        case BACKWARD:
          mtr_ctrl.setSpeeds(-1*i,-1*i);
          break;
        case LEFT:
          mtr_ctrl.setSpeeds(i,-1*i);
          break;
        case RIGHT:
          mtr_ctrl.setSpeeds(-1*i,i);
      }
    }
  }
  else //If zero, stop
  {
    mtr_ctrl.setSpeeds(STOP,STOP);
  }
}

//Align the robot to a wall. Must be within a certain distance (upperThresh)
void alignRobot()
{
  int tol = 1;          //mm
  int goTicks = 0;
  int distanceAvg_L = 0;
  int distanceAvg_R = 0;
  int upperThresh = 110;
  int lowerThresh = 45;
  long startTime = millis();

  checkSensors_IR_T();

  for (int i = 0;i<4;i++)
      {
        checkSensors_IR_T();
        distanceAvg_L += Distance_IR_L_T;
        distanceAvg_R += Distance_IR_R_T;
      } 
    
  distanceAvg_L = distanceAvg_L/4;
  distanceAvg_R = distanceAvg_R/4;
  
  if (distanceAvg_L < upperThresh && distanceAvg_R < upperThresh && distanceAvg_L > lowerThresh && distanceAvg_R > lowerThresh)
  {
    while (abs(Distance_IR_L_T - Distance_IR_R_T) > tol)
    {
      for (int i = 0;i<4;i++)
      {
        checkSensors_IR_T();
        distanceAvg_L += Distance_IR_L_T;
        distanceAvg_R += Distance_IR_R_T;
      } 
    
      distanceAvg_L = distanceAvg_L/4;
      distanceAvg_R = distanceAvg_R/4;
      
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

         if(millis()-startTime > 2000)
        {
          break;
        }
    }//end while
  }//end if
  checkSensors_IR_T(); 
  
}

//Align the robot to a wall. Must be within a certain distance (upperThresh)
void alignRobot2()
{
  int tol = 10;          //mm
  int goTicks = 0;
  int distanceAvg_L = 0;
  int distanceAvg_R = 0;
  int upperThresh = 110;

  checkSensors_IR_B();

  for (int i = 0;i<4;i++)
      {
        checkSensors_IR_B();
        distanceAvg_L += Distance_IR_L_B;
        distanceAvg_R += Distance_IR_R_B;
      } 
    
  distanceAvg_L = distanceAvg_L>>2;
  distanceAvg_R = distanceAvg_R>>2;
  
  if (distanceAvg_L < upperThresh && distanceAvg_R < upperThresh)
  {
    while (abs(Distance_IR_L_B - Distance_IR_R_B) > tol)
    {
      for (int i = 0;i<4;i++)
      {
        checkSensors_IR_B();
        distanceAvg_L += Distance_IR_L_B;
        distanceAvg_R += Distance_IR_R_B;
      } 
    
      distanceAvg_L = distanceAvg_L>>2;
      distanceAvg_R = distanceAvg_R>>2;
      
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
  checkSensors_IR_B(); 
  
}

//Turn Left using proportional control
void turn_L_P(long leftTickCount)
{
  int SLOW = 70;
  int Kp = 2;// 30;
  int accelSpeed = 0;

  //Set points
  int count_L = Count_Encoder_L - leftTickCount;
  int count_R = Count_Encoder_R + leftTickCount;

  long startTime = millis();

  int topSpeed = 130;     

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

  accelFromStop( (abs(leftInput)+abs(rightInput))/2 ,LEFT);
  
  while ((abs(el) >= tol) || (abs(er) >= tol))
  {
    //compute error
    el = count_L - Count_Encoder_L;       
    er = count_R - Count_Encoder_R;      
    
    leftInput = el*Kp;                    
    rightInput = er*Kp;

    if(el >= 0)
    {
      leftInput = (leftInput <= SLOW) ? SLOW : (leftInput >= topSpeed) ? topSpeed : leftInput;
      //leftInput = (leftInput >= topSpeed) ? topSpeed : leftInput;   //original
    }
    else 
    {
      leftInput = (leftInput >= -1*SLOW) ? -1*SLOW :(leftInput <= -1*topSpeed) ? -1*topSpeed : leftInput;
      //leftInput = (leftInput <= -1*topSpeed) ? -1*topSpeed : leftInput;   //original
    }

    if(er >= 0)
    {
      rightInput = (rightInput <= SLOW) ? SLOW : (rightInput >= topSpeed) ? topSpeed : rightInput;
      //rightInput = (rightInput >= topSpeed) ? topSpeed : rightInput;    //original
    }
    else
    {
      rightInput = (rightInput >= -1*SLOW) ? -1*SLOW : (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;
      //rightInput = (rightInput <= -1*topSpeed) ? -1*topSpeed : rightInput;    //original
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

//Turn Right using proportional control
void turn_R_P(long rightTickCount)
{
  int SLOW = 70;
  int Kp = 2; //1         
 
  //Set points
  int count_L = Count_Encoder_L + rightTickCount;
  int count_R = Count_Encoder_R - rightTickCount;

  long startTime = millis();
  int topSpeed = 130;     //140 orig

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
    
  accelFromStop( (abs(leftInput)+abs(rightInput))/2 ,RIGHT);
  
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

//Travel in reverse to get to target
void travelToTarget_IR()
{
    bool objectReached = false;
    int acquireSlow = 70;
    int distanceAvg_L = 0;
    int distanceAvg_R = 0;
    int thresh = 66; 
    int tol = 1;       

    for (int i = 0;i<2;i++)
      {
        checkSensors_IR_B();
        distanceAvg_L += Distance_IR_L_B;
        distanceAvg_R += Distance_IR_R_B;
      } 
    
      distanceAvg_L = distanceAvg_L>>1;
      distanceAvg_R = distanceAvg_R>>1;
    
    accelFromStop(acquireSlow,BACKWARD);    //go in reverse slowly
    objectReached = (abs(distanceAvg_L-thresh) <= tol) || (abs(distanceAvg_R-thresh) <= tol) ;
    while (objectReached == false)
    {
      distanceAvg_L = 0;
      distanceAvg_R = 0;
      
      for (int i = 0;i<2;i++)
      {
        checkSensors_IR_B();
        distanceAvg_L += Distance_IR_L_B;
        distanceAvg_R += Distance_IR_R_B;
      } 
    
      distanceAvg_L = distanceAvg_L>>1;
      distanceAvg_R = distanceAvg_R>>1;
      
      if ((abs(distanceAvg_L-thresh) <= tol) || (abs(distanceAvg_R-thresh) <= tol)) //at target
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

//Rotate until aligned with target
void align()
{
  int acquireSlow = 100;
  bool targetAcquired = false;
  int tol = 15;
  long currentTime = millis();
  int distanceAvg_L = 0;
  int distanceAvg_R = 0;
  long smallAdj = 60;
  long ticks;
  
  while (targetAcquired == false)
  {
    distanceAvg_L = 0;
    distanceAvg_R = 0;
    for (int i = 0;i<1;i++)
      {
        checkSensors_IR_B();
        distanceAvg_L += Distance_IR_L_B;
        distanceAvg_R += Distance_IR_R_B;
      } 
    
      distanceAvg_L = distanceAvg_L;
      distanceAvg_R = distanceAvg_R;
    
    //if ((abs(PICKUP_VICTIM_THRESHOLD-distanceAvg_L) <= tol)&& (abs(PICKUP_VICTIM_THRESHOLD-distanceAvg_R)<= tol))  
    if (abs(distanceAvg_L-distanceAvg_R) <= tol)
    {
      //pickup target
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
      if ((distanceAvg_L < 68) && (distanceAvg_R < 68))
      {
        ticks = 26;
        travelDistance_Enc(ticks);
      }
      targetAcquired = true;
    }
    else if((distanceAvg_L < distanceAvg_R) && (abs(distanceAvg_L-distanceAvg_R) >= tol) && (distanceAvg_L < 110)&& (distanceAvg_R < 110))
    {
      //turn right slightly
      turn_R_P(smallAdj);
      delay(500);
      //accelFromStop(acquireSlow,RIGHT); //get started backwards
      //mtr_ctrl.setM1Speed(-1*acquireSlow);
      //mtr_ctrl.setM2Speed(acquireSlow+LEFT_MOTOR_ADJ);
    }
    else if ((distanceAvg_L > distanceAvg_R) && (abs(distanceAvg_L-distanceAvg_R) >= tol) && (distanceAvg_L < 110)&& (distanceAvg_R < 110))
    {
      //turn LEFT slightly
      turn_L_P(smallAdj);
      delay(500);
      //accelFromStop(acquireSlow,LEFT); //get started backwards
      //mtr_ctrl.setM1Speed(acquireSlow);
      //mtr_ctrl.setM2Speed(-1*(acquireSlow+LEFT_MOTOR_ADJ));
    }
    
    if ((millis()-currentTime) >= 5000)
    {
      break;
    } 
    
  }
      mtr_ctrl.setM1Speed(STOP);
      mtr_ctrl.setM2Speed(STOP);
      manipulateGripper('O');
      delay(1000);
      manipulateGripper('D');
      delay(1000);
      manipulateGripper('C');
      delay(1000);
      manipulateGripper('U');
      delay(1000);
     
}

int moveGridUnits(int numGrids)
{
  //Use function (filtered_US())for filtered US readings. L/R are mm, F is still cm 
  //Distance_US_L,Distance_US_R, and Distance_US_F are still applicable
  int fullMoves = numGrids / 3;
  int remainder = numGrids % 3;
  
  int totalDistance = 0; //cm 
  int x; //cm
  int d; //cm
  
  int ticks3Units = 1585;
  int ticks2Units = 1057;
  int ticks1Units = 528;
  
  int leftUS[2] = {0,0};
  int rightUS[2] = {0,0};
  
  int diff_L = 0;
  int diff_R = 0;
  
  checkSensors_IR_T();
  if (Distance_IR_L_T <= 110 && Distance_IR_R_T <= 110)
  {
    alignRobot();
    delay(500); //wait for bot to lose momentum
  }

  for(int i = 0; i < fullMoves; ++i) 
  {
    checkSensors_US();
    leftUS[0] = Distance_US_L;
    rightUS[0] = Distance_US_R;
    
    d = travelDistance_Enc(ticks3Units);
    
    checkSensors_US();
    leftUS[1] = Distance_US_L;
    rightUS[1] = Distance_US_R;
    
    diff_L = leftUS[1] - leftUS[0];
    diff_R = rightUS[1] - rightUS[0];

    delay(500);
    
    if (diff_L < diff_R)
    {
      x = sqrt(d*d - diff_L*diff_L);
      if(abs(diff_L) > 2)
        turn_R_P((64687*abs(diff_L))/(d*1000));
    }
    else if(diff_R <= diff_L)
    {
      x = sqrt(d*d - diff_R*diff_R);
      if(abs(diff_R) > 2)
        turn_L_P((64687*abs(diff_R))/(d*1000));
    }
    
    totalDistance += x;

    delay(500);
  }
  
  if(remainder == 2)
  {
    checkSensors_US();
    leftUS[0] = Distance_US_L;
    rightUS[0] = Distance_US_R;
    
    d = travelDistance_Enc(ticks2Units);
    
    checkSensors_US();
    leftUS[1] = Distance_US_L;
    rightUS[1] = Distance_US_R;
    
    diff_L = leftUS[1] - leftUS[0];
    diff_R = rightUS[1] - rightUS[0];

    delay(500);
    
    if (leftUS[1] < rightUS[1])
    {
      x = sqrt(d*d - diff_L*diff_L);
      //if(abs(diff_L) > 2)
        //turnRight_P_city((64687*abs(diff_L))/(d*1000));
    }
    else if(leftUS[1] >= rightUS[1])
    {
      x = sqrt(d*d - diff_R*diff_R);
      //if(abs(diff_R) > 2)
        //turnLeft_P_city((64687*abs(diff_R))/(d*1000));
    }
    
    totalDistance += x;
  }
  else if(remainder == 1)
  {
    checkSensors_US();
    leftUS[0] = Distance_US_L;
    rightUS[0] = Distance_US_R;
    
    d = travelDistance_Enc(ticks1Units);
    
    checkSensors_US();
    leftUS[1] = Distance_US_L;
    rightUS[1] = Distance_US_R;
    
    diff_L = leftUS[1] - leftUS[0];
    diff_R = rightUS[1] - rightUS[0];

    delay(500);
    
    if (leftUS[1] < rightUS[1])
    {
      x = sqrt(d*d - diff_L*diff_L);
      //if(abs(diff_L) > 2)
        //turnRight_P_city((64687*abs(diff_L))/(d*1000));
    }
    else if(leftUS[1] >= rightUS[1])
    {
      x = sqrt(d*d - diff_R*diff_R);
      //if(abs(diff_R) > 2)
        //turnLeft_P_city((64687*abs(diff_R))/(d*1000));
    }
    
    totalDistance += x;
  }
  
  return totalDistance;
}

void filtered_US()
{
  long distanceAvg_L = 0;
  long distanceAvg_R = 0;
  long distanceUS_L[4] = {0,0,0,0};
  long distanceUS_R[4] = {0,0,0,0};
  int i = 0;
  int j = 0;
  long start = millis();
  int divL = 4;
  int divR = 4;
   
  while ((i < 4) && (j < 4))
  {
    checkSensors_US_mm();
    if (Distance_US_L > 0)
    {
      distanceUS_L[i] = Distance_US_L;
      i += 1;
    }
    else
    {
      i = (i==0) ? 0:i-1;
    }
    if (Distance_US_R > 0)
    {
      distanceUS_R[j] = Distance_US_R;
      j += 1;
    }
    else 
    {
      j = (j==0) ? 0:j-1;
    }
    if ((millis() - start)>500)
    {
      break;
    }
  }//end while

  for (i = 0;i<4;i++)
  {
    if (distanceUS_L[i] == 0)
    {
      divL -= 1 ;
    }
    else
    {
      distanceAvg_L += distanceUS_L[i];
    }
    if (distanceUS_R[i] == 0)
    {
      divR -= 1;
    }
    else
    {
      distanceAvg_R += distanceUS_R[i];
    } 
  }
  
  divL = (divL < 1) ? 1:divL;
  divR = (divR < 1) ? 1:divR;
  Distance_US_L = distanceAvg_L/divL;
  Distance_US_R = distanceAvg_R/divR;
}

//Used exclusively by the go grid squares function
void checkSensors_US_mm()
{
  Distance_US_L = checkUS_mm(DIG_PIN_US_L);
  Distance_US_F = checkUS_mm(DIG_PIN_US_F);
  Distance_US_R = checkUS_mm(DIG_PIN_US_R);
}

//Used exclusively by the go grid squares function
long checkUS_mm (int pinNumUS)
{
  digitalWrite(pinNumUS,LOW);                 // make sure pin is low before pulsing
  pinMode(pinNumUS,OUTPUT);                   // set up pin to initiate pulse
  delayMicroseconds(2);                       // for 2 microseconds 
  digitalWrite(pinNumUS,HIGH);                // Start pulse
  delayMicroseconds(5);                       // for 5 microseconds
  digitalWrite(pinNumUS,LOW);                 // set pin back to low to ready for return pulse
  pinMode(pinNumUS,INPUT);                    // change pin to Input mode for return pulse
  pulseWidth = pulseIn(pinNumUS,HIGH,18500);  // wait for return pulse. Timeout after 18.5 milliseconds
  cm = (pulseWidth*10)/58;                    // Convert to centimeters, use 58 for Mega
  delayMicroseconds(300);                     //short delay before using another US
  return cm;                                  // returns cm distance
}


// input - target distance to travel in centimeters
// output - approximate distance actually travelled in centimeters
// side effects - the robot makes a series of forward movements
// and adjustments to avoid touching the left and right walls.
// 
// cases:
// 1. angle is approx 0 and it is close enough to the middle of the corridor - no adjustment
// 2. angle is > 0 and it is too close to a wall - adjust to angle 0 and make an extra adjustment to move towards center
int moveDistanceWithAdjust(int distance_cms)
{
	//variable declarations for movement
	int num_moves = 1;
	long moveDistance = 0; //cm
	long totalDistance = 0; //cm 
	long legDistance = 0;
	long remainderDistance = 0;
	long approxDistance = 0; //cm
	
	
	//variable declarations for sensors
	long leftUS[2] = {0,0};
	long rightUS[2] = {0,0};
	long diff_L = 0;
  long diff_R = 0; 
	long avgWallDistance = 0;
  long maxThresh = 100; //used to see if diff is greater than a certain amount, indicating a gap, in mms
  
  
	//variables for angle calculation
	int wallThreshold = 50; //mms       
	long angleTickCount = 0;
	
	//compute number of moves to make first
	if( distance_cms > 0 && distance_cms <= 40 )
	{
		num_moves = 1;
	}
	else if( distance_cms > 40 && distance_cms <= 80 )
	{
		num_moves = 2;
	}
	else if( distance_cms > 80 && distance_cms <= 120 )     //changing 120 to 90 SMS
	{	
		num_moves = 3;
	}
	else if( distance_cms > 120 )                          //changing 120 to 90 SMS
	{
		num_moves = 4;
	}

	//calculate distance per move
	moveDistance = distance_cms / num_moves;
	remainderDistance = distance_cms % num_moves; // should be 0,1,2, or 3 at most, maybe negligible
	
	
	//align before moving if able to.
	checkSensors_IR_T();
	if (Distance_IR_L_T <= 110 && Distance_IR_R_T <= 110)
	{
		alignRobot();
		delay(500); //wait for bot to lose momentum
	}
	
	
	//do each move
	for(int i = 0; i < num_moves; ++i)
	{
		//first get sensor readings before the move
		filtered_US(); //update the sensor readings
		leftUS[0] = Distance_US_L;
		rightUS[0] = Distance_US_R;
		
		//do the move
		legDistance = travelDistance_Enc( moveDistance * CONV_FACTOR );
		
		//get sensor readings after completing the move
		filtered_US();
		leftUS[1] = Distance_US_L;
		rightUS[1] = Distance_US_R;
		
		diff_L = leftUS[1] - leftUS[0];
		diff_R = rightUS[1] - rightUS[0];
		
		
		delay(750);
		
		if( diff_L < 0 && Distance_US_L < 300 || abs(diff_R) > maxThresh && abs(diff_L) > 150 ) //use the left side //changed 15 to 150
		{
			//compute angle to adjust by
			angleTickCount =  (64687 * abs( diff_L )) / (legDistance * 1000);            
      //angleTickCount = (abs(diff_L)>100) ? 0:angleTickCount; //testing this

			//decide which direction to turn
			if( leftUS[1] < leftUS[0] && angleTickCount <= 210 )
			{
        
				turn_R_P(angleTickCount);
			}
			else if(leftUS[1] > leftUS[0] && angleTickCount <= 210 )        //changed to right from left SMS
			{
        
				turn_L_P(angleTickCount);             //changed from L to R SMS
			}
			//if left[1] == left[0] do nothing
			
			
			approxDistance = sqrt( legDistance * legDistance - (diff_L * diff_L) / 100 );  //because legDistance is in cms and diff_L is in mms
			
		}
		else if( diff_R < 0 && Distance_US_R < 300 || abs(diff_L) > maxThresh && abs(diff_R) > 150 ) //use the right side    //added in abs(diff_R)>150
		{
			//compute angle to adjust by
			angleTickCount = (64687 * abs( diff_R )) / (legDistance * 1000);       
      //angleTickCount = (abs(diff_R)>100) ? 0:angleTickCount; //testing this

			if( rightUS[1] < rightUS[0] && angleTickCount <= 210  )
			{
       
				turn_L_P(angleTickCount);
			}
			else if(rightUS[1] > rightUS[0] && angleTickCount <= 210 )
			{
        
				turn_R_P(angleTickCount);             //changed from R to L SMS
			}
			// if right[1] == right[0] do nothing
			
			//turn towards the center if necessary
			
			
			approxDistance = sqrt( legDistance * legDistance - ( diff_R * diff_R ) / 100  );  //because legDistance is in cms and diff_R is in mms
			
		}
    else
    {
    
      approxDistance = legDistance;
    }
		//else if diff_L == 0 and diff_R == 0 then  no adjustments

    if( leftUS[1] > 140 || rightUS[1] > 140 )
    {
      
      avgWallDistance = 70;
    }
    else 
    {
      avgWallDistance = (leftUS[1] + rightUS[1]) >> 1; // (l+r)/2
    }

    if (inDanger)
    {
      inDanger = false;
      break;
    }

    //turn towards the center if necessary
    if( leftUS[1] <= wallThreshold && i+1 != num_moves ) //don't do it in the last iteration
    {
      delay(750);
      //too close to the wall, turn towards center
      
      angleTickCount = abs((64687 * abs( avgWallDistance - wallThreshold )) / (moveDistance * 10000));        //added extra zero because mismatch,changed to abs value because negative values were being returned
      
      turn_R_P(angleTickCount);                      
    }
    else if( rightUS[1] <= wallThreshold && i+1 != num_moves ) ////don't do it in the last iteration
    {
      delay(750);
      //too close to the wall, turn towards the center
       
      angleTickCount = abs((64687 * abs( avgWallDistance - wallThreshold )) / (moveDistance * 10000));       //added extra zero because mismatch, changed to abs value because negative values were being returned
     
      turn_L_P(angleTickCount);                    
    }

		
		totalDistance += approxDistance;
		approxDistance = 0;
		angleTickCount = 0;
		
		delay(500); //delay to lose momentum before next move
	}
	
	return totalDistance;
}

//Test to figure out left motor adj
//void testTravel(int ticks)
//{
//  int speed = 130;
//  int leftMotorAdj = -11;
//
//  Count_Encoder_L = 0;
//  Count_Encoder_R = 0;
//  
//  while (Count_Encoder_L < ticks)
//  {
//    mtr_ctrl.setM1Speed(speed);
//    mtr_ctrl.setM2Speed(speed + leftMotorAdj);
//  }
//  
//  mtr_ctrl.setM1Speed(STOP);
//  mtr_ctrl.setM2Speed(STOP);
//}

void dropVictim()
{
  delay(1000);
  manipulateGripper('D');
  delay(1000);
  manipulateGripper('O');
  delay(1000);
  manipulateGripper('U');
  delay(1000);
  manipulateGripper('O');
  delay(1000);
}

void aboutFace()
{
  int thresh = 55; //original value = 40 4/27/16
  filtered_US();
  if ((Distance_US_L < Distance_US_R))// && (Distance_US_L >= thresh))
  {
      turn_L(TURN_LEFT_TICK);
      delay(1200);
      alignRobot();
      delay(1200);
      turn_L(TURN_LEFT_TICK);
      delay(1200);
  }
  else if ((Distance_US_R < Distance_US_L))// && (Distance_US_R >= thresh))
  {
      turn_R(TURN_RIGHT_TICK);
      delay(1200);
      alignRobot();
      delay(1200);
      turn_R(TURN_RIGHT_TICK);
      delay(1200);
  }
  else
  {
      turn_L(TURN_LEFT_TICK);
      delay(1200);
      alignRobot();
      delay(1200);
      turn_L(TURN_LEFT_TICK);
      delay(1200);
  }
}

