#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

#define STOP 0

#define SPEED 80

//declare pins for Ultrasonic sensor
const int leftUS = 34;   
const int rightUS = 36;   
const int forwardUS = 38;   

//Variables for US sensors
long pulseWidth,cm; 
long distanceLeftUS,distanceRightUS,distanceForwardUS; 

//Variables for LS7184 chips
const int clk1 = 21;          //2,3,18,19,20,21 for mega interrupts
const int direction1 = 22;
//volatile int count1 = 0;
//volatile boolean flag1 = false;

const int clk2 = 20;          //2,3,18,19,20,21 for mega interrupts
const int direction2 = 24;
//volatile int count2 = 0;
//volatile boolean flag2 = false;

//volatile int loopControl = 0;
//const int time = 3000;      //time to run motor;
const int distance = 100;      //distance to travel in cm
const int convFactor = 13;    // counts per cm 13 for cm 33 for in at 1x resolution
volatile int count1 = 0;
volatile int count2 = 0;

volatile int totalTicks = distance*convFactor;

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}


void setup() {
  // put your setup code here, to run once:

  pinMode(clk1,INPUT);
  pinMode(direction1,INPUT);
  attachInterrupt(2,encoderInterrupt1,RISING);
  pinMode(clk2,INPUT);
  pinMode(direction2,INPUT);
  attachInterrupt(3,encoderInterrupt2,RISING);
  Serial.begin(9600);
  
  //Run once
  md.init(); 

  //get setpoint
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
    //Serial.println("I am running now");
    //delay(10);
    //Serial.println("Go Forward");
    //Go

  
    //delay(time);
    //Serial.println("Stop");
    //Stop
 
//    Serial.print("Encoder 1 Value = ");
//    Serial.print(count1);
//    Serial.print(" after ");
//    //Serial.print(time);
//    Serial.print("microseconds ");
//    Serial.println();
//    Serial.print("Encoder 2 Value = ");
//    Serial.print(count2);
//    Serial.print(" after ");
//    //Serial.print(time);
//    Serial.print("microseconds ");
//    Serial.println();
  }

  md.setM1Speed(STOP);
  md.setM2Speed(STOP);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  




//  Serial.println();
//  Serial.print
}

//Interrupt routine for LS7184 chip 1 - Code based on ref 4
void encoderInterrupt1()
{
  count1 = digitalRead(direction1) ? count1 + 1: count1 - 1;
}

//Interrupt routine for LS7184 chip 2 - Code based on ref 4
void encoderInterrupt2()
{
  count2 = digitalRead(direction2) ? count2 + 1: count2 - 1;
}

void checkSensors()
{
  distanceLeftUS = checkUS(leftUS);
  distanceRightUS = checkUS(rightUS);
  distanceForwardUS = checkUS(forwardUS);
}

long checkUS (int pinNumUS)
{
  pinMode(pinNumUS,OUTPUT);         // set up pin to initiate pulse
  digitalWrite(pinNumUS,LOW);         // make sure pin is low before pulsing
  delayMicroseconds(2);           // for 2 microseconds 
  digitalWrite(pinNumUS,HIGH);        // Start pulse
  delayMicroseconds(5);           // for 5 microseconds
  digitalWrite(pinNumUS,LOW);         // set pin back to low to ready for return pulse
  pinMode(pinNumUS,INPUT);          // change pin to Input mode for return pulse
  pulseWidth = pulseIn(pinNumUS,HIGH,18500);  // wait for return pulse. Timeout after 18.5 milliseconds
  cm = pulseWidth/58;             // Convert to centimeters, use 58 for Mega, 53 for Due
//  Serial.print(cm);
//  Serial.print("cm ");
//  Serial.print(pulseWidth);
//  Serial.print("us ");
  //Serial.println();
  delayMicroseconds(200);           // delay before proceeding. May not be necessary because of IR checks after this
  return cm;
}

