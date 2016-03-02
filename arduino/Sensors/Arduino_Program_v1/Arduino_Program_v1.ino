//References: 
//1: http://learn.parallax.com/KickStart/28015 for Ultrasonic example code
//2: http://www.instructables.com/id/Get-started-with-distance-sensors-and-Arduino/?ALLSTEPS for example code for IR/US
//3: http://www.robotshop.com/blog/en/arduino-5-minute-tutorials-lesson-4-ir-distance-sensor-push-button-2-3637 for IR example code
//4: http://www.robotoid.com/appnotes/circuits-quad-encoding.html
//5: http://learn.robotgeek.com/getting-started/30-dev-kits/55-robotgeek-gripper-getting-started-guide.html

#include <Servo.h>   //include the servo library to control the RobotGeek Servos

#define MICRO_SERVOPIN 9  //pin that the micro servo will be attached to
#define LARGE_SERVOPIN 11  //pin that the large servo will be attached to

Servo microServo;   //create an servo object for the 9g FT-FS90MG micro servo
Servo largeServo;   //create an servo object for the RobotGeek 180 degree serco

const int timeDelay = 500;        //Delay between gripper movements
const int microDelay = 5;       //Micro-delay for for loop for wrist movement
const int gripOpen = 0;         //Gripper all the way open
const int gripClosed = 130;       //Gripper all the way closed
//const int wristDown = 150;
//const int wristUp = 0;
const int wristLevel = 90;        //Gripper straight out
const int gripGrab = 84;        //Angle needed for gripper to close on target
const int wristPickup = 10;       //Angle to pick target up to
const int wristLower = 110;       //Angle to lower wrist to be able to pick up target

//declare pins for Infrared sensors
const int analogPinIR1 = A0;
const int analogPinIR2 = A1;

//declare pins for Ultrasonic sensor
const int digPinUS1 = 24;
const int digPinUS2 = 26;
const int digPinUS3 = 28;

//Variables for IR sensors
long analogValue,distanceValue;
long distanceIR1,distanceIR2;  

//Variables for US sensors
long pulseWidth,cm; 
long distanceUS1,distanceUS2,distanceUS3;  

//Variables for LS7184 chips
const int clk1 = 25;
const int direction1 = 27;
volatile int count1 = 0;
volatile boolean flag1 = false;

const int clk2 = 29;
const int direction2 = 31;
volatile int count2 = 0;
volatile boolean flag2 = false;


void setup ()
{
  Serial.begin(9600);
  pinMode(clk1,INPUT);
  pinMode(direction1,INPUT);
  attachInterrupt(clk1,encoderInterrupt1,RISING);
  pinMode(clk2,INPUT);
  pinMode(direction2,INPUT);
  attachInterrupt(clk2,encoderInterrupt2,RISING);
  microServo.attach(MICRO_SERVOPIN);
  microServo.write(130);                    // sets the servo position to 150 degress, positioning the servo for the gripper closed
  largeServo.attach(LARGE_SERVOPIN);
  largeServo.write(90);                     // sets the servo position to 90 degress, centered
}

//Main Program
void loop()
{
  //Check IR sensors first
  distanceIR1 = checkIR(analogPinIR1,1);
  distanceIR2 = checkIR(analogPinIR2,2);
  //Check if both are about equal
  if ((distanceIR1 == 4)&&(distanceIR2 == 4))       // will need to test actual distance
    {
      grabVictim();
    }
  //Check US sensors
  distanceUS1 = checkUS(digPinUS1,1);           // This will be left or right sensor
  //logic check for threshold here //if (distanceUS1 <= 6) -> control motor away from object
  distanceUS2 = checkUS(digPinUS2,2);           // This will be left or right sensor  
  //logic check for threshold here //if (distanceUS2 <= 6) -> control motor away from object
  distanceUS3 = checkUS(digPinUS3,3);           // This will be front sensor
  
}

//Checks IR sensors - Code based on ref 2
int checkIR (int pinNumIR,int numIR)
{
  analogValue = analogRead(pinNumIR);                   // Read current analog value from pin
  distanceValue = (4000/analogValue)-(42/100);            //Conversion factor ref 2 and modified using datasheet information
  Serial.print("IR ");
  Serial.print(numIR);
  Serial.print(" ");
  Serial.print("Analog ");
  Serial.print(analogValue);
  Serial.print(" ");
  Serial.print(distanceValue);
  Serial.print(" cm ");
  Serial.println();
  return distanceValue;                           //cm distance
}

//Checks US sensors - Code based on ref 1,2
long checkUS (int pinNumUS,int numUS)
{
  pinMode(pinNumUS,OUTPUT);                         // set up pin to initiate pulse
  digitalWrite(pinNumUS,LOW);                         // make sure pin is low before pulsing
  delayMicroseconds(2);                             // for 2 microseconds 
  digitalWrite(pinNumUS,HIGH);                        // Start pulse
  delayMicroseconds(5);                             // for 5 microseconds
  digitalWrite(pinNumUS,LOW);                         // set pin back to low to ready for return pulse
  pinMode(pinNumUS,INPUT);                          // change pin to Input mode for return pulse
  pulseWidth = pulseIn(pinNumUS,HIGH,18500);              // wait for return pulse. Timeout after 18.5 milliseconds
  cm = pulseWidth/53;                               // Convert to centimeters, use 58 for Mega, 53 for Due
  Serial.print("US ");
  Serial.print(numUS);
  Serial.print(" pulse width = ");
  Serial.print(pulseWidth);
  Serial.print(" us ");
  Serial.print("distance = ");
  Serial.print(cm);
  Serial.print(" cm");
  Serial.println();
  //delayMicroseconds(200);                           // delay before proceeding. May not be necessary because of other sensor checks
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
 
//function to grab victim - Code based on ref 5
//This will have to be modified later
//Only picks up, holds, then drops victim
void grabVictim()
{
  //Pickup victim
  microServo.write(gripOpen);               //set gripper to fully open
  Serial.println("Gripper Open");
  delay(timeDelay);                         //wait
  largeServo.write(wristLower);               //Lower wrist to grab victim
  Serial.println("Wrist Level");
  delay(timeDelay);                         //wait
  microServo.write(gripGrab);               //set gripper to grab victim
  Serial.println("Grab Victim");
  delay(timeDelay);                         //wait
  for (int i=wristLower;i>wristPickup;i=i-1)
  {
    largeServo.write(i);                  //Pickup victim
    delay(microDelay);                  //Using small delay in loop to minimize momentum of gripper
  }
  Serial.println("Wrist Up");
  delay(timeDelay);                         //wait
  for (int i=wristPickup;i<wristLower;i=i+1)
  {
    largeServo.write(i);                  //Lower wrist to drop victim
    delay(microDelay);                  //Using small delay in loop to minimize momentum of gripper
  }
  Serial.println("Wrist lowered");
  delay(timeDelay);                         //wait
  microServo.write(gripOpen);               //set gripper to fully open
  Serial.println("Gripper Open");
  delay(timeDelay);                         //wait
} 
