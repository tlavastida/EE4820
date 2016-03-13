
//declare pins for Ultrasonic sensor
const int digPinUS1 = 34;
const int digPinUS2 = 36;
const int digPinUS3 = 38;

//Variables for US sensors
long pulseWidth,cm; 
long distanceUS1,distanceUS2,distanceUS3;  

//Variables for LS7184 chips
const int clk1 = 21;
const int direction1 = 22;
volatile int count1 = 0;
volatile boolean flag1 = false;

const int clk2 = 20;
const int direction2 = 24;
volatile int count2 = 0;
volatile boolean flag2 = false;


void setup ()
{
  Serial.begin(9600);
  pinMode(clk1,INPUT);
  pinMode(direction1,INPUT);
  attachInterrupt(2,encoderInterrupt1,RISING);
  pinMode(clk2,INPUT);
  pinMode(direction2,INPUT);
  attachInterrupt(3,encoderInterrupt2,RISING);                  
}

//Main Program
void loop()
{
 
  distanceUS1 = checkUS(digPinUS1,1);           // This will be left or right sensor
  //logic check for threshold here //if (distanceUS1 <= 6) -> control motor away from object
  distanceUS2 = checkUS(digPinUS2,2);           // This will be left or right sensor  
  //logic check for threshold here //if (distanceUS2 <= 6) -> control motor away from object
  distanceUS3 = checkUS(digPinUS3,3);           // This will be front sensor
  
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
 
