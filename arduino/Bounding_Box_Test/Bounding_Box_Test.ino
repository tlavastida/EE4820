/*****************************************************************************************
*  Notes:
* Constants are all capital
* Global volatile variables start with a capital letter and camelcase for the remaining
* Local variables are camelcase
*****************************************************************************************/

//*********************************************************************************
//                PIN ASSIGNMENTS
//*********************************************************************************
//declare pins for Infrared sensors
const int ANALOG_PIN_IR_L = A8;     
const int ANALOG_PIN_IR_R = A9;    

//declare pins for Ultrasonic sensors
const int DIG_PIN_US_L = 31;
const int DIG_PIN_US_R = 33;
const int DIG_PIN_US_F = 35;        

//*********************************************************************************

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

volatile long Distance;
volatile bool AtStart = true;

//Variables for serial
char Buffer[128]; 

//Threshold variables
volatile long US_HALL_THRESHOLD;          //Volatile but treat as a constant
volatile long US_DANGER_THRESHOLD;          //Volatile but treat as a constant

//setup servo objects and set initial position
void setup()
{ 
  
  Serial.begin(250000);

}
 
//repeat test process 
void loop()
{
  if (AtStart)
  {
    //Set Threshold to maintain distance from objects
    //Take US L/R measurements at start
      
      Distance_US_L = checkUS(DIG_PIN_US_L);
      Distance_US_R = checkUS(DIG_PIN_US_R);
      
      Distance = (Distance_US_L + Distance_US_R + 153);
      US_HALL_THRESHOLD = Distance/2 - 76;
      US_DANGER_THRESHOLD = Distance/4 - 76; 
      
    sprintf(Buffer, "US_HALL_THRESHOLD:%ld, US_DANGER_THRESHOLD:%ld\n",US_HALL_THRESHOLD,US_DANGER_THRESHOLD);
    Serial.print(Buffer);
    
      AtStart = false;

  }
  checkSensors();
  delay(1);
  printSensorValues();
  delay(2000);
}

long checkIR (int pinNumIR)
{
  analogValue = analogRead(pinNumIR);                       //Read current analog value from pin
  //distanceValue = (400000-(42*analogValue))/(10*analogValue);   //conversion based on one from ref //Due conversion
  distanceValue = (264000-(42*analogValue))/(10*analogValue);   //conversion based on one from ref //Mega conversion
  Serial.print(pinNumIR);
  Serial.print(" ");
  Serial.println(distanceValue);
  return distanceValue;                                 //mm
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
  Serial.print(pinNumUS);
  Serial.print(" ");
  Serial.println(mm);
 //Serial.print(" ");
  return mm;
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
  snprintf(Buffer, 128, "IR_L:%ld, IR_R:%ld, US_L:%ld, US_R:%ld, US_F:%ld\n",Distance_IR_L, Distance_IR_R, Distance_US_L, Distance_US_R, Distance_US_F);
  //sprintf(Buffer, "Encoder1:%d,Encoder2:%d\n",Count_Encoder_Left,Count_Encoder_Right); //We can use this if we only want to send encoder counts
  Serial.print(Buffer);
}





