//References: http://learn.parallax.com/KickStart/28015 for Ultrasonic example code
//http://www.instructables.com/id/Get-started-with-distance-sensors-and-Arduino/?ALLSTEPS for example code for US
//https://www.arduino.cc/en/Tutorial/Ping Sample code for US sensor

//declare pins for Ultrasonic sensor
const int digPinUS = 34;    

//Variables for US sensors
long pulseWidth,cm; 
long distanceUS[10];  
int loopControl = 0;

void setup ()
{
  Serial.begin(9600);   //Communication with PC
}

void loop()
{
  while (loopControl < 1)
  {
    //Check distance values for each sensor and
    //send distance information to other parts of program or pi
    int i;
    for (i=0;i<10;i=i+1)
    {
      distanceUS[i] = checkUS(digPinUS);
      delay(100);
    }
    
    int j;
    long sum = 0;
    long average = 0;
    for (j=0;j<10;j=j+1)
    {
      sum = sum + distanceUS[j];
    }
    average = sum/10;
    Serial.print(" Average distance = ");
    Serial.print(average);
    Serial.print("cm");
    Serial.println();
    delay(100);
    loopControl = loopControl + 1;
  }
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
  cm = pulseWidth/53;             // Convert to centimeters, use 58 for Mega, 53 for Due
  Serial.print(cm);
  Serial.print("cm ");
  Serial.print(pulseWidth);
  Serial.print("us ");
  //Serial.println();
  delayMicroseconds(200);           // delay before proceeding. May not be necessary because of IR checks after this
  return cm;
}
