//http://www.instructables.com/id/Get-started-with-distance-sensors-and-Arduino/?ALLSTEPS for example code for IR
//http://www.robotshop.com/blog/en/arduino-5-minute-tutorials-lesson-4-ir-distance-sensor-push-button-2-3637 for IR example code


//declare pins for Infrared sensors
const int analogPinIR1 = A0;
const int analogPinIR2 = A8;

//Variables for IR sensors
long analogValue,distanceValue;
long distanceIR1,distanceIR2;  
//int loopControl = 0;

void setup ()
{
  //Setup IR pins to output mode
  //pinMode(analogPinIR1,OUTPUT); //Commented out because readings were interfering with one another
  //pinMode(analogPinIR2,OUTPUT);
  //whatever communication protocol to go here
  Serial.begin(9600);
}

void loop()
{
  //Check distance values for each sensor and
  //send distance information to other parts of program or pi
 // while (loopControl < 100)
  //{

    distanceIR1 = checkIR(analogPinIR1); 
    delay(1000);
    distanceIR2 = checkIR(analogPinIR2); 
    delay(1000);
    //loopControl = loopControl +1;
  //}
}

int checkIR (int pinNumIR)
{
  analogValue = analogRead(pinNumIR);     // Read current analog value from pin
  distanceValue = (4000/analogValue)-(42/100);
  Serial.print(analogValue);
  Serial.print(" ");
  Serial.print(distanceValue);
  Serial.println();
  return analogValue;         //Should be cm I believe
}


