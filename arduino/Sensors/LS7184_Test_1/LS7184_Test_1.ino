//Code base from:
//http://www.robotoid.com/appnotes/circuits-quad-encoding.html

const int clk = 22;
const int direction = 38;
const int inputA = 34;
const int inputB = 36; 
volatile int count = 0;
const int pulseWidth = 10;
const int distance = 50;
volatile int loopControl = 0;
volatile boolean flag = false;

void setup()
{
  Serial.begin(9600);
  pinMode(clk,INPUT);
  pinMode(direction,INPUT);
  pinMode(inputA,OUTPUT);
  pinMode(inputB,OUTPUT);
  attachInterrupt(clk,encoderInterrupt,RISING);
}

void loop()
{ 
  //move forward
  int i;
  int j;
  digitalWrite(inputA,LOW);
  digitalWrite(inputB,LOW);
  while (loopControl<1){
  for (i=0;i<distance;i=i+1)
  {
    digitalWrite(inputA,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(inputB,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(inputA,LOW);
    delayMicroseconds(pulseWidth);
    digitalWrite(inputB,LOW);
    delayMicroseconds(pulseWidth);
    if (flag)
    {
      flag = false;
      Serial.print(count);
      Serial.println();
    }
  }
  //go in reverse
  for (j=0;j<distance;j=j+1)
  {
    digitalWrite(inputB,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(inputA,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(inputB,LOW);
    delayMicroseconds(pulseWidth);
    digitalWrite(inputA,LOW);
    delayMicroseconds(pulseWidth);
    if (flag)
    {
      flag = false;
      Serial.println(count);
    }
  }
  loopControl = loopControl+1;
  }
}

void encoderInterrupt()
{
  /*
  if (digitalRead(direction))// == HIGH)
  {
    count = count + 1;
  } 
  else
  {
    count = count - 1;
  }
  */
  count = digitalRead(direction) ? count + 1: count - 1;
  
  flag = true;
}
