

char byteIn = 0;
int ledPin = 3;
int bright = 255;

void setup() 
{
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
}

void loop() 
{

  while(Serial.available() > 0)
  {
    byteIn = Serial.read();

    switch(byteIn)
    {
      case '1':
      bright = 10;
      break;
      case '2':
      bright = 100;
      break;
      case '3':
      bright = 255;
      break;
      case '\n':
      break;
      default:
      bright = random(255);
      break;
    }
    
  }
  analogWrite(ledPin,bright);

}
