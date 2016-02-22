
char buf[128];
long x,y;
long z;
char val;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(2000000);
  //Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0)
  {
    val = Serial.read();
    x = Serial.parseInt();
    y = Serial.parseInt();

    //printInt(x);
    //printInt(y);
 
    if( val == '+' )
    {
      z = x + y;
      //Serial.println("plus");
    }
    else
    {
      z = x * y;
      //Serial.println("times");
    }
    sprintf(buf,"%d",z);
    Serial.println(buf);
    clear_serial();
    
  }
  

}

void clear_serial()
{
  while( Serial.available() > 0 )
    Serial.read();
}

void printInt(long x)
{
  char str[64];
  sprintf(str,"%d\n",x);
  Serial.print(str);
}

