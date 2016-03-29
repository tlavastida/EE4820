
char str[128];
int n;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(2000000);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available() > 0)
  {
    n = Serial.readBytesUntil('\n',str,120);
    //str[(n>0 ? n : 0)] = '\0';
    Serial.print("You said: ");
    Serial.write(str,n);
    if( n > 0 && str[n-1] != '\n' )
    {
      Serial.print(" ... length = ");
      Serial.print(n);
      Serial.print('\n');
      
    }

    
  }
  

}


