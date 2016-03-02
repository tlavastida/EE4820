#include "DualMC33926MotorShield.h"

DualMC33926MotorShield md;

#define STOP 0

#define SPEED 125


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

  md.init();

  delay(2000);
  md.setM1Speed(SPEED);
  delay(10);
  md.setM2Speed(SPEED);

  delay(7000);

  md.setM1Speed(STOP);
  delay(10);
  md.setM2Speed(STOP);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
