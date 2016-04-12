void setup() {
  // put your setup code here, to run once:

}

void alignmentUS() {
  // put your main code here, to run repeatedly:

  alignThreshold = 4;
  while (US_R1_D < US_R2_D || US_L1_D < US_L2_D)
  {
    if (US_R1_D <= alignThreshold) // veering right, go counter-clockwise
    {
      m2const = -1;
      m1const = 1;
    }
    else if (US_L1_D <= alignThreshold) // veering left, go clockwise
    {
      m2const = 1;
      m1const = -1;
    }
    mtr_ctrl.setM2Speed(m2const*(leftSpeed+leftAdjust)); // Left Motor
    mtr_ctrl.setM1Speed(m1const*(followerSpeed));//+8); // Right Motor
  }

}

void travelDistance_Enc_Steve(int numTicks)
{
  int speed = 100;
  int followerSpeed = speed;
  int leftAdjust = -6;
  int minSpeed = speed - 10;
  int maxSpeed = speed + 10 - leftAdjust;
  
  int error = 0;
  int Kp = 5;
  int dangerCounter = 0;
  int dangerCountThresh = 2;
  Count_Encoder_Left = 0;
  Count_Encoder_Right = 0;
  int leftSpeed = speed;
  checkSensors();
  accelFromStop(speed, 0);

  checkSensors_US();

  if (numTicks > 1820)
  {
    while (abs(Count_Encoder_Left) < numTicks)
    {
      checkSensors_US();
      if (Distance_US_F > US_DANGER_THRESHOLD)
      {
        dangerCounter = 0;
        alignmentUS(); // call alignment function
      }
      else
      {
        dangerCounter += 1;
        if (dangerCounter >= dangerCountThresh)
        {
          break;
        }
      }
    }
    mtr_ctrl.setM2Speed(STOP);
    mtr_ctrl.setM1Speed(STOP);
  }
  else
  {
    //do encoder stuff if US checks out
    checkSensors_US();

    if (Distance_US_F > US_DANGER_THRESHOLD)
    {
      dangerCounter = 0;
      error = Count_Encoder_Left - Count_Encoder_Right;
      followerSpeed += error/Kp;
      if (followerSpeed >= maxSpeed)
        followerSpeed = maxSpeed;
      else if (followerSpeed <= minSpeed)
        followerSpeed = minSpeed;
    }
    else
    {
      dangerCounter += 1;
      if (dangerCounter >= dangerCountThresh)
        break;
    }

    mtr_ctrl.setM2Speed(leftSpeed+leftAdjust);
    mtr_ctrl.setM1Speed(followerSpeed);
    delay(100);
  }
  mtr_ctrl.setM2Speed(STOP);
  mtr_ctrl.setM1Speed(STOP);
}

