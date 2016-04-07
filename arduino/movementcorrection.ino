void setup() {
  // put your setup code here, to run once:

}

void travelDistance_Enc(int numTicks)
{
   int totalTicks = 0;
   
   int followerSpeed = 85;//+5;   //right motor is the follower

   int error_US = 0;
   int error = 0;
   int Kp = 30;             //10 
      
   int dangerCounter = 0;
   int dangerCountThresh = 2;
   
   Count_Encoder_Left = 0;
   Count_Encoder_Right = 0;
   
   int leftSpeed = 85;
   
   int danger_US_lower = 3;     //US danger zone (cm);
   int danger_US_upper = 3;
   
   int Kp_US = 2;     //10

   accelFromStop(85,0);
   //checkSensors();
   while (abs(totalTicks) < numTicks)
   {
      checkSensors();
    
      if(Distance_US_F > US_DANGER_THRESHOLD)
      {
        //check if veering to left and lower right motor speed
        if ((Distance_US_L < danger_US_lower) && (Distance_US_L > 0))
        {
          error_US = Distance_US_L - danger_US_lower;
        }
        else if ((Distance_US_R > danger_US_upper) && (Distance_US_R > 0))
        {
          error_US = danger_US_upper - Distance_US_R;
        }
        //check if veering to right and lower left motor speed
        else if ((Distance_US_R < danger_US_lower) && (Distance_US_R > 0))
        {
          error_US = danger_US_lower - Distance_US_R;
        }
        else if ((Distance_US_L > danger_US_upper) && (Distance_US_L > 0))
        {
          error_US = Distance_US_L - danger_US_upper;
        }
        //else  
        //{
          dangerCounter = 0;
          error = Count_Encoder_Left - Count_Encoder_Right;
          followerSpeed += error/Kp + error_US*Kp_US; 
        //}

        mtr_ctrl.setM2Speed(leftSpeed+LEFT_MOTOR_ADJ);  //was 2
        mtr_ctrl.setM1Speed(followerSpeed);//+8); //was 1

        Count_Encoder_Left = 0;
        Count_Encoder_Right = 0;

        error_US = 0;
    
        //delay(100);
        delay(20);
      }
      //check if robot is tilted to the side by comparing forward US and side US
      else if ((Distance_US_F < US_DANGER_THRESHOLD) && (Distance_US_R > danger_US_upper))
      {
        height = (Distance_US_F * Distance_US_R) / (pow(Distance_US_F, 2) + pow(Distance_US_R, 2))
        diff_h_f = Distance = 
        
      }
      else
      {
        dangerCounter += 1;
        if(dangerCounter >= dangerCountThresh)
        {
//          leftSpeed = STOP;
//          followerSpeed = STOP;
          //maybe break?
          break;
        } 
      }

    //100-60 
    totalTicks += Count_Encoder_Left;
   } //end while loop
   
   mtr_ctrl.setM2Speed(STOP);
   mtr_ctrl.setM1Speed(STOP); 
}
