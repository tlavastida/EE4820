
#define STOP 0

#define SPEED 100


const int left_speed_pin = 9;
const int right_speed_pin = 10;

const int left_dir_pin = 7;
const int right_dir_pin = 8;

const int d2_pin = 4;

void setup() {
  // put your setup code here, to run once:

  digitalWrite(d2_pin,HIGH);

}


void loop() {
  // put your main code here, to run repeatedly:

  //go forward
  digitalWrite(left_dir_pin,HIGH);
  digitalWrite(right_dir_pin,HIGH);
  analogWrite(left_speed_pin,SPEED);
  analogWrite(right_speed_pin,SPEED);

  //5 secs
  delay(5000);

  //stop
  analogWrite(left_speed_pin,STOP);
  analogWrite(right_speed_pin,STOP);

  //2 secs
  delay(2000);

  //go backward
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
  analogWrite(left_speed_pin,SPEED);
  analogWrite(right_speed_pin,SPEED);

  //5 secs
  delay(5000);

  //stop
  analogWrite(left_speed_pin,STOP);
  analogWrite(right_speed_pin,STOP);


}
