///////////////////////////////////////////////////////////////
// DC motor control test sketch
//
// 2/24/2016
///////////////////////////////////////////////////////////////

const int left_dir = 7;
const int left_speed = 9;

const int right_dir = 8;
const int right_speed = 10;

const int d2 = 4;

//const int pot = A0;

int pulse_width = 0;

int sign = 1;

char pred = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(left_dir,OUTPUT);
  pinMode(left_speed,OUTPUT);
  pinMode(right_dir,OUTPUT);
  pinMode(right_speed,OUTPUT);
  pinMode(d2,OUTPUT);

  digitalWrite(left_dir,HIGH);
  digitalWrite(right_dir,HIGH);
  digitalWrite(d2,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:


  analogWrite(left_speed,pulse_width);
  analogWrite(right_speed,pulse_width);

  pulse_width = pred ? 0 : 64;
  pred = !pred;
  delay(5000);

}


