#include <Servo.h>    //the library which helps us to control the servo motor

int msg[5];
int flex_5_val_avg =0;

Servo myServo1;
Servo myServo2;
Servo myServo3;
Servo myServo4;
Servo myServo5;
//define the flex sensor input pins
int flex_5 = A5;
int flex_4 = A4;
int flex_3 = A3;
int flex_2 = A2;
int flex_1 = A1;

//define variables for flex sensor values
int flex_5_val;
int flex_4_val;
int flex_3_val;
int flex_2_val;
int flex_1_val;


//define variables for servo positions after mapping flex sensor values
int servo_5_pos;
int servo_4_pos;
int servo_3_pos;
int servo_2_pos;
int servo_1_pos;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  myServo1.attach(15); //A1
  myServo2.attach(16); //A2
  myServo3.attach(17); //A3
  myServo4.attach(18); //A4
  myServo5.attach(19); //A5

  
  
}

void loop() {
  // put your main code here, to run repeatedly:
//  flex_5_val = analogRead(flex_5);
//  Serial.println(flex_5_val);
//  servo_5_pos = map(flex_5_val, 40, 140, 0, 100);
  
  for (int i=0;i<15;i++){
  flex_5_val = analogRead(flex_5); 
  //Serial.println(flex_5_val);
  flex_5_val_avg = flex_5_val_avg + flex_5_val;
  }
  flex_5_val_avg = flex_5_val_avg/15;
  servo_5_pos = map(flex_5_val_avg, 50, 220, 0, 120);
  Serial.print("avg: ");
  Serial.println(flex_5_val_avg);
  
  
  //Serial.println(flex_5_val);
  //int servo_5_pos = map(flex_5_val, 35, 60, 0, 10);
  //servo_5_pos = map(flex_5_val, 25, 50, 0, 130);
  delay(100);
  
  flex_4_val  = analogRead(flex_4);
  servo_4_pos = map(flex_4_val, 0, 100, 0, 90);
 
  flex_3_val  = analogRead(flex_3);
  servo_3_pos = map(flex_3_val, 0, 60, 0, 100);
 
  flex_2_val  = analogRead(flex_2);
  servo_2_pos = map(flex_2_val, 10, 90, 0, 100);
  
  flex_1_val  = analogRead(flex_1);
  servo_1_pos = map(flex_1_val, 50, 17, 0, 100);
  
  msg[0] = servo_5_pos;
  msg[1] = servo_4_pos;
  msg[2] = servo_3_pos;
  msg[3] = servo_2_pos;
  msg[4] = servo_1_pos;
    
    myServo1.write(0); //A1
    myServo4.write(msg[0]); //A2
    myServo3.write(0); //A3
    myServo2.write(0); //A4
    myServo5.write(0); //A5
    
}
