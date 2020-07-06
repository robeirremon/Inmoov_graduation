
 /*
  Gearmotor Rotary Encoder Test
  motor-encoder-rpm.ino
  Read pulses from motor encoder to calculate speed
  Control speed with potentiometer
  Displays results on Serial Monitor
  Use Cytron MD10C PWM motor controller
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/
 
// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 20
 
// Encoder output to Arduino Interrupt pin
#define ENC_IN_A 3 
#define ENC_IN_B 2

// MD10C PWM connected to pin 10
#define PWM_A 10
#define PWM_B 5
 
// Motor A DIR
#define DIR1_A 12 
#define DIR2_A 11

// Motor B DIR
#define DIR1_B 7  
#define DIR2_B 6 
 
// Analog pin for potentiometer
int speedcontrol = 0;
 
// Pulse count from encoder
volatile long encoderValue_A = 0;
volatile long encoderValue_B = 0;
 
// One-second interval for measurements
int interval = 1000;
 
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
int rpm_A = 0;
int rpm_B = 0; 
// Variable for PWM motor speed output
int motorPwm = 0;
 
void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600); 
  
  // Set encoder as input with internal pullup  
  pinMode(ENC_IN_A, INPUT_PULLUP); 
  pinMode(ENC_IN_B, INPUT_PULLUP);
  
  // Set PWM and DIR connections as outputs
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR1_A, OUTPUT);
  pinMode(DIR2_A, OUTPUT);

  pinMode(PWM_B, OUTPUT);
  pinMode(DIR1_B, OUTPUT);
  pinMode(DIR2_B, OUTPUT);
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(ENC_IN_A), updateEncoder_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_B), updateEncoder_B, RISING);
  // Setup initial values for timer
  previousMillis = millis();
}
 
void loop()
{
  
    // Control motor with potentiometer
    //motorPwm = map(analogRead(speedcontrol), 0, 1023, 0, 255);
    
    // Write PWM to controller
    //Move Forward
    analogWrite(PWM_A, 200);
    digitalWrite(DIR1_A,LOW);
    digitalWrite(DIR2_A,HIGH);

    analogWrite(PWM_B, 212);
    digitalWrite(DIR1_B,LOW);
    digitalWrite(DIR2_B,HIGH);
    delay(2000);
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
    delay(1000);
    
    //Move Backword
    analogWrite(PWM_A, 200);
    digitalWrite(DIR1_A,HIGH);
    digitalWrite(DIR2_A,LOW);

    analogWrite(PWM_B, 212);
    digitalWrite(DIR1_B,HIGH);
    digitalWrite(DIR2_B,LOW);
    delay(2000);
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
    delay(1000);

    //Spin right
    analogWrite(PWM_A, 200);
    digitalWrite(DIR1_A,HIGH);
    digitalWrite(DIR2_A,LOW);

    analogWrite(PWM_B, 212);
    digitalWrite(DIR1_B,LOW);
    digitalWrite(DIR2_B,HIGH);
    delay(2000);
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
    delay(1000);

    //Spin left
    analogWrite(PWM_A, 200);
    digitalWrite(DIR1_A,LOW);
    digitalWrite(DIR2_A,HIGH);

    analogWrite(PWM_B, 212);
    digitalWrite(DIR1_B,HIGH);
    digitalWrite(DIR2_B,LOW);
    delay(2000);
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
    delay(10000);
    
  // Update RPM value every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
 
 
    // Calculate RPM
    rpm_A = (float)(encoderValue_A * 60 / ENC_COUNT_REV);
    rpm_B = (float)(encoderValue_B * 60 / ENC_COUNT_REV);
 
    // Only update display when there is a reading
    if (motorPwm > 0 || rpm_A  > 0|| rpm_B > 0) {
      Serial.print("PWM VALUE: ");
      Serial.print(motorPwm);
      Serial.print('\t');
      Serial.print(" PULSES_A: ");
      Serial.print(encoderValue_A);
      Serial.print('\t');
      Serial.print(" PULSES_B: ");
      Serial.print(encoderValue_B);
      Serial.print('\t');
      Serial.print(" SPEED_A: ");
      Serial.print(rpm_A);
      Serial.println(" RPM");
      Serial.print('\t');
      Serial.print(" SPEED_B: ");
      Serial.print(rpm_B);
      Serial.println(" RPM");
    }
    
    encoderValue_A = 0;
    encoderValue_B = 0;
  }

  delay(10000);
}

 
void updateEncoder_A()
{
  // Increment value for each pulse from encoder
  encoderValue_A++;
}

void updateEncoder_B()
{
  // Increment value for each pulse from encoder
  encoderValue_B++;
}
