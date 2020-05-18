//nRF Libraries
#include <SPI.h>                      //the communication interface with the modem
#include "RF24.h"                     //the library which helps us to control the radio modem

// IMU Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


MPU6050 mpu_1;
MPU6050 mpu_2(0x69);

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN_1 2  // use pin 2 on Arduino Uno
#define INTERRUPT_PIN_2 3  // use pin 3 on Arduino Uno
#define LED_PIN_1 13
#define LED_PIN_2 12 

bool blinkState_1 = false;
bool blinkState_2 = false;

// MPU1 control/status vars
bool dmpReady_1 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_1;   // holds actual interrupt status byte from MPU
uint8_t devStatus_1;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_1;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_1;     // count of all bytes currently in FIFO
uint8_t fifoBuffer_1[64]; // FIFO storage buffer

// MPU2 control/status vars
bool dmpReady_2 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus_2;   // holds actual interrupt status byte from MPU
uint8_t devStatus_2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize_2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount_2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer_2[64]; // FIFO storage buffer

// orientation/motion vars for MPU1
Quaternion q_1;           // [w, x, y, z]         quaternion container
VectorInt16 aa_1;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_1;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_1;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_1;    // [x, y, z]            gravity vector
float euler_1[3];         // [psi, theta, phi]    Euler angle container
float ypr_1[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// orientation/motion vars for MPU2
Quaternion q_2;           // [w, x, y, z]         quaternion container
VectorInt16 aa_2;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal_2;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld_2;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity_2;    // [x, y, z]            gravity vector
float euler_2[3];         // [psi, theta, phi]    Euler angle container
float ypr_2[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt_1 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_1() {
    mpuInterrupt_1 = true;
}

volatile bool mpuInterrupt_2 = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady_2() {
    mpuInterrupt_2 = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Serial.begin(115200);
 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu_1.initialize();                 //Setting Clock , Full Scale of Acc & Gyro ,Sleep Enable
    pinMode(INTERRUPT_PIN_1, INPUT);    

    mpu_2.initialize();                 //Setting Clock , Full Scale of Acc & Gyro ,Sleep Enable
    pinMode(INTERRUPT_PIN_2, INPUT);
        
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu_1.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(mpu_2.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus_1 = mpu_1.dmpInitialize();
    devStatus_2 = mpu_2.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu_1.setXGyroOffset(118);
    mpu_1.setYGyroOffset(3);
    mpu_1.setZGyroOffset(52);
    mpu_1.setZAccelOffset(4764); // 1688 factory default for my test chip

    mpu_2.setXGyroOffset(81);
    mpu_2.setYGyroOffset(102);
    mpu_2.setZGyroOffset(48);
    mpu_2.setZAccelOffset(1612); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus_1 == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu_1.CalibrateAccel(6);
        mpu_1.CalibrateGyro(6);
        mpu_1.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu_1.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_1));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), dmpDataReady_1, RISING);
        mpuIntStatus_1 = mpu_1.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_1 = true;

        // get expected DMP packet size for later comparison
        packetSize_1 = mpu_1.dmpGetFIFOPacketSize();
        } 
        
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus_1);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN_1, OUTPUT);


    // make sure it worked (returns 0 if so)
    if (devStatus_2 == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu_2.CalibrateAccel(6);
        mpu_2.CalibrateGyro(6);
        mpu_2.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu_2.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN_2));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), dmpDataReady_2, RISING);
        mpuIntStatus_2 = mpu_2.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady_2 = true;

        // get expected DMP packet size for later comparison
        packetSize_2 = mpu_2.dmpGetFIFOPacketSize();
        } 
        
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus_2);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN_2, OUTPUT);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady_1) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt_1 && fifoCount_1 < packetSize_1) {
        if (mpuInterrupt_1 && fifoCount_1 < packetSize_1) {
          // try to get out of the infinite loop 
          fifoCount_1 = mpu_1.getFIFOCount();
        }  
    }

    if (!dmpReady_2) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt_2 && fifoCount_2 < packetSize_2) {
        if (mpuInterrupt_2 && fifoCount_2 < packetSize_2) {
          // try to get out of the infinite loop 
          fifoCount_2 = mpu_2.getFIFOCount();
        }  
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt_1 = false;
    mpuIntStatus_1 = mpu_1.getIntStatus();

    mpuInterrupt_2 = false;
    mpuIntStatus_2 = mpu_2.getIntStatus();
    
    // get current FIFO count
    fifoCount_1 = mpu_1.getFIFOCount();
    if(fifoCount_1 < packetSize_1){
    }
    else if ((mpuIntStatus_1 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount_1 >= 1024) {
        // reset so we can continue cleanly
        mpu_1.resetFIFO();
        //  fifoCount_1 = mpu_1.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus_1 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
        while(fifoCount_1 >= packetSize_1){ // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu_1.getFIFOBytes(fifoBuffer_1, packetSize_1);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount_1 -= packetSize_1;
      }
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
           // display Euler angles in degrees
           mpu_1.dmpGetQuaternion(&q_1, fifoBuffer_1);
           mpu_1.dmpGetGravity(&gravity_1, &q_1);
           mpu_1.dmpGetYawPitchRoll(ypr_1, &q_1, &gravity_1);
           Serial.print("ypr_1\t");
           Serial.print(ypr_1[0] * 180/M_PI);
           Serial.print("\t");
           Serial.print(ypr_1[1] * 180/M_PI);
           Serial.print("\t");
           Serial.print(ypr_1[2] * 180/M_PI);
           Serial.print("\t\t");
       #endif

       // blink LED to indicate activity
       blinkState_1 = !blinkState_1;
       digitalWrite(LED_PIN_1, blinkState_1);
      }

     fifoCount_2 = mpu_2.getFIFOCount();
   if(fifoCount_2 < packetSize_2){
    }
    else if ((mpuIntStatus_2 & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount_2 >= 1024) {
        // reset so we can continue cleanly
        mpu_2.resetFIFO();
        //  fifoCount_2 = mpu_2.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus_2 & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
        while(fifoCount_2 >= packetSize_2){ // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu_2.getFIFOBytes(fifoBuffer_2, packetSize_2);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount_2 -= packetSize_2;
      }
       

       #ifdef OUTPUT_READABLE_YAWPITCHROLL
           // display Euler angles in degrees
           mpu_2.dmpGetQuaternion(&q_2, fifoBuffer_2);
           mpu_2.dmpGetGravity(&gravity_2, &q_2);
           mpu_2.dmpGetYawPitchRoll(ypr_2, &q_2, &gravity_2);
           Serial.print("ypr_2\t");
           Serial.print(ypr_2[0] * 180/M_PI);
           Serial.print("\t");
           Serial.print(ypr_2[1] * 180/M_PI);
           Serial.print("\t");
           Serial.print(ypr_2[2] * 180/M_PI);
           Serial.print("\t");
           Serial.println(millis());
       #endif
      
       // blink LED to indicate activity
       blinkState_2 = !blinkState_2;
       digitalWrite(LED_PIN_2, blinkState_2);
    }
  
    
}
