#include <Wire.h>
#include <Kalman.h> 

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

//First IMU
Kalman kalmanX_1; // Create the Kalman instances
Kalman kalmanY_1;
//Second IMU
Kalman kalmanX_2; // Create the Kalman instances
Kalman kalmanY_2;

/* IMU Data for 1st IMU */
double accX_1, accY_1, accZ_1;
double gyroX_1, gyroY_1, gyroZ_1;
int16_t tempRaw_1;

double gyroXangle_1, gyroYangle_1; // Angle calculate using the gyro only
double compAngleX_1, compAngleY_1; // Calculated angle using a complementary filter
double kalAngleX_1, kalAngleY_1; // Calculated angle using a Kalman filter

uint32_t timer_1;
uint8_t i2cData_1[14]; // Buffer for I2C data

/* IMU Data for 2nd IMU */
double accX_2, accY_2, accZ_2;
double gyroX_2, gyroY_2, gyroZ_2;
int16_t tempRaw_2;

double gyroXangle_2, gyroYangle_2; // Angle calculate using the gyro only
double compAngleX_2, compAngleY_2; // Calculated angle using a complementary filter
double kalAngleX_2, kalAngleY_2; // Calculated angle using a Kalman filter

uint32_t timer_2;
uint8_t i2cData_2[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

//First IMU DATA
  i2cData_1[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData_1[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData_1[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData_1[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite_1(0x19, i2cData_1, 4, false)); // Write to all four registers at once
  while (i2cWrite_1(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead_1(0x75, i2cData_1, 1));
//  if (i2cData[0] != 0x69) { // Read "WHO_AM_I" register
//    Serial.print(F("Error reading sensor"));
//    while (1);
//  }

//SECOND IMU DATA
  i2cData_2[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData_2[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData_2[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData_2[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite_2(0x19, i2cData_2, 4, false)); // Write to all four registers at once
  while (i2cWrite_2(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead_2(0x75, i2cData_1, 1));
  
  delay(100); // Wait for sensor to stabilize



  /* Set kalman and gyro starting angle for 1st IMU */
  while (i2cRead_1(0x3B, i2cData_1, 6));
  accX_1 = (int16_t)((i2cData_1[0] << 8) | i2cData_1[1]);
  accY_1 = (int16_t)((i2cData_1[2] << 8) | i2cData_1[3]);
  accZ_1 = (int16_t)((i2cData_1[4] << 8) | i2cData_1[5]);

 /* Set kalman and gyro starting angle for 1st IMU */
  while (i2cRead_2(0x3B, i2cData_2, 6));
  accX_2 = (int16_t)((i2cData_2[0] << 8) | i2cData_2[1]);
  accY_2 = (int16_t)((i2cData_2[2] << 8) | i2cData_2[3]);
  accZ_2 = (int16_t)((i2cData_2[4] << 8) | i2cData_2[5]);
  
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  
#ifdef RESTRICT_PITCH // Eq. 25 and 26  For 1st IMU
  double roll_1  = atan2(accY_1, accZ_1) * RAD_TO_DEG;
  double pitch_1 = atan(-accX_1 / sqrt(accY_1 * accY_1 + accZ_1 * accZ_1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll_1  = atan(accY_1 / sqrt(accX_1 * accX_1 + accZ_1 * accZ_1)) * RAD_TO_DEG;
  double pitch_1 = atan2(-accX_1, accZ_1) * RAD_TO_DEG;
#endif


#ifdef RESTRICT_PITCH // Eq. 25 and 26 For 2nd IMU
  double roll_2  = atan2(accY_2, accZ_2) * RAD_TO_DEG;
  double pitch_2 = atan(-accX_2 / sqrt(accY_2 * accY_2 + accZ_2 * accZ_2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll_2  = atan(accY_2 / sqrt(accX_2 * accX_2 + accZ_2 * accZ_2)) * RAD_TO_DEG;
  double pitch_2 = atan2(-accX_2, accZ_2) * RAD_TO_DEG;
#endif

  kalmanX_1.setAngle(roll_1); // Set starting angle
  kalmanY_1.setAngle(pitch_1);
  gyroXangle_1 = roll_1;
  gyroYangle_1 = pitch_1;
  compAngleX_1 = roll_1;
  compAngleY_1 = pitch_1;

   timer_1 = micros();

  kalmanX_2.setAngle(roll_2); // Set starting angle
  kalmanY_2.setAngle(pitch_2);
  gyroXangle_2 = roll_2;
  gyroYangle_2 = pitch_2;
  compAngleX_2 = roll_2;
  compAngleY_2 = pitch_2;
  
  timer_2 = micros();
}

void loop() {
  /* Update all the values for 1st IMU*/
  while (i2cRead_1(0x3B, i2cData_1, 14));
  accX_1 = (int16_t)((i2cData_1[0] << 8) | i2cData_1[1]);
  accY_1 = (int16_t)((i2cData_1[2] << 8) | i2cData_1[3]);
  accZ_1 = (int16_t)((i2cData_1[4] << 8) | i2cData_1[5]);
  tempRaw_1 = (int16_t)((i2cData_1[6] << 8) | i2cData_1[7]);
  gyroX_1 = (int16_t)((i2cData_1[8] << 8) | i2cData_1[9]);
  gyroY_1 = (int16_t)((i2cData_1[10] << 8) | i2cData_1[11]);
  gyroZ_1 = (int16_t)((i2cData_1[12] << 8) | i2cData_1[13]);;

  double dt_1 = (double)(micros() - timer_1) / 1000000; // Calculate delta time
  timer_1 = micros();

  /* Update all the values for 2nd IMU*/
  while (i2cRead_2(0x3B, i2cData_2, 14));
  accX_2 = (int16_t)((i2cData_2[0] << 8) | i2cData_2[1]);
  accY_2 = (int16_t)((i2cData_2[2] << 8) | i2cData_2[3]);
  accZ_2 = (int16_t)((i2cData_2[4] << 8) | i2cData_2[5]);
  tempRaw_2 = (int16_t)((i2cData_2[6] << 8) | i2cData_2[7]);
  gyroX_2 = (int16_t)((i2cData_2[8] << 8) | i2cData_2[9]);
  gyroY_2 = (int16_t)((i2cData_2[10] << 8) | i2cData_2[11]);
  gyroZ_2 = (int16_t)((i2cData_2[12] << 8) | i2cData_2[13]);;

  double dt_2 = (double)(micros() - timer_2) / 1000000; // Calculate delta time
  timer_2 = micros();


  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  
#ifdef RESTRICT_PITCH // Eq. 25 and 26  For 1st IMU
  double roll_1  = atan2(accY_1, accZ_1) * RAD_TO_DEG;
  double pitch_1 = atan(-accX_1 / sqrt(accY_1 * accY_1 + accZ_1 * accZ_1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll_1  = atan(accY_1 / sqrt(accX_1 * accX_1 + accZ_1 * accZ_1)) * RAD_TO_DEG;
  double pitch_1 = atan2(-accX_1, accZ_1) * RAD_TO_DEG;
#endif

  double gyroXrate_1 = gyroX_1 / 131.0; // Convert to deg/s
  double gyroYrate_1 = gyroY_1 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH // Eq. 25 and 26 For 2nd IMU
  double roll_2  = atan2(accY_2, accZ_2) * RAD_TO_DEG;
  double pitch_2 = atan(-accX_2 / sqrt(accY_2 * accY_2 + accZ_2 * accZ_2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll_2  = atan(accY_2 / sqrt(accX_2 * accX_2 + accZ_2 * accZ_2)) * RAD_TO_DEG;
  double pitch_2 = atan2(-accX_2, accZ_2) * RAD_TO_DEG;
#endif

  double gyroXrate_2 = gyroX_2 / 131.0; // Convert to deg/s
  double gyroYrate_2 = gyroY_2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll_1 < -90 && kalAngleX_1 > 90) || (roll_1 > 90 && kalAngleX_1 < -90)) {
    kalmanX_1.setAngle(roll_1);
    compAngleX_1 = roll_1;
    kalAngleX_1 = roll_1;
    gyroXangle_1 = roll_1;
  } else
    kalAngleX_1 = kalmanX_1.getAngle(roll_1, gyroXrate_1, dt_1); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX_1) > 90)
    gyroYrate_1 = -gyroYrate_1; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY_1 = kalmanY_1.getAngle(pitch_1, gyroYrate_1, dt_1);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch_1 < -90 && kalAngleY_1 > 90) || (pitch_1 > 90 && kalAngleY_1 < -90)) {
    kalmanY_1.setAngle(pitch_1);
    compAngleY_1 = pitch_1;
    kalAngleY_1 = pitch_1;
    gyroYangle_1 = pitch_1;
  } else
    kalAngleY_1 = kalmanY_1.getAngle(pitch_1, gyroYrate_1, dt_1); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY_1) > 90)
    gyroXrate_1 = -gyroXrate_1; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX_1 = kalmanX_1.getAngle(roll_1, gyroXrate_1, dt_1); // Calculate the angle using a Kalman filter
#endif

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll_2 < -90 && kalAngleX_2 > 90) || (roll_2 > 90 && kalAngleX_2 < -90)) {
    kalmanX_2.setAngle(roll_2);
    compAngleX_2 = roll_2;
    kalAngleX_2 = roll_2;
    gyroXangle_2 = roll_2;
  } else
    kalAngleX_2 = kalmanX_2.getAngle(roll_2, gyroXrate_2, dt_2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX_2) > 90)
    gyroYrate_2 = -gyroYrate_2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY_2 = kalmanY_2.getAngle(pitch_2, gyroYrate_2, dt_2);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch_2 < -90 && kalAngleY_2 > 90) || (pitch_2 > 90 && kalAngleY_2 < -90)) {
    kalmanY_2.setAngle(pitch_2);
    compAngleY_2 = pitch_2;
    kalAngleY_2 = pitch_2;
    gyroYangle_2 = pitch_2;
  } else
    kalAngleY_2 = kalmanY_2.getAngle(pitch_2, gyroYrate_2, dt_2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY_2) > 90)
    gyroXrate_2 = -gyroXrate_2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX_2 = kalmanX_2.getAngle(roll_2, gyroXrate_2, dt_2); // Calculate the angle using a Kalman filter
#endif


//Calculate Gyro Angles for 1st IMU
  gyroXangle_1 += gyroXrate_1 * dt_1; // Calculate gyro angle without any filter
  gyroYangle_1 += gyroYrate_1 * dt_1;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX_1 = 0.93 * (compAngleX_1 + gyroXrate_1 * dt_1) + 0.07 * roll_1; // Calculate the angle using a Complimentary filter
  compAngleY_1 = 0.93 * (compAngleY_1 + gyroYrate_1 * dt_1) + 0.07 * pitch_1;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle_1 < -180 || gyroXangle_1 > 180)
    gyroXangle_1 = kalAngleX_1;
  if (gyroYangle_1 < -180 || gyroYangle_1 > 180)
    gyroYangle_1 = kalAngleY_1;

//Calculate Gyro Angles for 1st IMU
  gyroXangle_2 += gyroXrate_2 * dt_2; // Calculate gyro angle without any filter
  gyroYangle_2 += gyroYrate_2 * dt_2;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX_2 = 0.93 * (compAngleX_2 + gyroXrate_2 * dt_2) + 0.07 * roll_2; // Calculate the angle using a Complimentary filter
  compAngleY_2 = 0.93 * (compAngleY_2 + gyroYrate_2 * dt_2) + 0.07 * pitch_2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle_2 < -180 || gyroXangle_2 > 180)
    gyroXangle_2 = kalAngleX_2;
  if (gyroYangle_2 < -180 || gyroYangle_2 > 180)
    gyroYangle_2 = kalAngleY_2;
    
  /* Print Data */

  Serial.print("roll_1: ");
  Serial.print(roll_1); Serial.print("\t");
  Serial.print("pitch_1: ");
  Serial.print(pitch_1); Serial.print("\t\t");
  
  Serial.print("roll_2: ");  
  Serial.print(roll_2); Serial.print("\t");
  Serial.print("pitch_2: ");
  Serial.println(pitch_2); Serial.print("\t");
    
//#if 0 // Set to 1 to activate
//  Serial.print(accX); Serial.print("\t");
//  Serial.print(accY); Serial.print("\t");
//  Serial.print(accZ); Serial.print("\t");
//
//  Serial.print(gyroX); Serial.print("\t");
//  Serial.print(gyroY); Serial.print("\t");
//  Serial.print(gyroZ); Serial.print("\t");
//
//  Serial.print("\t");
//#endif

//  Serial.print(gyroXangle); Serial.print("\t");
//  Serial.print(compAngleX); Serial.print("\t");
//  Serial.print(kalAngleX); Serial.print("\t");

//  Serial.print("\t");

//  Serial.print(gyroYangle); Serial.print("\t");
//  Serial.print(compAngleY); Serial.print("\t");
//  Serial.print(kalAngleY); Serial.print("\t");

//#if 0 // Set to 1 to print the temperature
//  Serial.print("\t");
//
//  double temperature = (double)tempRaw / 340.0 + 36.53;
//  Serial.print(temperature); Serial.print("\t");
//#endif
//
//  Serial.print("\r\n");
  delay(2);
}
