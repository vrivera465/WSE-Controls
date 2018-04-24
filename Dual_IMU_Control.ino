#include <Wire.h> //Necessary Library for the TCA9548A Multiplexor 
#include <Adafruit_Sensor.h> //Necessary Library to for the IMUs
#include <Adafruit_BNO055.h> //Necessary Library to perform IMUs
#include <utility/imumaths.h> //Necessary Library to perform different math with the IMU values 
#include <EEPROM.h> ////Necessary Library to save calibration status
#include <BasicLinearAlgebra.h> //Necessary Library to perform Matrix Multiplication
using namespace BLA; //needed to perform Matrix Multiplication

#define TCAADDR 0x70 //Needed to work with the Multiplexor 
#define BNO055_SAMPLERATE_DELAY_MS (500) //Defines the default time delay used throughout the code
uint8_t tca; // Defines the variable dictating the tca port being used in the multiplexor 

/* This code gives the  quaternion angles of each sensor 
    along with each sensors sensor calibration value. Code was adapted from 
    Adafruit BNO055 and TCA9548A code found at
    https://github.com/adafruit/Adafruit_BNO055 and
    https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
    Specific files used were restoreoffsets and rawdata 
*/

/* The IMU located on the arm should correspond to Sensor 1 connections 
  (IMU port 2 on the circuit board) and the IMU on the back should correspond 
  to Sensor 2 connections (IMU port 3 on the circuit board).
   If the two sensors are switched, the matrix multiplcation will be 
   inaccurate and the Arm coordinate system is meant to be in reference to
   the IMU on the back.
 */

/* Assign a unique ID to this sensor at the same time */
Adafruit_BNO055 bno = Adafruit_BNO055(55); // Defines the address used by the IMU

void tcaselect(uint8_t tca) { //Scans the multiplexor and determines what ports are being read and transmits the values from those ports 
  if (tca > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << tca);
  Wire.endTransmission();  
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (system != 3)
  {
    Serial.print("!");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);

  if (system == 3 && accel == 3 && gyro == 3 && mag == 3)
  {
    Serial.print("Fully Calibrated");
  }
 
}


void setup(void)
{
  Serial.begin(9600); //Initializes the Serial Monitor 
  
  Serial.println("Orientation Sensor Test"); Serial.println("");

   Wire.begin(); //Initializes the multiplexor 

   //**************************SENSOR 1 *************************
   tca = 1; //defines sensor one as the sensor located on the first SCL/SDA bus on the multiplexor
   tcaselect(tca); 
   if(!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, BNO055 1 not detected ... Check your wiring or I2C ADDR!");
      while(1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    //getting bnoID for sensor 1
    EEPROM.get(eeAddress, bnoID);

    //blank calibration data struct
    adafruit_bno055_offsets_t calibrationData;
    //blank sensor struct
    sensor_t sensor;

    //populates the sensor struct so now points to sensor 1
    bno.getSensor(&sensor);

    //here we check to make sure that the sensor id for sensor 1 is actually in the hash
    //table
    if (bnoID != sensor.sensor_id)
    {
      //if its not then we know we have never stored that value -- so no calibration data exists
      Serial.println("\nNo Calibration Data for Sensor 1 exists in EEPROM");
      delay(500);
    }
    else //otherwise we have calibration data for sensor 1
    {
      Serial.println("\nFound Calibration for Sensor 1 in EEPROM.");

      //to get calibration data, we look at the next slot in the hash table 
      //which is eeAddress (0) + the size of an int
      eeAddress += sizeof(long);

      //populate the empty calibration data struct with the calibration data for sensor 1
      EEPROM.get(eeAddress, calibrationData);

      //display sensor 1 offsets
      //displaySensorOffsets(calibrationData);

      Serial.println("\n\nRestoring Calibration data for Sensor 1 to the BNO055...");

      //restores the sensor 1 offsets
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data for Sensor 1 loaded into BNO055");
      //calibrated data
      foundCalib = true;
    }

    //Store calibrated data
    eeAddress = 0; //we want to store the bnoID for sensor 1
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id; //set bnoID equal to sensor 1's id

    //updating the sensor id information
    EEPROM.put(eeAddress, bnoID);

    //update the calibration data information
    eeAddress += sizeof(long); 
    EEPROM.put(eeAddress, calibrationData);
    Serial.println("Data stored to EEPROM.");
  
   

    //**********************SENSOR 2**************************
    tca = 2;
    tcaselect(tca); 
    if(!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, BNO055 2 not detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    
    //bnoID for sensor 2
    eeAddress += sizeof(long);
    foundCalib = false; //no longer true so set to false
    EEPROM.get(eeAddress, bnoID); //store sensor 2 id information in bnoID

    //populates the sensor struct so now points to sensor 2
    bno.getSensor(&sensor);

    int calibrationDataSizeSensor1 = sizeof(calibrationData);

    //here we check to make sure that the sensor id for sensor 2 is actually in the hash
    //table
    if (bnoID != sensor.sensor_id)
    {
      //if its not then we know we have never stored that value -- so no calibration data exists
      Serial.println("\nNo Calibration Data for Sensor 2 exists in EEPROM");
      delay(500);
    }
    else //otherwise we have calibration data for sensor 2
    {
      Serial.println("\nFound Calibration for Sensor 2 in EEPROM.");

      //to get calibration data, we look at the next slot in the hash table 
      //which is eeAddress (0) + the size of an int + the size of an int + the size of an int
      eeAddress += calibrationDataSizeSensor1;

      //populate the empty calibration data struct with the calibration data for sensor 2
      EEPROM.get(eeAddress, calibrationData);

      //display sensor 2 offsets
      //displaySensorOffsets(calibrationData);

      Serial.println("\n\nRestoring Calibration data for Sensor 2 to the BNO055...");

      //restores the sensor 2 offsets
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data for Sensor 2 loaded int BNO055");
      //calibrated data
      foundCalib = true;
    }

    //Store calibrated data
    eeAddress = (2* sizeof(long)) + calibrationDataSizeSensor1; //we want to store the bnoID for sensor 2
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id; //set bnoID equal to sensor 2's id

    //updating the sensor id information
    EEPROM.put(eeAddress, bnoID);

    //update the claibration data information
    eeAddress += sizeof(long); 
    EEPROM.put(eeAddress, calibrationData);
    Serial.println("Data stored to EEPROM.");

 
  delay(BNO055_SAMPLERATE_DELAY_MS);

}

void loop(void)
{
  sensors_event_t event; //Initializes event coming from the sensor 
  bno.getEvent(&event); //Calls function in the header file of the IMU telling it to start collecting data

    BLA:: Matrix<3,3> A; //Defines rotation matrix created by IMU 1
    A.Fill(0); // Fills matrix with 0s to initialize it 
    
    BLA:: Matrix<3,3> B; //Defines rotation matrix created by IMU 2
    B.Fill(0); // Fills matrix with 0s to initialize it

    BLA::Matrix<3,3> B_inv; //Defines inverse of the rotation matrix created by IMU 1
    B_inv.Fill(0); // Fills matrix with 0s to initialize it
    
    BLA:: Matrix<3,3> C; // Define the product from the multiplication of B_inv and A 
    C.Fill(0); // Fills matrix with 0s to initialize it

  // Quaternion data

for (tca = 1; tca <=2; tca++)
{
  tcaselect(tca);
    imu::Quaternion quat = bno.getQuat();
    Serial.print("Sensor #"); Serial.print(tca, 1); //Serial.println(""); // Prints the sensor name based on the tca line it's connected to
// Prints out the angle values for the sensor in quaternion 
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");

    displayCalStatus(); // Displays the calibration status for each sensor
    Serial.println("");

     // Generate the Roll (change in x), Pitch (change in y), and Yaw (change in z) Angles from the quaternion data 
    double xx = quat.x() * quat.x(); // represent x^2
    double yy = quat.y() * quat.y(); // represent y^2
    double zz = quat.z() * quat.z(); // represent z^2
    
    double roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2*(xx + yy));
    double pitch = asin(2 * quat.w() * quat.y() - quat.x() * quat.z());
    double yaw = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2*(yy + zz));
    
    //Convert Radians to Degrees 
    float rollDeg = 57.2958 * roll; 
    float pitchDeg = 57.2958 * pitch;
    float yawDeg = 57.2958 * yaw;

    // Display Roll, Pitch and Yaw in Degrees  
    
    Serial.print("Roll:"); Serial.print(rollDeg,2); Serial.print(" degrees"); Serial.println("");
    Serial.print("Pitch:"); Serial.print(pitchDeg,2); Serial.print(" degrees"); Serial.println("");
    Serial.print("Yaw:"); Serial.print(yawDeg,2); Serial.print(" degrees"); Serial.println("");

    if (tca == 1) // For the Sensor plugged into port 1, creates a rotation matrix based off the IMU data 
    {
    A(1,1) = quat.w()*quat.w() + quat.x()*quat.x() - quat.y()*quat.y() - quat.z()*quat.z(); 
    A(1,2) = 2*quat.x()*quat.y() - 2*quat.w()*quat.z();
    A(1,3) = 2*quat.x()*quat.z() + 2*quat.w()*quat.y();
    A(2,1) = 2*quat.x()*quat.y() + 2*quat.w()*quat.z();
    A(2,2) = quat.w()*quat.w() - quat.x()*quat.x() + quat.y()*quat.y() - quat.z()*quat.z();
    A(2,3) = 2*quat.y()*quat.z() - 2*quat.w()*quat.x();
    A(3,1) = 2*quat.x()*quat.z() - 2*quat.w()*quat.y();
    A(3,2) = 2*quat.y()*quat.z() + 2*quat.w()*quat.x();
    A(3,3) = quat.w()*quat.w() - quat.x()*quat.x() - quat.y()*quat.y() + quat.z()*quat.z();

      // Displays each value in the Rotation Matrix
    Serial << "A: " << A << '\n'; //Prints full matrix at once 
    // prints out each individiual value in the matrix separately 
    Serial.print ("Matrix A:"); Serial.println("");
    Serial.print (A(1,1), 5); Serial.print("  \t");
    Serial.print (A(1,2), 5); Serial.print("  \t");
    Serial.print (A(1,3), 5); Serial.println("  \t");
    Serial.print (A(2,1), 5); Serial.print("  \t");
    Serial.print (A(2,2), 5); Serial.print("  \t");
    Serial.print (A(2,3), 5); Serial.println("  \t");
    Serial.print (A(3,1), 5); Serial.print("  \t");
    Serial.print (A(3,2), 5); Serial.print("  \t");
    Serial.print (A(3,3), 5); Serial.println("  \t"); 
    }
    else if (tca == 2)
    {
// For the Sensor plugged into port 2, creates a rotation matrix based off the IMU data 
    imu::Quaternion quat = bno.getQuat();
    B(1,1) = quat.w()*quat.w() + quat.x()*quat.x() - quat.y()*quat.y() - quat.z()*quat.z(); 
    B(1,2) = 2*quat.x()*quat.y() - 2*quat.w()*quat.z();
    B(1,3) = 2*quat.x()*quat.z() + 2*quat.w()*quat.y();
    B(2,1) = 2*quat.x()*quat.y() + 2*quat.w()*quat.z();
    B(2,2) = quat.w()*quat.w() - quat.x()*quat.x() + quat.y()*quat.y() - quat.z()*quat.z();
    B(2,3) = 2*quat.y()*quat.z() - 2*quat.w()*quat.x();
    B(3,1) = 2*quat.x()*quat.z() - 2*quat.w()*quat.y();
    B(3,2) = 2*quat.y()*quat.z() + 2*quat.w()*quat.x();
    B(3,3) = quat.w()*quat.w() - quat.x()*quat.x() - quat.y()*quat.y() + quat.z()*quat.z();
    
   
    // Displays each value in the Rotation Matrix
    Serial << "B: " << B << '\n';//Prints full matrix at once 
    // prints out each individiual value in the matrix separately 
    Serial.print("Matrix B:"); Serial.println("");
    Serial.print (B(1,1), 5); Serial.print("  \t");
    Serial.print (B(1,2), 5); Serial.print("  \t");
    Serial.print (B(1,3), 5); Serial.println("  \t");
    Serial.print (B(2,1), 5); Serial.print("  \t");
    Serial.print (B(2,2), 5); Serial.print("  \t");
    Serial.print (B(2,3), 5); Serial.println("  \t");
    Serial.print (B(3,1), 5); Serial.print("  \t");
    Serial.print (B(3,2), 5); Serial.print("  \t");
    Serial.print (B(3,3), 5); Serial.println("  \t");  

//Manually creates the inverse of the Matrix for IMU 2
    B_inv(1,1) = quat.w()*quat.w() + quat.x()*quat.x() - quat.y()*quat.y() - quat.z()*quat.z(); 
    B_inv(1,2) = 2*quat.x()*quat.y() + 2*quat.w()*quat.z();
    B_inv(1,3) = 2*quat.x()*quat.z() - 2*quat.w()*quat.y();
    B_inv(2,1) = 2*quat.x()*quat.y() - 2*quat.w()*quat.z();
    B_inv(2,2) = quat.w()*quat.w() - quat.x()*quat.x() + quat.y()*quat.y() - quat.z()*quat.z();
    B_inv(2,3) = 2*quat.y()*quat.z() + 2*quat.w()*quat.x();
    B_inv(3,1) = 2*quat.x()*quat.z() + 2*quat.w()*quat.y();
    B_inv(3,2) = 2*quat.y()*quat.z() - 2*quat.w()*quat.x();
    B_inv(3,3) = quat.w()*quat.w() - quat.x()*quat.x() - quat.y()*quat.y() + quat.z()*quat.z();

//Prints each value in the inverse matrix 
    Serial.print("Matrix B_inv:"); Serial.println("");
    Serial.print (B_inv(1,1), 5); Serial.print("  \t");
    Serial.print (B_inv(1,2), 5); Serial.print("  \t");
    Serial.print (B_inv(1,3), 5); Serial.println("  \t");
    Serial.print (B_inv(2,1), 5); Serial.print("  \t");
    Serial.print (B_inv(2,2), 5); Serial.print("  \t");
    Serial.print (B_inv(2,3), 5); Serial.println("  \t");
    Serial.print (B_inv(3,1), 5); Serial.print("  \t");
    Serial.print (B_inv(3,2), 5); Serial.print("  \t");
    Serial.print (B_inv(3,3), 5); Serial.println("  \t");
    }
    else
    { 
      // if no IMUs are detected prints empty to indicate there is an error
      Serial.println("empty");
    }

    
}

   //Manually multiplies the inverse of IMU 2 and IMU 1 to create a new reference frame for IMU 1 
    C(1,1) = B_inv(1,1)*A(1,1) + B_inv(1,2)*A(2,1) + B_inv(1,3)*A(3,1);
    C(1,2) = B_inv(1,1)*A(1,2) + B_inv(1,2)*A(2,2) + B_inv(1,3)*A(3,2);
    C(1,3) = B_inv(1,1)*A(1,3) + B_inv(1,2)*A(2,3) + B_inv(1,3)*A(3,3);
    C(2,1) = B_inv(2,1)*A(1,1) + B_inv(2,2)*A(2,1) + B_inv(2,3)*A(3,1);
    C(2,2) = B_inv(2,1)*A(1,2) + B_inv(2,2)*A(2,2) + B_inv(2,3)*A(3,2);
    C(2,3) = B_inv(2,1)*A(1,3) + B_inv(2,2)*A(2,3) + B_inv(2,3)*A(3,3);
    C(3,1) = B_inv(3,1)*A(1,1) + B_inv(3,2)*A(2,1) + B_inv(3,3)*A(3,1);
    C(3,2) = B_inv(3,1)*A(1,2) + B_inv(3,2)*A(2,2) + B_inv(3,3)*A(3,2);
    C(3,3) = B_inv(3,1)*A(1,3) + B_inv(3,2)*A(2,3) + B_inv(3,3)*A(3,3);

//Prints the values in matrix C
    Serial.print("Matrix C:");Serial.println("");
    Serial.print (C(1,1), 5); Serial.print("  \t");
    Serial.print (C(1,2), 5); Serial.print("  \t");
    Serial.print (C(1,3), 5); Serial.println("  \t");
    Serial.print (C(2,1), 5); Serial.print("  \t");
    Serial.print (C(2,2), 5); Serial.print("  \t");
    Serial.print (C(2,3), 5); Serial.println("  \t");
    Serial.print (C(3,1), 5); Serial.print("  \t");
    Serial.print (C(3,2), 5); Serial.print("  \t");
    Serial.print (C(3,3), 5); Serial.println("  \t");  

  // Generate the Roll (change in x), Pitch (change in y), and Yaw (change in z) Angles from the quaternion data 
    
    double roll = atan(C(3,2)/C(3,3));
    double pitch = atan(-C(3,1)/(sqrt(sq(C(3,2))+sq(C(3,3)))));
    double yaw = atan(C(2,1)/C(1,1));
    
    //Convert Radians to Degrees 
    float rollArm = 57.2958 * roll; 
    float pitchArm = 57.2958 * pitch;
    float yawArm = 57.2958 * yaw;

    // Display Roll, Pitch and Yaw in Degrees  
    
    Serial.print("Arm Roll: "); Serial.print(rollArm,2); Serial.print(" degrees"); Serial.println("");
    Serial.print("Arm Pitch: "); Serial.print(pitchArm,2); Serial.print(" degrees"); Serial.println("");
    Serial.print("Arm Yaw: "); Serial.print(yawArm,2); Serial.print(" degrees"); Serial.println("");

  delay(BNO055_SAMPLERATE_DELAY_MS);

}
