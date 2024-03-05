#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <math.h>
#include <PWMServo.h>
#include <PID_v1.h>

unsigned long millisOld;
float pitchM, rollM, pitch, roll, dt;

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


// Pitch PID loop
double desired_Pitch = 0;
double input_Pitch;
double outputPID_Pitch;
double kp_Pitch = 0.5, ki_Pitch = 0.5, kd_Pitch = 0.5;
PID pidP(&input_Pitch, &outputPID_Pitch, &desired_Pitch, kp_Pitch, ki_Pitch, kd_Pitch, DIRECT);

// Roll PID loop
double desired_Roll = 0;
double input_Roll;
double outputPID_Roll;
double kp_Roll = 0.5, ki_Roll = 0.5, kd_Roll = 0.5;
PID pidR(&input_Roll, &outputPID_Roll, &desired_Roll, kp_Roll, ki_Roll, kd_Roll, DIRECT);

// Rotational Velocity
int lastdata;
int lastVcal = 0;
int lastDegCal = 0;
float degree_Pitch_1 = 0;
float degree_Pitch_2;
float degree_Roll_1 = 0;
float degree_Roll_2;
float rVelocity_Pitch;
float rVelocity_Roll;

// Looking at the back of the plane
PWMServo leftAileron;
PWMServo rightAileron;
PWMServo leftElevator;
PWMServo rightElevator;


void setup()
{

  leftAileron.attach(0);
  rightAileron.attach(1);
  leftElevator.attach(2);
  rightElevator.attach(3);

  Serial.begin(115200);
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  if(bno.begin())
  {
    EEPROM.get(eeAddress, bnoID);
    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;
    bno.getSensor(&sensor);
    if(bnoID==sensor.sensor_id)
    {
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);
      bno.setSensorOffsets(calibrationData);
      foundCalib = true;
    }
    bno.setExtCrystalUse(true);
    sensors_event_t event;
    bno.getEvent(&event);
    if(!foundCalib)
    {
      while(!bno.isFullyCalibrated())
      {
        bno.getEvent(&event);
        delay(BNO055_SAMPLERATE_DELAY_MS);
      }
      adafruit_bno055_offsets_t newCalib;
      bno.getSensorOffsets(newCalib);
      eeAddress = 0;
      bno.getSensor(&sensor);
      bnoID = sensor.sensor_id;
      EEPROM.put(eeAddress, bnoID);
      eeAddress += sizeof(long);
      EEPROM.put(eeAddress, newCalib);
    }
    
  }
  millisOld = millis();


  leftAileron.write(0);
  rightAileron.write(0);
  leftElevator.write(0);
  rightElevator.write(0);

    
  // Get current degrees for P axis
  
}

void loop()
{
  if((millis() - lastdata)> 100){
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  pitchM = atan2(acc.x()/9.8, acc.z()/9.8)/2/3.141592654*360; // Pitch based off acceleration
  rollM = atan2(acc.y()/9.8, acc.z()/9.8)/2/3.141592654*360;

  dt = (millis()-millisOld)/1000.0;
  millisOld = millis();
  
  pitch = (pitch-gyro.y()*dt)*0.98 + pitchM*0.02; 
  roll = (roll+gyro.x()*dt)*0.98 + rollM*0.02;

  lastdata = millis();
  }

  

  if((millis() - lastVcal)> 100){               // Calculate Rotational Velocity on all axis
    rVelocity_Pitch = (degree_Pitch_2 - degree_Pitch_1)/0.1;
    degree_Pitch_1 = degree_Pitch_2;
    rVelocity_Roll = (degree_Roll_2 - degree_Roll_1)/0.1;
    degree_Roll_1 = degree_Roll_2;
    lastVcal = millis();
  }

  
  
  if((millis() - lastdata)> 100){
  printf("rVelocityP: ,%f,   PIDoutput: ,%f,\n",rVelocity_Pitch,outputPID_Pitch);
  lastdata = millis();
  }

  // Limit servo to min 0, or max 180 degrees
  if (outputPID_Pitch < -90) {
    outputPID_Pitch = -90;
  } else if (outputPID_Pitch > 90) {
    outputPID_Pitch = 90;
  }
  if (outputPID_Roll < -90) {
    outputPID_Roll = -90;
  } else if (outputPID_Roll > 90) {
    outputPID_Roll = 90;
  }
  //servo.write(ServoInputP); // Add initialServoValue to make sure Servo is between 0 and 180
  
  // leftAileron.write(outputPID_Roll);
  // rightAileron.write(outputPID_Roll);
  // leftElevator.write(outputPID_Pitch);
  // rightElevator.write((outputPID_Pitch)*1);

}