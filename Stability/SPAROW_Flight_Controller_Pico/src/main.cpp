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
const int initialServoValue = 100;

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
PWMServo servo;

// Initialize PID algorithm
double desired = 0;
double input, outputPID;
double kp = 0.5, ki = 0.5, kd = 0.5;
PID pid(&input, &outputPID, &desired, kp, ki, kd, DIRECT);

const int initialServoValue = 90;

int lastdata;
int lastVcal = 0;
int lastPID_O = 0;
float degree1 = 0;
float degree2;
float rVelocityZ;
float PID_O_1 = 0;
float PID_O_R = 0;
PWMServo pitchServo;

void setup()
{
  pitchServo.attach(0);
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
  pitchServo.write(initialServoValue);

    servo.attach(0);
    servo.write(initialServoValue);
    pid.SetMode(AUTOMATIC);

  
  // Get current degrees for Z axis
  
}

void loop()
{
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  pitchM = atan2(acc.x()/9.8, acc.z()/9.8)/2/3.141592654*360;
  rollM = atan2(acc.y()/9.8, acc.z()/9.8)/2/3.141592654*360;

  dt = (millis()-millisOld)/1000.0;
  millisOld = millis();
  
  pitch = (pitch-gyro.y()*dt)*0.98 + pitchM*0.02;
  roll = (roll+gyro.x()*dt)*0.98 + rollM*0.02;

  if((millis() - lastVcal)> 100){               // Calculate Rotational Velocity on all axis
    rVelocityZ = (degree2-degree1)/0.1;
    degree1 = degree2;
    lastVcal = millis();
  }

  // if((millis() - lastPID_O)> 100){
  //   PID_O_R = (outputPID-PID_O_1)/0.1;
  //   PID_O_1 = outputPID;
  //   lastPID_O = millis();
  // }

  // Apply correction value for Z axis
  int ServoInputZ;
  ServoInputZ = PID_O_R;
  //Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  if((millis() - lastdata)> 100){
  printf("rVelocityZ: ,%f,   PIDrate: ,%f,\n",rVelocityZ,PID_O_R);
  /*
  Serial.print("rVelocityZ: ");
  Serial.println(rVelocityZ);
  Serial.print("PIDrate: ");
  Serial.println(PID_O_R);
  */
  //Serial.print("newZ: ");
  //Serial.println(newZ);
  lastdata = millis();
  }
  // Limit servo to min 0, or max 180 degrees
  if (ServoInputZ < -90) {
    ServoInputZ = -90;
  } else if (ServoInputZ > 90) {
    ServoInputZ = 90;
  }
  //servo.write(newZ + initialServoValue); // Add initialServoValue to make sure Servo is between 0 and 180
  delay(BNO055_SAMPLERATE_DELAY_MS);
}