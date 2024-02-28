#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PWMServo.h>
#include <PID_v1.h>
#include <EEPROM.h>

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

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
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
                digitalToggle(LED_BUILTIN);
                delay(BNO055_SAMPLERATE_DELAY_MS);
            }
            digitalWrite(LED_BUILTIN, LOW);
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

    servo.attach(0);
    servo.write(initialServoValue);
    pid.SetMode(AUTOMATIC);

  
  // Get current degrees for Z axis
  
}

void loop()
{
  // Read gyro sensor data
  sensors_event_t event;
  bno.getEvent(&event);
  // Get current degrees for Z axis
  degree2 = event.gyro.pitch;
  if((millis() - lastVcal)> 100){

    rVelocityZ = (degree2-degree1)/0.1;
    degree1 = degree2;
    lastVcal = millis();
  }

  if((millis() - lastPID_O)> 100){
    PID_O_R = (outputPID-PID_O_1)/0.1;
    PID_O_1 = outputPID;
    lastPID_O = millis();
  }
  
  input = rVelocityZ; 
  pid.Compute();                      // Get correction value for Z axis


  // Apply correction value for Z axis
  int newZ;
  newZ = PID_O_R;
  //Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  if((millis() - lastdata)> 500){
  Serial.print("rVelocityZ: ");
  Serial.println(rVelocityZ);
  Serial.print("PIDrate: ");
  Serial.println(PID_O_R);
  //Serial.print("newZ: ");
  //Serial.println(newZ);
  lastdata = millis();
  }
  // Limit servo to min 0, or max 180 degrees
  if (newZ < -90) {
    newZ = -90;
  } else if (newZ > 90) {
    newZ = 90;
  }
  //servo.write(newZ + initialServoValue); // Add initialServoValue to make sure Servo is between 0 and 180
}