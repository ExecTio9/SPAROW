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
double desired, input, output;
double kp = 1.25, ki = 0.04, kd = 0.1;
PID pid(&input, &output, &desired, kp, ki, kd, DIRECT);

const int initialServoValue = 90;

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

    desired = 0;
    servo.attach(0);
    servo.write(initialServoValue);
    pid.SetMode(AUTOMATIC);
}

void loop()
{
  // Read gyro sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  // Get current degrees for Z axis
  int degreeZ = event.gyro.pitch;
  input = abs(degreeZ);               // Calculate to absolute degrees
  pid.Compute();                      // Get correction value for Z axis

  // Apply correction value for Z axis
  int newZ;
  if (degreeZ < 0) {
    newZ = degreeZ + output;          // If gyro was moved counter clock-wise, add correction value
  } else {
    newZ = degreeZ - output;          // If gyro was moved clock-wise, subtract correction value
  }

  // Limit servo to min 0, or max 180 degrees
  if (newZ < -90) {
    newZ = -90;
  } else if (newZ > 90) {
    newZ = 90;
  }
  servo.write(newZ + initialServoValue); // Add initialServoValue to make sure Servo is between 0 and 180
}