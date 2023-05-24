#ifndef Sensor_h
#define Sensor_h

#include "Arduino.h"


#define READING_NO 10

class Sensor {
  public:
    Sensor(String sensorType, int pin) {
      this->sensorType = sensorType;
      this->pin = pin;
      pinMode(pin, INPUT);
      this->previousTime = 0;
      this->prevReading = 0;
      this->angle = 0;
      this->offset = 0;
    }
    Sensor(String sensorType, int pin, int pin2) {
      this->sensorType = "ULTRASONIC";
      this->pin = pin;
      this->pin2 = pin2;
      this->offset = 0;
      pinMode(pin, OUTPUT);
      pinMode(pin2, INPUT);
      digitalWrite(pin, LOW);
    }
    float getReading();
    String getSensorType();
    void setAngle(double angleValue);
    void setOffset(double offset);
    double getOffset();
    void initialise();

  private:
    String sensorType;
    int pin;
    int pin2;
    double offset;
    int getPin();
    float HC_SR04_range();
    //Smoothing function variables
    int readings[READING_NO];
    int idx = 0;
    int total = 0;
    int average = 0;
    float Smooth(float analogReading);
    //Gyro Stuff
    double previousTime;
    float prevReading;
    float angle;
};

#endif