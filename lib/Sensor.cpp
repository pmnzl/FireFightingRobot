#include "Arduino.h"
#include "Sensor.h"
#include "math.h"

float Sensor::Smooth(float analogReading) {
  total = total - readings[idx];      //Subtract the last reading
  readings[idx] = analogReading;    //Read from the sensor
  total = total + readings[idx];      //Add the reading to the total
  idx = idx + 1;                			//Advance to the next position in the array

  if (idx >= READING_NO) {           	//If we're at the end of the array
    idx = 0;                          //Wrap around to the beginning
  }

  average = total/READING_NO;         //Calculate the average:
  delay(1);                           //Delay in between reads for stability
  return average;            	        //Return the average reading
}

int Sensor::getPin() {
  return this->pin;
}

String Sensor::getSensorType() {
  return this->sensorType;
}

void Sensor::setAngle(double angleValue) {
  this->angle = (angleValue-2.1-this->offset)*1418;
}

void Sensor::setOffset(double offset){
  this->offset = offset;
}

double Sensor::getOffset(){
  return this->offset;
}

float Sensor::getReading() {
  float val = 0.0;

  if (this->getSensorType() == "IRTRANSISTOR") {
    val = this->Smooth(analogRead(this->pin)) + this->offset;
    return val;
  }
  else if (this->getSensorType() == "IRSHORT") {
    val = 40546.45815 * pow(this->Smooth(analogRead(this->pin)), -1.092180602) + this->offset;
    if (val > 250) return 250;
    else if (val < 40) return 39;
    else return val;
  }
  else if (this->getSensorType() == "IRLONG") {
    val = 152246.3973*pow(this->Smooth(analogRead(this->pin)), -1.184450627)+this->offset;
    if(val > 250) return 251;
    else if(val < 75) return 74;
	else return val;
  }
  else if (this->getSensorType() == "ULTRASONIC") {
    val = this->Smooth(this->HC_SR04_range())+this->offset;
    if(val > 2000) return 2001;
		else if(val < 40) return 39;
		else return val;
  }
  else if (this->getSensorType() == "GYROSCOPE") {
    double currentTime = millis();
    double elapedTime = (double)(currentTime - this->previousTime);
    val = this->Smooth(analogRead(this->pin))-523;
    if (abs(val - prevReading) <= 3) {val = prevReading;}    
    angle += val*elapedTime;
    previousTime = currentTime;
    return this->offset+(2.1+(angle)/1418);
  }
  else {return 0;}
}

void Sensor::initialise() {
  if (this->getSensorType() == "IRTRANSISTOR") {
    for (int i = 0; i < READING_NO; i++){this->getReading();}
  }
  else if (this->getSensorType() == "IRSHORT") {
    for (int i = 0; i < READING_NO; i++){this->getReading();}
  }
  else if (this->getSensorType() == "IRLONG") {
    for (int i = 0; i < READING_NO; i++){this->getReading();}
  }
  else if (this->getSensorType() == "ULTRASONIC") {
    for (int i = 0; i < READING_NO; i++){this->getReading();}
  }
  else if (this->getSensorType() == "GYROSCOPE") {
    for (int i = 0; i < READING_NO; i++){this->getReading();}
    this->setAngle(0);
  }
  else {return;}
}


float Sensor::HC_SR04_range() {
  const int TRIG_PIN = this->pin;
  const int ECHO_PIN = this->pin2;
  const int MAX_DIST = 23200;
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      Serial.println("HC-SR04: NOT found");
      return 9999;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      //Serial.println("HC-SR04: Out of range");
      return 9999;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    //Serial.println("HC-SR04: Out of range");
  } else {
    return cm*10;
  }
  return 9999;
}