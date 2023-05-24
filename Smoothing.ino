float Smoothing(int pin){
  total = total - readings[readIndex];         //Subtract the last reading
  readings[readIndex] = analogRead(pin);       //Read from the sensor
  total = total + readings[readIndex];         //Add the reading to the total
  readIndex = readIndex + 1;                   //Advance to the next position in the array

  if (readIndex >= numReadings) {              //If at the end of the array
    readIndex = 0;                             //Wrap around to the beginning
  }

  average = total / numReadings;               //Calculate the average:
  delay(1);                                    //Delay in between reads for stability
  return average;                              //Return the average reading
}

float Smoothing_US(int pin){
  total = total - readings[readIndex];         //Subtract the last reading
  readings[readIndex] = HC_SR04_range();       //Read from the sensor
  total = total + readings[readIndex];         //Add the reading to the total
  readIndex = readIndex + 1;                   //Advance to the next position in the array

  if (readIndex >= numReadings) {              //If at the end of the array
    readIndex = 0;                             //Wrap around to the beginning
  }

  average = total / numReadings;               //Calculate the average:
  delay(1);                                    //Delay in between reads for stability
  return average;                              //Return the average reading
}
