void Calibration_IR(int pin, int ranges[], int arraySize){
  for (int i = 0; i < arraySize; i++){
    Serial.flush();
    Serial.print("Place Robot "); Serial.print(ranges[i]); Serial.println("mm Away from a Wall");
    while(Serial.available() == 1){}
    Serial.read();
    for (int i = 0; i < 20; i++){Smoothing(pin);}
    results[i] = Smoothing(pin);
    Serial.print("Actual Reading: "); Serial.println(results[i]);
  }
}


void Calibration_US(int ranges[], int arraySize){
  for (int i = 0; i < arraySize; i++){
    Serial.flush();
    Serial.print("Place Robot "); Serial.print(ranges[i]); Serial.println("mm Away from a Wall");
    while(Serial.available() == 1){}
    Serial.read();
    results[i] = HC_SR04_range();
    Serial.print("Actual Reading: "); Serial.println(results[i]);
    
  }
}


void Calibration_GYRO(int ranges[], int arraySize){
  for (int i = 0; i < arraySize; i++){
    Serial.flush();
    Serial.print("Place Robot "); Serial.print(ranges[i]); Serial.println("mm Away from a Wall");
    while(Serial.available() == 1){}
    Serial.read();
    results[i] = analogRead(GYRO);
    Serial.print("Actual Reading: "); Serial.println(results[i]);
  }
}
