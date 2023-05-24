/* RANGE CALIBRATION FOR IR AND ULTRASONIC SENSORS */

/* SENSOR ID LIST
 *   Name   | Type    | ID# | Port#
 * ---------------------------------------
 *   IR_FL  | IR Long |  1  |  A4
 *   IR_FR  | IR Long |  2  |  A5
 *   IR_LF  | IR Med  |  3  |  A6
 *   IR_LR  | IR Med  |  4  |  A7
 *   GYRO   | GYRO    |  5  |  A3
 *   US     | RADAR   |  6  |  T48/E49
*/

#include <math.h>
#include <stdlib.h>

#define TEST_NO 10
#define IR_FL A4
#define IR_FR A5
#define IR_LF A6
#define IR_LR A7
#define GYRO A3
#define TRIG_PIN 48
#define ECHO_PIN 49
#define MAX_DIST 23200

int Ranges_IR_Long [TEST_NO] = {75, 95, 115, 135, 155, 175, 195, 215, 235, 255}; 
int Ranges_IR_Med [TEST_NO] = {40, 50, 60, 70, 80, 90, 100, 110, 120, 130}; 

//70,100,130,160,190,220,250,280,300};
int Ranges_Gyro [TEST_NO] = {0,0,45,90,135,180,225,270,315,360};
int Ranges_Ultrasonic [TEST_NO] = {20,220,420,620,820,1020,1220,1420,1620,1820};

//Smoothing Filter Variables
const int numReadings = 10; //Smoothing Array
int readings[numReadings];  //Readings from sensor
int readIndex = 0;          //Current reading index
int total = 0;              //Running total
int average = 0;            //Average


int ID = -1;

int ranges [TEST_NO];
float results [TEST_NO];

void Calibration();
void Calibration_IR(int pin, int ranges[], int arraySize);
void Calibration_US(int ranges[]);
void Calibration_GYRO(int ranges[], int arraySize);
void printResults(int ranges[], int arraySize);
double HC_SR04_range();


void setup() {
  pinMode(IR_FL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_LF, INPUT);
  pinMode(IR_LR, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  digitalWrite(TRIG_PIN, LOW);
  
  Serial.begin(9600);
}

void loop() {
  Calibration();
  ID = -1;
  Serial.println();
  Serial.println();
}

void Calibration(){
  goto Error;
  Error:
  Serial.print("Enter Sensor ID: ");
  while (ID == -1){ID = Serial.read();}
  switch(ID){
    case 49: //1
      Serial.println("1");
      Calibration_IR(IR_FL, Ranges_IR_Long, TEST_NO);
      printResults(Ranges_IR_Long, TEST_NO);
      break;
    case 50: //2
      Serial.println("2");
      Calibration_IR(IR_FR, Ranges_IR_Long, TEST_NO);
      printResults(Ranges_IR_Long, TEST_NO);
      break;
    case 51: //3
      Serial.println("3");
      Calibration_IR(IR_LF, Ranges_IR_Med, TEST_NO);
      printResults(Ranges_IR_Med, TEST_NO);
      break;
    case 52: //4
      Serial.println("4");
      Calibration_IR(IR_LR, Ranges_IR_Med, TEST_NO);
      printResults(Ranges_IR_Med, TEST_NO);
      break;
    case 53: //5
      Serial.println("5");
      Calibration_GYRO(Ranges_Gyro, TEST_NO);
      printResults(Ranges_Gyro, TEST_NO);
      break;
    case 54: //6
      Serial.println("6");
      Calibration_US(Ranges_Ultrasonic, TEST_NO);
      printResults(Ranges_Ultrasonic, TEST_NO);
      break;
    default:
      Serial.println("ERROR: ID NOT VALID!");
      ID = -1;
      goto Error;
      break;
  }
  Serial.println();
}

void printResults(int ranges[], int arraySize){
  Serial.println("Range\tReading");
  for (int i = 0; i < arraySize; i++){
    Serial.print(ranges[i]); Serial.print("mm\t"); Serial.println(results[i]);
  }
  Serial.println();
}
