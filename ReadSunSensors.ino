#include <math.h>

// baud: 4800

// select the input pin for LDR
int sensorPin0 = A0; 
int sensorPin1 = A1; 
int sensorPin2 = A2; 
int sensorPin3 = A3; 
int sensorPin4 = A4;
int sensorPin5 = A5;

// variable to store the value coming from the sensor
double sensorValue0 = 0;
double sensorValue1 = 0;
double sensorValue2 = 0;
double sensorValue3 = 0;
double sensorValue4 = 0;
double sensorValue5 = 0;

double beta0 = 0;
double beta1 = 0;
double beta2 = 0;
double beta3 = 0;
double beta4 = 0;
double beta5 = 0;
double maxValue = 0;
double maxAngle = 0;

void setup() {
Serial.begin(9600); //sets serial port for communication 4800 BAUD
}
void loop() {
// read the value from the sensor
sensorValue0 = analogRead(sensorPin0);
sensorValue1 = analogRead(sensorPin1);
sensorValue2 = analogRead(sensorPin2);
sensorValue3 = analogRead(sensorPin3);
sensorValue4 = analogRead(sensorPin4);
sensorValue5 = analogRead(sensorPin5);

//do we want max values for each face?
if (sensorValue0>sensorValue1){
  maxValue = sensorValue0;}
else if (sensorValue1>maxValue){
  maxValue = sensorValue1;}
else if (sensorValue2>maxValue){
  maxValue = sensorValue2;}
else if (sensorValue3>maxValue){
  maxValue = sensorValue3;}
else if (sensorValue4>maxValue){
  maxValue = sensorValue4;}
else if (sensorValue5>maxValue){
  maxValue = sensorValue5;}

beta0 = (180/3.141592654)*asin(sensorValue0/maxValue);
beta1 = (180/3.141592654)*asin(sensorValue1/maxValue);
beta2 = (180/3.141592654)*asin(sensorValue2/maxValue);
beta3 = (180/3.141592654)*asin(sensorValue3/maxValue);
beta4 = (180/3.141592654)*asin(sensorValue4/maxValue);
beta5 = (180/3.141592654)*asin(sensorValue5/maxValue);

// max angle
//if (beta0>beta1){
  //maxAngle = beta0;}
//else if (beta2>maxAngle){
  //maxAngle = beta2;}
//else if (beta3>maxAngle){
  //maxAngle = beta3;}
//else if (beta4>maxAngle){
  //maxAngle = beta4;}
//else if (beta5>maxAngle){
  //maxAngle = beta5;}

//calculate sun vector in body frame 
double sun0 = sin(beta0)*cos(beta2);
double sun1 = sin(beta1)*cos(beta2);
double sun2 = -sin(beta2); 
double sunVector[] = {sun0, sun1, sun2};


//prints the values coming from the sensor on the screen 
//Serial.print("A0 Reading (mA)\t"); 
Serial.println("     ");
Serial.print(sensorValue0,6);
//Serial.print("Beta0 (degrees)\t");
Serial.print("\t");
Serial.println(beta0,6);
//Serial.print("A1 Reading (mA)\t"); 
Serial.print(sensorValue1,6);
//Serial.print("Beta1 (degrees)\t");
Serial.print("\t");
Serial.println(beta1,6);
//Serial.print("A2 Reading (mA)\t"); 
Serial.print(sensorValue2,6);
//Serial.print("Beta2 (degrees)\t");
Serial.print("\t");
Serial.println(beta2,6);
//Serial.print("A3 Reading (mA)\t"); 
Serial.print(sensorValue3,6);
//Serial.print("Beta3 (degrees)\t");
Serial.print("\t");
Serial.println(beta3,6);
//Serial.print("A4 Reading (mA)\t");
Serial.print(sensorValue4,6);
//Serial.print("Beta4 (degrees)\t");
Serial.print("\t");
Serial.println(beta4,6);
//Serial.print("A5 Reading (mA)\t");
Serial.print(sensorValue5,6);
//Serial.print("Beta5 (degrees)\t");
Serial.print("\t");
Serial.println(beta5,6);
Serial.println("     ");

//Serial.print("Max current (mA)\t");
//Serial.println(maxValue,6);
//Serial.print("Max angle (degrees)\t");
//Serial.println(maxAngle,6);

//Serial.print("Sun vector: \t");
Serial.println(sunVector[0]);
Serial.println(sunVector[1]);
Serial.println(sunVector[2]);

delay(2000);
}
