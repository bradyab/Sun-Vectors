#include <math.h>

// baud: 4800

// select the input pin for LDR
int sensorPin1 = A1; 
int sensorPin2 = A2; 
int sensorPin3 = A3; 
int sensorPin4 = A4; 
int sensorPin5 = A5;
int sensorPin6 = A6;

// variable to store the value coming from the sensor
double maxValue1 = 0;
double Value1 = 0;
double maxValue2 = 0;
double Value2 = 0;
double maxValue3 = 0;
double Value3 = 0;
double maxValue4 = 0;
double Value4 = 0;
double maxValue5 = 0;
double Value5 = 0;
double maxValue6 = 0;
double Value6 = 0;

double beta1 = 0;
double beta2 = 0;
double beta3 = 0;
double beta4 = 0;
double beta5 = 0;
double beta6 = 0;

void setup() {
Serial.begin(9600); //sets serial port for communication 4800 BAUD
}
void loop() {
// read the value from the sensor
Value1 = analogRead(sensorPin1);
Value2 = analogRead(sensorPin2);
Value3 = analogRead(sensorPin3);
Value4 = analogRead(sensorPin4);
Value5 = analogRead(sensorPin5);

//We want to normalize values for each face by dividing by their maximum value
if (Value1>maxValue1){
  maxValue1 = Value1;}
if (Value2>maxValue2){
  maxValue2 = Value2;}
if (Value3>maxValue3){
  maxValue3 = Value3;}
if (Value4>maxValue4){
  maxValue4 = Value4;}
if (Value5>maxValue5){
  maxValue5 = Value5;}
if (Value6>maxValue6){
  maxValue6 = Value6;}

beta1 = (180/3.141592654)*asin(Value1/maxValue1);
beta2 = (180/3.141592654)*asin(Value2/maxValue2);
beta3 = (180/3.141592654)*asin(Value3/maxValue3);
beta4 = (180/3.141592654)*asin(Value4/maxValue4);
beta5 = (180/3.141592654)*asin(Value5/maxValue5);
beta6 = (180/3.141592654)*asin(Value6/maxValue6);

//prints the values coming from the sensor on the screen 
Serial.print("A1 Reading (mA)\t"); 
Serial.println(Value1,6);
Serial.print("Beta1 (degrees)\t");
Serial.println(beta1,6);
Serial.print("A2 Reading (mA)\t"); 
Serial.println(Value2,6);
Serial.print("Beta2 (degrees)\t");
Serial.println(beta2,6);
Serial.print("A3 Reading (mA)\t"); 
Serial.println(Value3,6);
Serial.print("Beta3 (degrees)\t");
Serial.println(beta3,6);
Serial.print("A4 Reading (mA)\t"); 
Serial.println(Value4,6);
Serial.print("Beta4 (degrees)\t");
Serial.println(beta4,6);
Serial.print("A5 Reading (mA)\t");
Serial.println(Value5,6);
Serial.print("Beta5 (degrees)\t");
Serial.println(beta5,6);
Serial.print("A6 Reading (mA)\t");
Serial.println(Value6,6);
Serial.print("Beta6 (degrees)\t");
Serial.println(beta6,6);

delay(1000);
}
