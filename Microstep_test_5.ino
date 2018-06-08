#include <MatrixMath.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Keyboard.h>
#include "MPU9250.h"
//#include <Matrix.h>


#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging
double time;

MPU9250 myIMU;
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)

Adafruit_StepperMotor *myMotor = AFMS.getStepper(400, 2); // 400 steps, port #2
int commandBit;
double deg = 0;

//////////////////////////////ReadSunSensors_1
#include <math.h>
#include <Keyboard.h>

//ID[rank] returns the sensorID (face)
int ID[6] = {0, 1, 2, 3, 4, 5};
//value[rank] returns the sensor reading
int value[6];
float sun[3];
int sorted[6];
double beta = 0;
double ratio = 0;

// select the input pin for LDR
int sensorPin1 = A8;
int sensorPin2 = A9;
int sensorPin3 = A10;
int sensorPin4 = A11;
int sensorPin5 = A12;
int sensorPin6 = A13;

// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}
/////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600); //sets serial port for communication 4100 BAUD
  //Serial.begin(38400);
  Serial.println("Send '1' to take data");
  AFMS.begin();  // create with the default frequency 1.6KHz

  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  myMotor->setSpeed(10);  // 10 rpm
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  //Serial.begin(38400);


  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);


  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5], 1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);


    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);



    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }
  Serial.println("\npress 1 to take data. Data is in the form:\n");
  Serial.println("Angle gx gy gz mx my mz[(mA)min->max] [face min->max] [sun unit vector x,y,z]\n");
}

void loop() {
  //if (Serial.available() > 0) {
  //commandBit = Serial.read();

  //if (commandBit == '1'){
  //myMotor->step(2, FORWARD, MICROSTEP);
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  //time = millis()/1000; // in seconds
  //delay(10);
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)


  // Print acceleration values in milligs!
  //Serial.print(time);Serial.print("\t");
  //Serial.print(1000*myIMU.ax);Serial.print("\t");
  //Serial.print(1000*myIMU.ay);Serial.print("\t");
  //Serial.print(1000*myIMU.az);Serial.print("\t");

  Serial.print(deg); Serial.print("\t");
  // Print gyro values in degree/sec
  Serial.print(myIMU.gx, 3); Serial.print("\t");
  Serial.print(myIMU.gy, 3); Serial.print("\t");
  Serial.print(myIMU.gz, 3); Serial.print("\t");


  // Print mag values in degree/sec
  Serial.print(myIMU.mx); Serial.print("\t");
  Serial.print(myIMU.my); Serial.print("\t");
  Serial.print(myIMU.mz); Serial.print("\t");

  // read the value from the sensor - these need to be input in the right order
  //0,1 are opposite, 2,3 (y) are opposite, 4,5 (z)
  //THIS MAY NEED TO BE TRANSPOSED
  value[0] = analogRead(sensorPin2);
  value[1] = analogRead(sensorPin4);
  value[2] = analogRead(sensorPin3);
  value[3] = analogRead(sensorPin5);
  value[4] = analogRead(sensorPin1);
  value[5] = analogRead(sensorPin6);

  float offsetValues[3] = {value[1] - value[2], value[3] - value[4], value[5] - value[6]};
  offsetValues *= 1/255;
  float primary_css_normals[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; //css_normals(:,1:2:end);
  sun = {offsetValues[1], offsetValues[2], offsetValues[3]}; //this is just primary_css_normals*offsetValues
  
  /*
    /arbitrary body frame: +x is 1; +y is 2; +z is 0
    //make vector values negative if they come from 3(-x),4(-y),or 5(-z)
    //copy 'value' array into 'sorted'
    memcpy(sorted, value, sizeof(value[0])*6);

    //put sensor values in ascending order in sorted[]
    int sorted_length = sizeof(sorted) / sizeof(sorted[0]);
    // qsort - last parameter is a function pointer to the sort function
    qsort(sorted, sorted_length, sizeof(sorted[0]), sort_desc);
    // sorted is now sorted

    for(int y=0;y<6;y++){
    for(int z = 0; z<6;z++){
      if(sorted[z] == value[y]){
        ID[z] = y;
        break;
      }
    }
    }

    //normal vectors for all 6 sensors:
    //here, adjacent inputs face opposite directions
    // int css_normals[3][6] = {
                  {1,-1,0,0,0,0},
                 {0,0,1,-1,0,0},
                 {0,0,0,0,1,-1} };



    //columns 1, 3, 5 are the primary normal vectors
  
  
  //BLA::Matrix<3,3> primary_css_normals_inv = primary_css_normals.Inverse();
  //int inverse = Matrix<int>
  //Matrix.Multiply((float*)primary_css_normals, (float*)offsetValues, 3, 3, 3, (float*)sun);
  
  int l;
  for (l = 0; l < 2; l++) {
    offsetValues[l] *= 1/255;
  }
  //create unit vectors
    sun[0] = sorted[0]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    sun[1] = sorted[1]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    sun[2] = sorted[2]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    //arbitrary body frame: +x is 1; +y is 2; +z is 0
    //make vector values negative if they come from 3(-x),4(-y),or 5(-z) on which face they come from
    //sun vector is the current through each sensor, divided by magnitude, and signed for the appropriate direction based on axes choice
    THIS IS WRONG
    for(int m=3; m<=5; m++){
    if(ID[m] == 1){
      sun[0] = sorted[m]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    }
    else if(ID[m] == 3){
      sun[0] = -sorted[m]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    }
    else if(ID[m] == 2){
      sun[1] = sorted[m]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    }
    else if(ID[m] == 4){
      sun[1] = -sorted[m]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    }
    else if(ID[m] == 0){
      sun[2] = sorted[m]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    }
    else if(ID[m] == 5){
      sun[2] = -sorted[m]/sqrt(sq(sorted[5])+sq(sorted[4])+sq(sorted[3]));
    }
    }
  */
  //Serial.print(n);
  //Serial.print("\n");
  Serial.print(sorted[5]); Serial.print("\t");
  Serial.print(sorted[4]); Serial.print("\t");
  Serial.print(sorted[3]); Serial.print("\t");
  Serial.print(sorted[2]); Serial.print("\t");
  Serial.print(sorted[1]); Serial.print("\t");
  Serial.print(sorted[0]); Serial.print("\t");
  Serial.print(ID[5]); Serial.print("\t");
  Serial.print(ID[4]); Serial.print("\t");
  Serial.print(ID[3]); Serial.print("\t");
  Serial.print(ID[2]); Serial.print("\t");
  Serial.print(ID[1]); Serial.print("\t");
  Serial.print(ID[0]); Serial.print("\t");
  //Serial.print("Sun vector in body frame:[");
  Serial.print(offsetValues[0]); Serial.print("\t");
  Serial.print(offsetValues[1]); Serial.print("\t");
  Serial.print(offsetValues[2]); Serial.print("\n");
  //}

  deg = deg + 1.8;
  myMotor->step(2, FORWARD, MICROSTEP);
  delay(1000);
  //}
  if (deg > 360) {
    return;
  }
}

