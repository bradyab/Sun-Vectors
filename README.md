Library for testing cubesat attitude determination algorithms using a Sparkfun MPU-9250 IMU and 6 sun sensors. Prototypes were created with Arduino, Monte Carlo testing performed in Matlab. Based off Kris Winer's repository: https://github.com/kriswiner/MPU9250


ReadSunSensors.ino is an Arduino program that reads and prints each of the 9-axis IMU values at a specified rate.

Microstep_test_5.ino is an Arduino program that uses the motorshield to turn a 5V stepper motor at specified increments and prints IMU and sun sensor data.

MPU_9250_code_full.ino is an Arduino program taken from Chris Winer's GItHub library, and was referenced to create ReadSunSensors.ino.

SunSensorCheck.m is a Matlab monte carlo simulation program given to us by TRICEPT, which prompts you to enter your sun vector algorithm and plugs in random sun sensor values and evaluates the error involved in calculating a sun vector.

Useful tutorial: https://kingtidesailing.blogspot.com/2015/09/how-to-setup-mpu-9150-9-axis.html
