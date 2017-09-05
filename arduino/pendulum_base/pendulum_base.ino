// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

TODO: Rewrite

Notes Andreas Gerken:
- The timing message gives a rough idea of the percentage of time, in which the arduino is active.
  On the Arduino Mega 3ms as imu interval led to ~ 35% activity, 2ms led to 80% activity which could possibly disturb the other loops
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
int motorCommand = 0;

int16_t ax, ay, az, gx, gy, gz;
double ax_m, ay_m, az_m, gx_m, gy_m, gz_m;
double current_raw;
double poti_m, current_m, current_long_m;

double accel_alpha = 0.15;
double gyro_alpha = 0.15;
double poti_alpha = 0;
double current_alpha = 0.15;
double current_long_alpha = 0.001;

#include <ros.h>
#include <ros/time.h>
#include <tiny_msgs/tinyVector.h>
#include <tiny_msgs/tinyIMU.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <Servo.h>

// General variables
#define dim_m 1 // Legs
#define dim_s 8 // Gyro acceleration and rotation, poti, current

#define pinEnable 10
#define pinRev1 7
#define pinRev2 8

// Prototypes
void receiveMotorCommand( const std_msgs::Float32MultiArray& message);
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max);

ros::NodeHandle  nh;

tiny_msgs::tinyIMU imu_msg;
std_msgs::Float32 timing_msg;
std_msgs::Float32 poti_msg;
std_msgs::Float32 current_msg;

ros::Publisher imu_pub("tinyImu", &imu_msg);
ros::Publisher timing_pub("timing", &timing_msg);
ros::Publisher poti_pub("poti", &poti_msg);
ros::Publisher current_pub("current", &current_msg);

ros::Subscriber<std_msgs::Float32MultiArray> subscriber("pendulumMotor", receiveMotorCommand);

uint32_t seq;
unsigned long currentMillis;

unsigned long publishPrevious = 0;
unsigned long spinPrevious = 0;
unsigned long imuPrevious = 0;
unsigned long timingPrevious = 0;

unsigned long publishInterval = 50;
unsigned long spinInterval = 10;
unsigned long imuInterval = 3;
unsigned long timingInterval = 1000;

unsigned long activeTime = 0;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);
  
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(timing_pub);
  nh.advertise(poti_pub);
  nh.advertise(current_pub);
  nh.subscribe(subscriber);
  
  nh.loginfo("ROS Setup finished, starting MPU6050");
  
  accelgyro.initialize();

  // set offsets, the values are measured with the calibration script from https://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/ and are specific to one IMU!!
  accelgyro.setXAccelOffset(-1373);
  accelgyro.setYAccelOffset(-2669);
  accelgyro.setZAccelOffset(1793);
  accelgyro.setXGyroOffset(64);
  accelgyro.setYGyroOffset(10);
  accelgyro.setZGyroOffset(53);

  // MPU 6050 initialization
  nh.loginfo(accelgyro.testConnection() ? "MPU6050 connection successfull" : "MPU6050 connection failed");

  seq = 0;
}

void loop()
{
  currentMillis = millis();

  // check which case should be executed (important first)
  if(currentMillis - publishPrevious >= publishInterval && nh.connected()){
    /*
     * The smoothed imu values are published
     */

    seq ++;
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = 0;
    imu_msg.header.seq = seq;
  
    imu_msg.accel.x = (int16_t)ax_m;
    imu_msg.accel.y = (int16_t)ay_m;
    imu_msg.accel.z = (int16_t)az_m;
    imu_msg.gyro.x = (int16_t)gx_m;
    imu_msg.gyro.y = (int16_t)gy_m;
    imu_msg.gyro.z = (int16_t)gz_m;

    poti_msg.data = poti_m;

    //if (motorCommand > 0){
      current_msg.data = current_m;
    //}else{
    //  current_msg.data = -1. * current_m;
    //}
    
    imu_pub.publish( &imu_msg );
    poti_pub.publish( &poti_msg );
    current_pub.publish( &current_msg );

    // accumulate active time
    activeTime += millis() - currentMillis;
    
    publishPrevious = currentMillis;
  }else if(currentMillis - spinPrevious >= spinInterval){
    /*
     * Ros node spinning
     */
    
    nh.spinOnce();

    // accumulate active time
    activeTime += millis() - currentMillis;

    spinPrevious = currentMillis;
  }else if(currentMillis - imuPrevious >= imuInterval && nh.connected()) {
    /*
     * Measure IMU and average
     */

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    ax_m = (double)ax * accel_alpha + ax_m * (1-accel_alpha);
    ay_m = (double)ay * accel_alpha + ay_m * (1-accel_alpha);
    az_m = (double)az * accel_alpha + az_m * (1-accel_alpha);
    gx_m = (double)gx * accel_alpha + gx_m * (1-gyro_alpha);
    gy_m = (double)gy * accel_alpha + gy_m * (1-gyro_alpha);
    gz_m = (double)gz * accel_alpha + gz_m * (1-gyro_alpha);

    

    poti_m = poti_m * poti_alpha + (analogRead(A0) / 940.) * (1-poti_alpha);
    current_raw = ((analogRead(A1) - 500.) / 50.);
    current_m = current_m * current_alpha +  current_raw * (1-current_alpha);
    
    //#current_m = current_m * current_alpha +  (current_raw - current_long_m) * (1-current_alpha);
    //current_long_m = current_long_m * current_long_alpha + current_raw * (1-current_long_alpha);

    // accumulate active time
    activeTime += millis() - currentMillis;

    imuPrevious = currentMillis;
  }else if(currentMillis - timingPrevious >= timingInterval && nh.connected()){
    /*
     * Send information about the timing activity in period (0-1)
     */
    timing_msg.data = (float)activeTime /timingInterval;
    timing_pub.publish(&timing_msg);


    // reset active time
    activeTime = 0;
    
    timingPrevious = currentMillis;
  }

}

void setMotorSpeed(int mSpeed){
  bool rev = mSpeed > 0;
  digitalWrite(pinRev1, rev);
  digitalWrite(pinRev2, !rev);
  analogWrite(pinEnable,abs(mSpeed));
}


float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
 if( x > in_max) x = in_max;
 if (x < in_min) x = in_min;
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


void receiveMotorCommand( const std_msgs::Float32MultiArray& message){
   motorCommand = message.data[0];
   setMotorSpeed(motorCommand);
}

