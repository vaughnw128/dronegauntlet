// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors. 
// The effect of temperature has not been taken into account so I can't promise that it will work if you 
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.

/* ==========  LICENSE  ==================================
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
 =========================================================
 */

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x68); // <-- use for AD0 high

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float accErrorX, accErrorY;
float gyroErrorX, gyroErrorY, gyroErrorZ;
float accelAngleX, accelAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;
int c = 0;


///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.

  // initialize serial communication
  Serial.begin(19200);

  // initialize device
  accelgyro.initialize();

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer            
  while (Serial.available() && Serial.read()); // empty buffer again

  // start message
  Serial.println("\nMPU6050 Accel Sketch");
  delay(2000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(3000);
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);

  //printcalib();

  // reset offsets
  accelgyro.setXAccelOffset(-387); // -387
  accelgyro.setYAccelOffset(-799); // -799
  accelgyro.setZAccelOffset(2125); // 2125
  accelgyro.setXGyroOffset(17); // 17
  accelgyro.setYGyroOffset(17); // 17
  accelgyro.setZGyroOffset(-16); // -16

  calculate_IMU_error();
  delay(20);

}

///////////////////////////////////   LOOP   ////////////////////////////////////

void loop(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelX = (float)ax / 16384.0;
  accelY = (float)ay / 16384.0;
  accelZ = (float)az / 16384.0;

  accelAngleX = (atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI) - accErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accelAngleY = (atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI) - accErrorY; // AccErrorY ~(-1.58

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  gyroX = ((float) gx / 131.0) - gyroErrorX; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gyroY = ((float) gy / 131.0) - gyroErrorY;
  gyroZ = ((float) gz / 131.0) - gyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + gyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + gyroY * elapsedTime;
  yaw =  yaw + gyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accelAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accelAngleY;
  
  // Print the values on the serial monitor
  Serial.print(yaw);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(roll);

}


///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accelX = (float)ax / 16384.0;
    accelY = (float)ay / 16384.0;
    accelZ = (float)az / 16384.0;
    // Sum all readings
    accErrorX = accErrorX + ((atan((accelY) / sqrt(pow((accelX), 2) + pow((accelZ), 2))) * 180 / PI));
    accErrorY = accErrorY + ((atan(-1 * (accelX) / sqrt(pow((accelY), 2) + pow((accelZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  accErrorX = accErrorX / 200;
  accErrorY = accErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    // Sum all readings
    gyroErrorX = gyroErrorX + (gx / 131.0);
    gyroErrorY = gyroErrorY + (gy / 131.0);
    gyroErrorZ = gyroErrorZ + (gz / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  gyroErrorX = gyroErrorX / 200;
  gyroErrorY = gyroErrorY / 200;
  gyroErrorZ = gyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("accErrorX: ");
  Serial.println(accErrorX);
  Serial.print("accErrorY: ");
  Serial.println(accErrorY);
  Serial.print("gyroErrorX: ");
  Serial.println(gyroErrorX);
  Serial.print("gyroErrorY: ");
  Serial.println(gyroErrorY);
  Serial.print("gyroErrorZ: ");
  Serial.println(gyroErrorZ);
}

void printcalib() {
  if (state==0){
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset); 
    Serial.print("\t");
    Serial.print(ay_offset); 
    Serial.print("\t");
    Serial.print(az_offset); 
    Serial.print("\t");
    Serial.print(gx_offset); 
    Serial.print("\t");
    Serial.print(gy_offset); 
    Serial.print("\t");
    Serial.println(gz_offset); 
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
  }
}

void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}