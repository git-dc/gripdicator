/*
  Basic_I2C.ino
  Brian R Taylor
  brian.taylor@bolderflight.com

  Copyright (c) 2017 Bolder Flight Systems

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
  and associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute,
  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or
  substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#define SENSOR_LOOP_DURATION 20

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
double ax;  // accelerometer reading in x axis
double ay;  //        --//--         in y axis
double az;  //        --//--         in z axis
double ar;  //        --//--         resultant vector magnitude
double ar2; //        --//--         resultant vector magnitude squared
double energy_integral; // integral of the energy of the resultant acceleration signal
const double g = 9.806 - 0.5 * (9.832 - 9.780) * cos(2 * 24.4539 * PI / 180); // precise value of g for Abu Dhabi
//double g_bias = - 0.2925; //no accel range setting
//double g_bias = - 0.487; //2g accel range setting
double g_bias; //use with adaptive calibration
double prev_val = 0;
double energy_b_derivative = 0;
long interval = SENSOR_LOOP_DURATION;
long prev_loop = millis() - interval;
long cur_loop = millis();


void wakeUp() {
  Serial.println("Awake!");
}

void calibration() {
  double calibration_sum = 0;
  int k = 0;
  long start_time = millis();
//  Serial.print("Starting calibration: ");
//  Serial.println(millis());

  while (millis() - start_time < 1000) {
    cur_loop = millis();
    if (cur_loop - prev_loop >= 10) {
      prev_loop = cur_loop;
      IMU.readSensor();
      ax = IMU.getAccelX_mss();
      ay = IMU.getAccelY_mss();
      az = IMU.getAccelZ_mss();
      ar = sqrt(ax * ax + ay * ay  + az * az);
      calibration_sum += ar;
      k++;
    }
  }
  g_bias = calibration_sum / k - g;
//  Serial.print("Calibration complete: ");
//  Serial.print(millis());
//  Serial.print(". Bias calculated to be: ");
//  Serial.println(g_bias);
}

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful.");
    Serial.println("Check IMU wiring or try cycling power.");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  //  // setting DLPF bandwidth to 20 Hz
  //  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  //  // setting SRD to 19 for a 50 Hz update rate
  //  IMU.setSrd(19);
  //  // enabling wake on motion low power mode with a threshold of 400 mg and
  //  // an accelerometer data rate of 15.63 Hz.
  //  IMU.enableWakeOnMotion(400,MPU9250::LP_ACCEL_ODR_15_63HZ);
  ////   attaching the interrupt to microcontroller pin 1
  //  pinMode(1, INPUT);
  //  attachInterrupt(1, wakeUp, RISING);

  calibration();
}

void loop() {
  cur_loop = millis();
  if (cur_loop - prev_loop >= interval)  {
    prev_loop = cur_loop;
    IMU.readSensor();
    ax = IMU.getAccelX_mss();
    ay = IMU.getAccelY_mss();
    az = IMU.getAccelZ_mss();
    ar = sqrt(ax * ax + ay * ay  + az * az) - (g + g_bias);
    ar2 = pow(ar, 2);
    energy_integral += ar2;
    energy_b_derivative = ar2 - prev_val;
    prev_val = ar2;
    Serial.println(energy_integral, 6);
  }
}
