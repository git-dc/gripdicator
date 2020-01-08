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
#define SENSOR_LOOP_DURATION 5

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
double ax;
double ay;
double az;
double ar;
double ar2;
double integral;
double g = 9.806 - 0.5 * (9.832 - 9.780) * cos(2 * 24.4539 * PI / 180);
double g_fudge = - 0.2925;
double prev_val = 0;
double bderivative = 0;
long interval = SENSOR_LOOP_DURATION;
long prev_loop = millis();
long cur_loop = millis();

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

}

void loop() {
  cur_loop = millis();
  if (cur_loop - prev_loop >= interval)  {
    prev_loop = cur_loop;
    IMU.readSensor();
    ax = IMU.getAccelX_mss();
    ay = IMU.getAccelY_mss();
    az = IMU.getAccelZ_mss();

    ar2 = ax * ax + ay * ay  + az * az;
    ar = sqrt(ar2) - (g + g_fudge);
    ar2 = pow(ar, 2);
    integral += ar2 ;
    bderivative = ar2 - prev_val;
    prev_val = ar2;
    Serial.println(integral, 6);
  }
}
