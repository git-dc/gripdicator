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
#include <Queue.h>
#include "MPU9250.h"
#include <QueueArray.h>
#define SENSOR_LOOP_DURATION 20
#define WINDOW 5
#define TOLERANCE 100

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
QueueArray<double> queue;

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
float cd_coeff[5] = {0.0833, -0.6666, 0, 0.6666, -0.0833};
double passphrase[4] = {1.0, 2.0, 0.5, 0.5};
bool tap_detected = false;
DataQueue<long> tap_interval_q(20);

bool in_range(double val, double target, double tolerance = TOLERANCE) {
  if (val < target + tolerance and val > target - tolerance) {
    return true;
  }
  else {
    return false;
  }
}
void wakeUp() {
  Serial.println("Awake!");
}

double getEnergy(){
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
  }

  return energy_integral;
}

bool check_for_reset() {
  DataQueue<long> temp_q(20);
  DataQueue<long> rhythm_base(4);
  long val = 0;
  long last_val = 0;
  long difference = 0;
  int k = 0;
  bool successes[4][4] = {
    {false, false, false, false},
    {false, false, false, false},
    {false, false, false, false},
    {false, false, false, false},
  };
  while (not tap_interval_q.isEmpty()) {
    val = tap_interval_q.dequeue();
    temp_q.enqueue(val);
    difference = val - last_val;
    last_val = val;
    rhythm_base.enqueue(difference);

    if (in_range(difference, passphrase[k]*rhythm_base.dequeue())) {
      Serial.println("in range");
      successes[0][k] = true;
      k++;
    }

    else {
      Serial.println("not in range, resetting");
      k = 0;
      for (int i = 0; i++; i < 4) {
        successes[0][i] = false;
      }
    }


    //    if (rhythm_base.isFull()) {
    //      rhythm_base.dequeue();
    //    }

  }
  while (not temp_q.isEmpty()) {
    val = temp_q.dequeue();
    tap_interval_q.enqueue(val);
  }
  return true;
}

void accel_calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  double calibration_sum = 0;
  int k = 0;
  long start_time = millis();
  //  Serial.print("Starting accelerometer calibration: ");
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
  //  Serial.print("Accelerometer calibration complete: ");
  //  Serial.print(millis());
  //  Serial.print(". Bias calculated to be: ");
  //  Serial.println(g_bias);
  digitalWrite(LED_BUILTIN, LOW);
}

bool tapDetect(){

  // delete old value
  queue.dequeue();

  // enqueue a new reading
  queue.enqueue(getEnergy());

  // obtain values by dequeing
  // iteratively calculate central difference
  // requeue to restore how the queue looks

  double central_difference = 0;

  for (int i=0; i<WINDOW; ++i){
    double val = queue.dequeue();
    central_difference += (val*cd_coeff[i]);
    queue.enqueue(val);
  }

  central_difference /= interval*0.001;

//  Serial.print("Central Difference:");
//  Serial.println(central_difference);

  if (central_difference >= 15){
    Serial.println("Tapped!");
    Serial.println(central_difference);
    return 1;
  }

  else {
    return 0;
  }
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
  accel_calibration();
  for (int i = 0; i < 5; ++i) {
    queue.enqueue(getEnergy());
  }
  while (not tap_interval_q.isFull()) {
    tap_interval_q.enqueue(0);
  }
}

void loop() {
  getEnergy();
  bool tapFlag = tapDetect();
  if (tap_detected) {
    tap_detected = false;
    tap_interval_q.dequeue();
    tap_interval_q.enqueue(millis());
    check_for_reset();
  }

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
