// This program can be used to get the offsets and scaling factors for the gyro and accelerometer. This cannot be done
// automatically. The cube must be placed in six different orientations, corresponding to each axis facing up and down.
// Calibration offsets should initially be set to zero. Use the methodology as described in Analog Devices AN-1057,
// which may be found here: https://www.analog.com/en/app-notes/an-1057.html. In particular, equations 17 and 18.

#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68

TwoWire wirePort = TwoWire(0);
ICM20948_WE myIMU = ICM20948_WE(&wirePort);

void setup() {
  // Set up serial
  Serial.begin(921600);

  // Set up I2C on pins 47 (SDA) and 21 (SCL) at 400 kHz
  wirePort.begin(47, 21, 400000);
  
  // Initialize IMU
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }
  
  // Set accelerometer and gyroscope ranges
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_2000);
  
  // Set digital low pass filter setting such that noise is minimized (lowest cut-off frequency)
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setGyrDLPF(ICM20948_DLPF_6); 
  
  // Disable sample rate divider
  myIMU.setAccSampleRateDivider(0);
  myIMU.setGyrSampleRateDivider(0);
}

unsigned long timer;
xyzFloat acc, gyr;
float ax, ay, az, gx, gy, gz;

// Calibration offsets
const float b_gyr_x = 0.74;
const float b_gyr_y = 0.66;
const float b_gyr_z = -0.45;

// IMU accelerator offsets and gain
const float b_acc_x = -6;
const float b_acc_y = -215;
const float b_acc_z = 161;
const float f_acc_x = 1672.7;
const float f_acc_y = 1673.3;
const float f_acc_z = 1691.5;

void loop() {
  // Start a timer
  timer = millis();

  // Measure 1000 samples and average them
  for(int i = 0; i < 1000; i++) {
    myIMU.readSensor();
    acc = myIMU.getAccRawValues();
    gyr = myIMU.getGyrValues();

    ax += acc.x / 1000;
    ay += acc.y / 1000;
    az += acc.z / 1000;
    gx += gyr.x / 1000;
    gy += gyr.y / 1000;
    gz += gyr.z / 1000;

    // Print raw and calibrated values in last cycle
    if(i == 999) {
      Serial.print("Raw: Acc: ");
      Serial.print(ax);
      Serial.print(", ");
      Serial.print(ay);
      Serial.print(", ");
      Serial.print(az);
      Serial.print(", ");
      Serial.print("gyr: ");
      Serial.print(gx);
      Serial.print(", ");
      Serial.print(gy);
      Serial.print(", ");
      Serial.println(gz);

  	  // Equation 19 in AD AN-1057 is used here
      Serial.print("Calibrated: Acc: ");
      Serial.print((ax - b_acc_x) / f_acc_x);
      Serial.print(", ");
      Serial.print((ay - b_acc_y) / f_acc_y);
      Serial.print(", ");
      Serial.print((az - b_acc_z) / f_acc_z);
      Serial.print(", ");
      Serial.print("gyr: ");
      Serial.print(gx - b_gyr_x);
      Serial.print(", ");
      Serial.print(gy - b_gyr_y);
      Serial.print(", ");
      Serial.println(gz - b_gyr_z);
    }
  }

  // Print how long it took to get 1000 measurements
  Serial.print("Took "); Serial.print(millis() - timer); Serial.println(" ms.");

  // Reset averages
  ax = 0;
  ay = 0;
  az = 0;
  gx = 0;
  gy = 0;
  gz = 0;
}
