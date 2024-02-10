#ifndef icm20948_h
#define icm20948_h

#include "Arduino.h"
#include "Wire.h"
#include "ICM20948_WE.h"

#include "../definitions/parameters.h"

// ICM20948 class
class ICM20948 {
    public:
        // Class constructor
        ICM20948(int pin_sda, int pin_scl);
        // Initialize sensor
        void init();
        // Read sensor data
        void read();
        // Gyroscope data in x, y and z axis (rad/s)
        float gx, gy, gz;
        // Accelerometer data x, y and z axis (m/s^2)
        float ax, ay, az;
        
    private:
        // i2c bus
        TwoWire wirePort;
        // i2c pins
        int pin_sda, pin_scl;
        // IMU object
        ICM20948_WE imu;
        // Measurement stores
        xyzFloat acc, gyr;
};

#endif
