#ifndef estimator_attitude_h
#define estimator_attitude_h

#include "Arduino.h"

#include "../definitions/parameters.h"
#include "../classes/icm20948.h"

// Attitude estimator class
class AttitudeEstimator {
    public:
        // Constructor
        AttitudeEstimator(int pin_sda, int pin_scl);
        // Initializer
        void init();
        // Estimate step
        void estimate();
        // Rotation quaternion estimations
        float q0, q1, q2, q3;
        // Angular velocity (rad/s) estimations
        float omega_x, omega_y, omega_z;
        // Get acceleration and gyroscope values directly from IMU
        float ax(), ay(), az(), gx(), gy(), gz();

    private:
        // IMU sensor object
        ICM20948 imu;
        // Angular velocity bias calibration
        void calibrate();
        // Predict step
        void predict(float omega_x, float omega_y, float omega_z);
        // Correct step
        void correct(float ax, float ay, float az);
        // Angular velocity (rad/s) bias
        float b_omega_x, b_omega_y, b_omega_z;
};

#endif