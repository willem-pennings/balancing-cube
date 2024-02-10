#include "estimator_attitude.h"

// Constructor
AttitudeEstimator::AttitudeEstimator(int pin_sda, int pin_scl) : imu(pin_sda, pin_scl) {
    // Set initial rotation quaternion
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;

    // Set initial angular velocity
    omega_x = 0.0;
    omega_y = 0.0;
    omega_z = 0.0;

    // Set initial angular velocity bias
    b_omega_x = 0.0;
    b_omega_y = 0.0;
    b_omega_z = 0.0;
}

// Initializer
void AttitudeEstimator::init() {
    // Initialize IMU sensor object
    imu.init();

    // Angular velocity bias calibration
    calibrate();
}

// Angular velocity bias calibration 
void AttitudeEstimator::calibrate() {
    // Calculate angular velocity bias by averaging one thousand samples or 5 seconds worth of gyroscope data
    // This is done on top of the one-time calibration that was done manually in advance
    for(int i = 0; i < 1000; i++) {
        // Read sensor values
        imu.read();

        // Add 1/f-th part of the current reading to the bias
        b_omega_x += imu.gx / 1000;
        b_omega_y += imu.gy / 1000;
        b_omega_z += imu.gz / 1000;

        delay(5);
    }
}

// Estimate step
void AttitudeEstimator::estimate() {
    // Read values from IMU
    imu.read();

    // Get angular velocity from IMU gyroscope data and apply second offset correction
    omega_x = imu.gx - b_omega_x;
    omega_y = imu.gy - b_omega_y;
    omega_z = imu.gz - b_omega_z;

    // Predict step
    predict(omega_x, omega_y, omega_z);

    // Get linear acceleration from IMU accelerometer data
    float ax = imu.ax;
    float ay = imu.ay;
    float az = imu.az;

    // Normalize linear acceleration
    float a_norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= a_norm;
    ay /= a_norm;
    az /= a_norm;

    // Correct step
    correct(ax, ay, az);

    // Normalize rotation quaternion
    float q_norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= q_norm;
    q1 /= q_norm;
    q2 /= q_norm;
    q3 /= q_norm;
}

// Estimate step
void AttitudeEstimator::predict(float omega_x, float omega_y, float omega_z) {   
    // Predict rotation quaternion time derivative
    float q0_dot = 0.5 * (-q1 * omega_x - q2 * omega_y - q3 * omega_z);
    float q1_dot = 0.5 * ( q0 * omega_x - q3 * omega_y + q2 * omega_z);
    float q2_dot = 0.5 * ( q3 * omega_x + q0 * omega_y - q1 * omega_z);
    float q3_dot = 0.5 * (-q2 * omega_x + q0 * omega_z + q1 * omega_y);

    // Predict rotation quaternion
    q0 += q0_dot * dt;
    q1 += q1_dot * dt;
    q2 += q2_dot * dt;
    q3 += q3_dot * dt;
}

// Correct step
void AttitudeEstimator::correct(float ax, float ay, float az) {    
    // Calculate rotation quaternion measurement
    float qm0 =  ax * q2 - ay * q1 - az * q0;
    float qm1 = -ax * q3 - ay * q0 + az * q1;
    float qm2 =  ax * q0 - ay * q3 + az * q2;
    float qm3 = -ax * q1 - ay * q2 - az * q3;

    // Calculate rotation quaternion error
    float qe0 = q0 * qm0 + q1 * qm1 + q2 * qm2 + q3 * qm3;
    float qe1 = q0 * qm1 - q1 * qm0 - q2 * qm3 + q3 * qm2;
    float qe2 = q0 * qm2 + q1 * qm3 - q2 * qm0 - q3 * qm1;
    float qe3 = q0 * qm3 - q1 * qm2 + q2 * qm1 - q3 * qm0;

    // Calculate rotation Gibbs-vector error
    float se1 = qe1 / qe0;
    float se2 = qe2 / qe0;
    float se3 = qe3 / qe0;

    // Calculate rotation quaternion error time derivative
    float qe0_dot = -q1 * se1 - q2 * se2 - q3 * se3;
    float qe1_dot =  q0 * se1 - q3 * se2 + q2 * se3;
    float qe2_dot =  q3 * se1 + q0 * se2 - q1 * se3;
    float qe3_dot = -q2 * se1 + q1 * se2 + q0 * se3;

    // Correct rotation quaternion
    q0 += lds * dt * qe0_dot;
    q1 += lds * dt * qe1_dot;
    q2 += lds * dt * qe2_dot;
    q3 += lds * dt * qe3_dot;
}

float AttitudeEstimator::ax() {
    return imu.ax;
}

float AttitudeEstimator::ay() {
    return imu.ay;
}

float AttitudeEstimator::az() {
    return imu.az;
}

float AttitudeEstimator::gx() {
    return imu.gx;
}

float AttitudeEstimator::gy() {
    return imu.gy;
}

float AttitudeEstimator::gz() {
    return imu.gz;
}
