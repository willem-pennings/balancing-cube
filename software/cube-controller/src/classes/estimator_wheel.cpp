#include "estimator_wheel.h"

// Constructor
WheelEstimator::WheelEstimator(int pin_speed) : hall(pin_speed) {
    // Set initial angular displacement and angular velocity
    theta_w = 0.0;
    omega_w = 0.0;
}

// Initializer
void WheelEstimator::init() {
    // Initialize and calibrate hall sensor
    hall.init();
}

// Estimate step
void WheelEstimator::estimate(float tau) {
    // Predict step
    predict(tau);

    // Get angular velocity measurement from hall sensor
    hall.read();

    // Correct step
    correct(hall.omega);
}

// Predict step
void WheelEstimator::predict(float tau) {
    // Calculate friction torque
    float sign = (0.0 < omega_w) - (omega_w < 0.0);
    float tau_f = sign * (tau_c + bw * abs(omega_w));

    // Calculate angular acceleration
    omega_w_dot = (1.0 / I_w_xx) * (-tau_f + tau);

    // Predict angular displacement and angular velocity
    theta_w += omega_w * dt + omega_w_dot * dt * dt / 2.0;
    omega_w += omega_w_dot * dt;
}

// Correct step
void WheelEstimator::correct(float omega_w_m) {
    // Correct angular velocity with measurement
    omega_w += ldw * dt * (omega_w_m - omega_w);
}

float WheelEstimator::omega() {
    // Get hall sensor reading
    return hall.omega;
}
