#include "controller_attitude_wheel.h"

// Constructor
AttitudeWheelController::AttitudeWheelController(bool useNonlinear) : useNonlinear(useNonlinear) {
    // Set initial quaternion error
    qe0 = 0.0;
    qe1 = 0.0;
    qe2 = 0.0;
    qe3 = 0.0;

    // Set initial torque
    tau_1 = 0.0;
    tau_2 = 0.0;
    tau_3 = 0.0;

    // Set initial linearized input
    u_1 = 0.0;
    u_2 = 0.0;
    u_3 = 0.0;
}

// Control step
void AttitudeWheelController::control(float qr0, float qr1, float qr2, float qr3, float q0, float q1, float q2, 
        float q3, float omega_r_x, float omega_r_y, float omega_r_z, float omega_x, float omega_y, float omega_z, 
        float alpha_r_x, float alpha_r_y, float alpha_r_z, float theta_1, float theta_2, float theta_3, float omega_1,
        float omega_2, float omega_3) {
    if(useNonlinear) {
        state_regulator(qr0, qr1, qr2, qr3, q0, q1, q2, q3, omega_r_x, omega_r_y, omega_r_z, omega_x, omega_y, omega_z,
            alpha_r_x, alpha_r_y, alpha_r_z, theta_1, theta_2, theta_3, omega_1, omega_2, omega_3);
        feedback_linearization(q0, q1, q2, q3, omega_x, omega_y, omega_z, omega_1, omega_2, omega_3);
    } else {
        linear_regulator(qr0, qr1, qr2, qr3, q0, q1, q2, q3, omega_r_x, omega_r_y, omega_r_z, omega_x, omega_y, omega_z, 
            alpha_r_x, alpha_r_y, alpha_r_z, theta_1, theta_2, theta_3, omega_1, omega_2, omega_3);
    }
}

// State regulator step
void AttitudeWheelController::state_regulator(float qr0, float qr1, float qr2, float qr3, float q0, float q1, float q2,
        float q3, float omega_r_x, float omega_r_y, float omega_r_z, float omega_x, float omega_y, float omega_z, 
        float alpha_r_x, float alpha_r_y, float alpha_r_z, float theta_1, float theta_2, float theta_3, float omega_1,
        float omega_2, float omega_3) {
    // Calculate rotation quaternion error
    qe0 = q0 * qr0 + q1 * qr1 + q2 * qr2 + q3 * qr3;
    qe1 = q0 * qr1 - q1 * qr0 - q2 * qr3 + q3 * qr2;
    qe2 = q0 * qr2 - q2 * qr0 + q1 * qr3 - q3 * qr1;
    qe3 = q0 * qr3 - q1 * qr2 + q2 * qr1 - q3 * qr0;

    // Normalize rotation quaternion error
    float qe_norm = sqrt(qe0 * qe0 + qe1 * qe1 + qe2 * qe2 + qe3 * qe3);
    qe0 /= qe_norm;
    qe1 /= qe_norm;
    qe2 /= qe_norm;
    qe3 /= qe_norm;

    // Auxiliary variables to avoid computing the same term multiple times
    float qe0qe1 = qe0 * qe1;
    float qe0qe2 = qe0 * qe2;
    float qe0qe3 = qe0 * qe3;
    float qe1qe1 = qe1 * qe1;
    float qe1qe2 = qe1 * qe2;
    float qe1qe3 = qe1 * qe3;
    float qe2qe2 = qe2 * qe2;
    float qe2qe3 = qe2 * qe3;
    float qe3qe3 = qe3 * qe3;

    // Calculate angular velocity error
    float omega_e_x = omega_r_x + 2.0 * (omega_r_x * (-qe2qe2 - qe3qe3) + omega_r_y * (-qe0qe3 + qe1qe2) + omega_r_z * 
        ( qe0qe2 + qe1qe3)) - omega_x;
    float omega_e_y = omega_r_y + 2.0 * (omega_r_x * ( qe0qe3 + qe1qe2) + omega_r_y * (-qe1qe1 - qe3qe3) + omega_r_z * 
        (-qe0qe1 + qe2qe3)) - omega_y;
    float omega_e_z = omega_r_z + 2.0 * (omega_r_x * (-qe0qe2 + qe1qe3) + omega_r_y * ( qe0qe1 + qe2qe3) + omega_r_z * 
        (-qe1qe1 - qe2qe2)) - omega_z;

    // Auxiliary variable to avoid computing the same term multiple times
    float _2_kp_omega_e_omega_e_4 = 2.0 * (kp - (omega_e_x * omega_e_x + omega_e_y * omega_e_y + omega_e_z * omega_e_z)
        / 4.0);

    // Attitude feedback
    u_1 = _2_kp_omega_e_omega_e_4 * qe1 / qe0 + kd * omega_e_x;
    u_2 = _2_kp_omega_e_omega_e_4 * qe2 / qe0 + kd * omega_e_y;
    u_3 = _2_kp_omega_e_omega_e_4 * qe3 / qe0 + kd * omega_e_z;

    // Attitude feedforward
    u_1 += omega_e_y * omega_z - omega_e_z * omega_y + alpha_r_x + 2.0 * (alpha_r_x *(-qe2qe2 - qe3qe3) + alpha_r_y * 
        (-qe0qe3 + qe1qe2) + alpha_r_z * ( qe0qe2 + qe1qe3));
    u_2 += omega_e_z * omega_x - omega_e_x * omega_z + alpha_r_y + 2.0 * (alpha_r_x *( qe0qe3 + qe1qe2) + alpha_r_y * 
        (-qe1qe1 - qe3qe3) + alpha_r_z * (-qe0qe1 + qe2qe3));
    u_3 += omega_e_x * omega_y - omega_e_y * omega_x + alpha_r_z + 2.0 * (alpha_r_x *(-qe0qe2 + qe1qe3) + alpha_r_y * 
        ( qe0qe1 + qe2qe3) + alpha_r_z * (-qe1qe1 - qe2qe2));

    // Wheel feedback
    u_1 += -kpw * theta_1 - kdw * omega_1;
    u_2 += -kpw * theta_2 - kdw * omega_2;
    u_3 += -kpw * theta_3 - kdw * omega_3;
}

// Feedback linearization step
void AttitudeWheelController::feedback_linearization(float q0, float q1, float q2, float q3, float omega_x, 
        float omega_y, float omega_z, float omega_1, float omega_2, float omega_3) {
    // Calculate friction torque
    float sign_1 = (0.0 < omega_1) - (omega_1 < 0.0);
    float sign_2 = (0.0 < omega_2) - (omega_2 < 0.0);
    float sign_3 = (0.0 < omega_3) - (omega_3 < 0.0);
    tau_f_1 = sign_1 * (tau_c + bw * abs(omega_1));
    tau_f_2 = sign_2 * (tau_c + bw * abs(omega_2));
    tau_f_3 = sign_3 * (tau_c + bw * abs(omega_3));

    // Auxiliary variable to avoid computing the same term multiple times
    float omega_x_omega_y_omega_z = omega_x + omega_y + omega_z;

    // Feedback linearization
    tau_1 = -I_c_xy_bar * (omega_y - omega_z) * omega_x_omega_y_omega_z - I_w_xx * (omega_3 * omega_y - omega_2 * 
        omega_z) + m_c_bar_g_l * (0.5 - q0 * q0 + q0 * q1 - q3 * q3 + q2 * q3) + tau_f_1 - I_c_xx_bar * u_1 - 
        I_c_xy_bar * (u_2 + u_3);
    tau_2 = -I_c_xy_bar * (omega_z - omega_x) * omega_x_omega_y_omega_z - I_w_xx * (omega_1 * omega_z - omega_3 * 
        omega_x) + m_c_bar_g_l * (0.5 + q0 * q2 - q1 * q1 - q1 * q3 - q2 * q2) + tau_f_2 - I_c_xx_bar * u_2 - 
        I_c_xy_bar * (u_1 + u_3);
    tau_3 = -I_c_xy_bar * (omega_x - omega_y) * omega_x_omega_y_omega_z - I_w_xx * (omega_2 * omega_x - omega_1 * 
        omega_y) - m_c_bar_g_l * (      q0 * q1 + q0 * q2 - q1 * q3 + q2 * q3) + tau_f_3 - I_c_xx_bar * u_3 - 
        I_c_xy_bar * (u_1 + u_2);
}

void AttitudeWheelController::linear_regulator(float qr0, float qr1, float qr2, float qr3, float q0, float q1, 
    float q2, float q3, float omega_r_x, float omega_r_y, float omega_r_z, float omega_x, float omega_y, float omega_z, 
        float alpha_r_x, float alpha_r_y, float alpha_r_z, float theta_1, float theta_2, float theta_3, float omega_1, 
        float omega_2, float omega_3) {
    // Calculate rotation quaternion error
    qe0 = q0 * qr0 + q1 * qr1 + q2 * qr2 + q3 * qr3;
    qe1 = q0 * qr1 - q1 * qr0 - q2 * qr3 + q3 * qr2;
    qe2 = q0 * qr2 - q2 * qr0 + q1 * qr3 - q3 * qr1;
    qe3 = q0 * qr3 - q1 * qr2 + q2 * qr1 - q3 * qr0;

    // Normalize rotation quaternion error
    float qe_norm = sqrt(qe0 * qe0 + qe1 * qe1 + qe2 * qe2 + qe3 * qe3);
    qe0 /= qe_norm;
    qe1 /= qe_norm;
    qe2 /= qe_norm;
    qe3 /= qe_norm;

    // Calculate angle error
    float theta_e_x = 2 * asin(qe1);
    float theta_e_y = 2 * asin(qe2);
    float theta_e_z = 2 * asin(qe3);

    // Calculate angular velocity error
    float omega_e_x = omega_r_x - omega_x;
    float omega_e_y = omega_r_y - omega_y;
    float omega_e_z = omega_r_z - omega_z;

    // Attitude feedback (basically a PD controller)
    u_1 = kp * theta_e_x + kd * omega_e_x;
    u_2 = kp * theta_e_y + kd * omega_e_y;
    u_3 = kp * theta_e_z + kd * omega_e_z;

    // Attitude feedforward
    u_1 += alpha_r_x;
    u_2 += alpha_r_y;
    u_3 += alpha_r_z;

    // Wheel feedback
    u_1 += -kpw * theta_1 - kdw * omega_1;
    u_2 += -kpw * theta_2 - kdw * omega_2;
    u_3 += -kpw * theta_3 - kdw * omega_3;

    // Convert inputs into torques
    tau_1 = -I_c_xx_bar * u_1 - I_c_xy_bar * (u_2 + u_3);
    tau_2 = -I_c_xx_bar * u_2 - I_c_xy_bar * (u_1 + u_3);
    tau_3 = -I_c_xx_bar * u_3 - I_c_xy_bar * (u_1 + u_2);
}
