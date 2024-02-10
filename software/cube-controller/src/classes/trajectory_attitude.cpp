#include "trajectory_attitude.h"

// Constructor
AttitudeTrajectory::AttitudeTrajectory() {
    // Initial reference quaternion
    qr0 = qu0;
    qr1 = qu1;
    qr2 = qu2;
    qr3 = qu3;

    // Reference angular velocities for cube
    omega_r_x = 0.0;
    omega_r_y = 0.0;
    omega_r_z = 0.0;

    // Reference angular accelerations for cube
    alpha_r_x = 0.0;
    alpha_r_y = 0.0;
    alpha_r_z = 0.0;

    // Algorithm flags
    flag_rot1 = true;
    flag_res1 = true;
    flag_rot2 = true;
    flag_res2 = true;

    // Initialization of position and its derivatives
    pos = 0.0;
    vel = 0.0;
    acc = 0.0;
    jer = 0.0;
    sna = 0.0;
    cra = 0.0;
}

// Initializer
void AttitudeTrajectory::init() {
    // Start timer
    timer = micros();
}

// Generate step
void AttitudeTrajectory::generate() {
    // Get current time in seconds since start of timer
    float t = (micros() - timer) / 1e6;
    
    // Minimum jerk trajectory algorithm
    if((t >= t_rest / 2.0) && flag_rot1) {
        flag_rot1 = false;
        jer =  jer_0;
        sna = -sna_0;
        cra =  cra_0;
    }

    if((t >= t_traj + t_rest / 2.0) && flag_res1) {
        flag_res1 = false;
        pos = pos_traj;
        vel = 0.0;
        acc = 0.0;
        jer = 0.0;
        sna = 0.0;
        cra = 0.0;
    }

    if((t >= t_traj + 3.0 * t_rest / 2.0) && flag_rot2) {
        flag_rot2 = false;
        jer = -jer_0;
        sna =  sna_0;
        cra = -cra_0;
    }

    if((t >= 2.0 * t_traj + 3.0 * t_rest / 2.0) && flag_res2) {
        flag_res2 = false;
        pos = 0.0;
        vel = 0.0;
        acc = 0.0;
        jer = 0.0;
        sna = 0.0;
        cra = 0.0;
    }

    if(t >= 2.0 * t_traj + 2.0 * t_rest) {
        timer = micros();
        flag_rot1 = true;
        flag_res1 = true;
        flag_rot2 = true;
        flag_res2 = true;
    }

    pos += vel * dt + acc * pow(dt, 2) / 2.0 + jer * pow(dt, 3) / 6.0 + sna * pow(dt, 4) / 24.0 + cra * pow(dt, 5) / 120.0;
    vel += acc * dt + jer * pow(dt, 2) / 2.0 + sna * pow(dt, 3) / 6.0 + cra * pow(dt, 4) / 24.0;
    acc += jer * dt + sna * pow(dt, 2) / 2.0 + cra * pow(dt, 3) / 6.0;
    jer += sna * dt + cra * pow(dt, 2) / 2.0;
    sna += cra * dt;
 
    // Euler angles and its time derivatives
    float phi = pos;
    float phi_dot = vel;
    float phi_ddot = acc;
    float theta = 0.0;
    float theta_dot = 0.0;
    float theta_ddot = 0.0;
    float psi = 0.0;
    float psi_dot = 0.0;
    float psi_ddot = 0.0;

    // Auxiliary varaibles to avoid double arithmetic
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);
    float sin_psi = sin(psi);
    float cos_psi = cos(psi);
    float sin_phi_2 = sin(phi / 2.0);
    float cos_phi_2 = cos(phi / 2.0);
    float sin_theta_2 = sin(theta / 2.0);
    float cos_theta_2 = cos(theta / 2.0);
    float sin_psi_2 = sin(psi / 2.0);
    float cos_psi_2 = cos(psi / 2.0);

    // Calculate orientation quaternion in terms of Euler angles
    float q0 = cos_phi_2 * cos_psi_2 * cos_theta_2 - cos_theta_2 * sin_phi_2 * sin_psi_2;
    float q1 = cos_psi_2 * sin_phi_2 * sin_theta_2 - cos_phi_2 * sin_psi_2 * sin_theta_2;
    float q2 = cos_phi_2 * cos_psi_2 * sin_theta_2 + sin_phi_2 * sin_psi_2 * sin_theta_2;
    float q3 = cos_phi_2 * cos_theta_2 * sin_psi_2 + cos_psi_2 * cos_theta_2 * sin_phi_2;

    // Calculate angular velocity in terms of Euler angles and its derivatives
    float omega_x = sin_phi * theta_dot - cos_phi * sin_theta * psi_dot;
    float omega_y = cos_phi * theta_dot + sin_phi * sin_theta * psi_dot;
    float omega_z = phi_dot + cos_theta * psi_dot;

    // Calculate angular acceleration in terms of Euler angles and its time derivatives
    float alpha_x = cos_phi * phi_dot * theta_dot + sin_phi * theta_ddot - sin_phi * phi_dot * sin_theta * psi_dot - 
        cos_phi * cos_theta * theta_dot * psi_dot - cos_phi * sin_theta * psi_ddot;
    float alpha_y = -sin_phi * phi_dot * theta_dot + cos_phi * theta_ddot + cos_phi * phi_dot * sin_theta * psi_dot + 
        sin_phi * cos_theta * theta_dot * psi_dot + sin_phi * sin_theta * psi_ddot;
    float alpha_z = phi_ddot - sin_theta * theta_dot * psi_dot + cos_theta * psi_ddot;  

    // Calculate orientation quaternion reference with Cubli in vertex
    qr0 = q0 * qu0 - q1 * qu1 - q2 * qu2 - q3 * qu3;
    qr1 = q0 * qu1 + q1 * qu0 + q2 * qu3 - q3 * qu2;
    qr2 = q0 * qu2 - q1 * qu3 + q2 * qu0 + q3 * qu1;
    qr3 = q0 * qu3 + q1 * qu2 - q2 * qu1 + q3 * qu0;

    // Calculate angular velocity reference with Cubli in vertex
    omega_r_x = omega_x + 2.0 * (omega_x * (-qu2_qu2 - qu3_qu3) + omega_y * ( qu0_qu3 + qu1_qu2) + omega_z * 
        (-qu0_qu2 + qu1_qu3));
    omega_r_y = omega_y + 2.0 * (omega_x * (-qu0_qu3 + qu1_qu2) + omega_y * (-qu1_qu1 - qu3_qu3) + omega_z * 
        ( qu0_qu1 + qu2_qu3));
    omega_r_z = omega_z + 2.0 * (omega_x * ( qu0_qu2 + qu1_qu3) + omega_y * (-qu0_qu1 + qu2_qu3) + omega_z * 
        (-qu1_qu1 - qu2_qu2));

    // Calculate angular acceleration reference with Cubli in vertex
    alpha_r_x = alpha_x + 2.0 * (alpha_x * (-qu2_qu2 - qu3_qu3) + alpha_y * ( qu0_qu3 + qu1_qu2) + alpha_z * 
        (-qu0_qu2 + qu1_qu3));
    alpha_r_y = alpha_y + 2.0 * (alpha_x * (-qu0_qu3 + qu1_qu2) + alpha_y * (-qu1_qu1 - qu3_qu3) + alpha_z * 
        ( qu0_qu1 + qu2_qu3));
    alpha_r_z = alpha_z + 2.0 * (alpha_x * ( qu0_qu2 + qu1_qu3) + alpha_y * (-qu0_qu1 + qu2_qu3) + alpha_z * 
        (-qu1_qu1 - qu2_qu2));
}
