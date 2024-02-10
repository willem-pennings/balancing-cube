#ifndef parameters_h
#define parameters_h

// System frequencies and periods
const float f = 250;
const float dt = 1 / f;
const unsigned int dt_us = dt * 1e6;

// IMU gyroscope offsets
const float b_gyr_x = 0.74;
const float b_gyr_y = 0.66;
const float b_gyr_z = -0.45;

// IMU accelerator offsets and gains
const float b_acc_x = -6;
const float b_acc_y = -215;
const float b_acc_z = 161;
const float f_acc_x = 1672.7;
const float f_acc_y = 1673.3;
const float f_acc_z = 1691.5;

// Physical parameters
const float pi = 3.14159265359;
const float g = 9.80665;

// Surface parameters
const float b = 0.0;

// Electrical motor properties
const float Ra = 0.942; // Armature (winding) resistance (Ohm)
const float La = 0.363e-3; // Armature (winding) inductance (H)
const float Km = 36e-3; // Torque constant (Nm/A)
const float ia_max = 6; // Maximum current (A)
const float omega_nl = 6250 * pi / 30; // No load speed (rad/s)

// Mechanical motor properties
const float tau_c = 3.4e-3; // Coulomb friction torque (Nm)
const float bw = 1.513e-5; // Rotational viscous friction coefficient (Nms/rad)

// Structure properties
const float l = 0.15; // Structure side length (m)
const float m_s = 1.0; // Structure mass (kg)
const float I_s_xx = 3e-3; // Structure moment of inertia around xyz at center of mass (kgm^2)

// Reaction wheel properties
const float m_w = 0.225; // Reaction wheel mass (kg)
const float I_w_xx = 4.855e-4; // Reaction wheel moment of inertia around x axis at center of mass (kgm^2)
const float I_w_yy = 2.439e-4; // Reaction wheel moment of inertia around yz axis at center of mass (kgm^2)

// Combined properties of structure and reaction wheels
const float m_c = m_s + 3 * m_w; // Cubli total mass (kg)
const float I_c_xx_bar = I_s_xx + 2 * I_w_yy + (m_s + 2.0 * m_w) * l * l / 2.0; // In-plane moment of inertia (kgm^2)
const float I_c_xy_bar = -(m_s + m_w) * l * l / 4.0; // Out-of-plane moment of inertia (kgm^2)

// Auxiliary parameters
const float m_c_bar = m_c - m_w;
const float m_c_bar_g_l = m_c_bar * g * l;

// Estimator gains
const float lds = 1; // How much do you trust the accelerometer compared to the gyroscope? This gain determines that.
const float ldw = 150; // How much do you trust the hall sensor compared to the wheel model? This gain determines that.

// Controller gains. These must be re-tuned if your cube has different dynamics (weights, inertias, dimensions, etc.)
const float kp = 300;
const float kd = 40;
const float kpw = 0.009;
const float kdw = 0.02;

// One out of three of the following initial reference quaternions should be uncommented
// Quaternion reference (Cubli sitting on on corner, corrected for center of mass misalignment)
const float phi_e = 0 * pi / 180.0;
const float qu0 =                    cos(phi_e / 2.0 + acos(sqrt(3.0) / 3.0) / 2.0);
const float qu1 =  sqrt(2.0) / 2.0 * sin(phi_e / 2.0 + acos(sqrt(3.0) / 3.0) / 2.0);
const float qu2 = -sqrt(2.0) / 2.0 * sin(phi_e / 2.0 + acos(sqrt(3.0) / 3.0) / 2.0);
const float qu3 =  0.0;

// Quaternion reference (Cubli sitting on x axis edge, corrected for center of mass misalignment)
// const float phi_e = -3.0 * pi / 180.0;
// const float qu0 = cos(phi_e / 2.0 - pi / 8.0);
// const float qu1 = cos(phi_e / 2.0 + 3.0 * pi / 8.0);
// const float qu2 = 0.0;
// const float qu3 = 0.0;

// Quaternion reference (Cubli sitting on y axis edge, corrected for center of mass misalignment)
// const float phi_e = -3.0 * pi / 180.0;
// const float qu0 = cos(phi_e / 2.0 - pi / 8.0);
// const float qu1 = 0.0;
// const float qu2 = -cos(phi_e / 2.0 + 3.0 * pi / 8.0);
// const float qu3 = 0.0;

// Quaternion stuff
const float qu0_qu0 = qu0 * qu0;
const float qu0_qu1 = qu0 * qu1;
const float qu0_qu2 = qu0 * qu2;
const float qu0_qu3 = qu0 * qu3;
const float qu1_qu1 = qu1 * qu1;
const float qu1_qu2 = qu1 * qu2;
const float qu1_qu3 = qu1 * qu3;
const float qu2_qu2 = qu2 * qu2;
const float qu2_qu3 = qu2 * qu3;
const float qu3_qu3 = qu3 * qu3;

// Minimum and maximum error limits (for control safety)
const float phi_min = 5.0 * pi / 180.0;
const float phi_max = 40.0 * pi / 180.0;

// Minimum jerk trajectory parameters
const float pos_traj = 2.0 * pi; // Trajectory path (rad)
const float t_rest = 10.0; // Rest time (s)
const float t_traj = 20.0; // Trajectory time (s)
const float cra_0 = 720.0 * pos_traj / pow(t_traj, 5);
const float sna_0 = 360.0 * pos_traj / pow(t_traj, 4);
const float jer_0 =  60.0 * pos_traj / pow(t_traj, 3);

#endif
