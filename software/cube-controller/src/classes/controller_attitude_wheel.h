#ifndef controller_attitude_wheel_h
#define controller_attitude_wheel_h

#include "Arduino.h"

#include "../definitions/parameters.h"

// Attitude and wheel controller class
class AttitudeWheelController {
    public:
        // Constructor
        AttitudeWheelController(bool useNonlinear);
        // Control step
        void control(float qr0, float qr1, float qr2, float qr3, float q0, float q1, float q2, float q3, 
            float omega_r_x, float omega_r_y, float omega_r_z, float omega_x, float omega_y, float omega_z, 
            float alpha_r_x, float alpha_r_y, float alpha_r_z, float theta_1, float theta_2, float theta_3, 
            float omega_1, float omega_2, float omega_3);
        // Quaternion error
        float qe0, qe1, qe2, qe3;
        // Torque (Nm)
        float tau_1, tau_2, tau_3;
        // Friction torques
        float tau_f_1, tau_f_2, tau_f_3;

    private:
        // State regulator step
        void state_regulator(float qr0, float qr1, float qr2, float qr3, float q0, float q1, float q2, float q3, 
            float omega_r_x, float omega_r_y, float omega_r_z, float omega_x, float omega_y, float omega_z, 
            float alpha_r_x, float alpha_r_y, float alpha_r_z, float theta_1, float theta_2, float theta_3, 
            float omega_1, float omega_2, float omega_3);
        // Feedback linearization step
        void feedback_linearization(float q0, float q1, float q2, float q3, float omega_x, float omega_y, 
            float omega_z, float omega_1, float omega_2, float omega_3);
        // Linear regulator
        void linear_regulator(float qr0, float qr1, float qr2, float qr3, float q0, float q1, float q2, float q3, 
            float omega_r_x, float omega_r_y, float omega_r_z, float omega_x, float omega_y, float omega_z, 
            float alpha_r_x, float alpha_r_y, float alpha_r_z, float theta_1, float theta_2, float theta_3, 
            float omega_1, float omega_2, float omega_3);
        // Linearized input
        float u_1, u_2, u_3;
        // Controller type
        bool useNonlinear;
};

#endif
