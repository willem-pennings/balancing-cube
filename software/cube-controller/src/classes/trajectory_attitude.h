#ifndef trajectory_attitude_h
#define trajectory_attitude_h

#include "Arduino.h"

#include "../definitions/parameters.h"

// Attitude trajectory class
class AttitudeTrajectory {
    public:
        // Constructor
        AttitudeTrajectory();
        // Initializer
        void init();
        // Generate step
        void generate();
        //
        float qr0, qr1, qr2, qr3;
        //
        float omega_r_x, omega_r_y, omega_r_z;
        //
        float alpha_r_x, alpha_r_y, alpha_r_z;

    private:
        // Microsecond timer
        unsigned long timer;
        //
        float pos, vel, acc, jer, sna, cra;
        //
        bool flag_rot1, flag_res1, flag_rot2, flag_res2;
};

#endif
