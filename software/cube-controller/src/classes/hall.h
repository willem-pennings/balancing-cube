#ifndef hall_h
#define hall_h

#include "Arduino.h"

#include "../definitions/parameters.h"

// Hall class
class Hall {
    public:
        // Class constructor
        Hall(int pin_speed);
        // Initialize and calibrate
        void init();
        // Read angular velocity
        void read();
        // Angular velocity (rad/s)
        float omega;

    private:
        // Input pin
        int pin_speed;
        // Bias (calibration offset)
        float bias;
};

#endif