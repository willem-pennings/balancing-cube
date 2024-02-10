#ifndef motor_h
#define motor_h

#include "Arduino.h"

#include "../definitions/parameters.h"

// Motor class
class Motor {
    public:
        // Class constructor
        Motor(int pin_enable, int pin_current);
        // Initialize
        void init();
        // Set current (A)
        void set_current(float ia);
        // Set torque (Nm)
        void set_torque(float tau);

    private:
        // PWM channel
        int channel;
        // Pins
        int pin_enable, pin_current;
};

#endif
