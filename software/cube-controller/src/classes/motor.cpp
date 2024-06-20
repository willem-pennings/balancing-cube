#include "motor.h"

// Class constructor
Motor::Motor(int pin_enable, int pin_current) : pin_enable(pin_enable), pin_current(pin_current) {
    // Configure enable pin as digital output
    pinMode(pin_enable, OUTPUT);

    // Map pin number to channel (these must be updated if mx_current in pins.h is changed...)
    if(pin_current == 12) {
        channel = 0;
    } else if(pin_current == 18) {
        channel = 1;
    } else {
        channel = 2;
    }

    // Configure current pin as 1 kHz 12-bit PWM output
    ledcSetup(channel, 1000, 12);
    ledcAttachPin(pin_current, channel);
}

void Motor::init() {
    // Briefly set current to verify motor behaviour
    set_current(0.5);
    delay(250);
    set_current(-0.4);
    delay(250);
    set_current(0);
}

void Motor::set_current(float ia) {
    // Variables
    bool enable;
    float current;

    // Reverse motor current because of definitions (positive corresponds to CCW rotation)
    ia = -ia;

    if (ia == 0.0) {
        // If current is zero, disable motor and set corresponding PWM value
        enable = false;
        current = 0.5;   
    } else {
        // Enable motor
        enable = true;

        // Set PWM value according to current input. Upper and lower 10% are not used for reliability. ESCON drivers are
        // configured accordingly.
        if(ia > ia_max) {
            // Saturation at upper limit
            current = 0.9;
        } else if (ia < -ia_max) {
            // Saturation at lower limit
            current = 0.1;
        } else {
            // Interpolation in valid range
            current = 0.5 + ia * (0.8 / (2.0 * ia_max));
        }
    }

    // Set enable pin
    digitalWrite(pin_enable, enable);

    // Map 0-1 PWM value to 12-bit range (0-4095) and write duty
    uint32_t duty = current * 4095;
    ledcWrite(channel, duty);
}

void Motor::set_torque(float tau) {
    // Convert torque to current using motor torque constant Km
    set_current(tau / Km);
}
