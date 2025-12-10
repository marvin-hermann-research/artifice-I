#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"

class ServoDriver {
public:
    ServoDriver() = default;
    
    // Initializes PWM for servos
    void initializePWM();
    // Sets the angle of the servo at the given index
    void setAngle(int angle, int index);

private:
    // variables for servo controle which will be defined at compile time
    static const gpio_num_t SERVO_PINS[];
    static const size_t NUM_SERVOS = 6;
    static const ledc_channel_t LEDC_CHANNELS[];
    static const ledc_timer_t LEDC_TIMERS[];
};
