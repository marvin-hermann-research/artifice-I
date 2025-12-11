#include "servo_driver.hpp"

const gpio_num_t ServoDriver::SERVO_PINS[] = {
    GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5,
    GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4
};

const ledc_channel_t ServoDriver::LEDC_CHANNELS[] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
    LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5
};

// Alle Servos benutzen denselben Timer (LEDC_TIMER_0)
const ledc_timer_t ServoDriver::LEDC_TIMERS[] = {
    LEDC_TIMER_0, LEDC_TIMER_0, LEDC_TIMER_0,
    LEDC_TIMER_0, LEDC_TIMER_0, LEDC_TIMER_0
};

void ServoDriver::initializePWM() {
    // Einen Timer konfigurieren (einmal)
    ledc_timer_config_t timer_conf{};
    timer_conf.speed_mode       = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num        = LEDC_TIMER_0;
    timer_conf.duty_resolution  = LEDC_TIMER_16_BIT;
    timer_conf.freq_hz          = 50; // Standard Servo-Freq
    ledc_timer_config(&timer_conf);

    // Channels konfigurieren (ein Channel pro Servo)
    for (size_t i = 0; i < NUM_SERVOS; ++i) {
        ledc_channel_config_t ch_conf{};
        ch_conf.channel    = LEDC_CHANNELS[i];
        ch_conf.gpio_num   = SERVO_PINS[i];
        ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
        ch_conf.timer_sel  = LEDC_TIMER_0;
        ch_conf.duty       = 0;
        ledc_channel_config(&ch_conf);
    }
}

void ServoDriver::setAngle(int angle, int index) {
    if (index < 0 || index >= static_cast<int>(NUM_SERVOS)) return;

    const int pulse_min_us = 500;
    const int pulse_max_us = 2500;

    // frei gewählter "Winkel" -> Pulse, dann auf sicheren Bereich klemmen
    int pulse = pulse_min_us + (angle * (pulse_max_us - pulse_min_us) / 180);
    if (pulse < pulse_min_us) pulse = pulse_min_us;
    if (pulse > pulse_max_us) pulse = pulse_max_us;

    uint32_t duty = static_cast<uint32_t>((int64_t)pulse * 65535LL / 20000LL);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNELS[index], duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNELS[index]);
}

void ServoDriver::moveToRawAngle(int index, int raw_angle) {
    if (index < 0 || index >= static_cast<int>(NUM_SERVOS)) return;

    // MG90S-typischer Pulsbereich: ~500–2500 µs bei 50 Hz [web:101][web:111]
    const int pulse_min_us = 500;
    const int pulse_max_us = 2500;

    // Mapping: Rohwinkel -> Pulsbreite
    int pulse = pulse_min_us + (raw_angle * (pulse_max_us - pulse_min_us) / 180);

    // Sicherheits-Clip nur auf PWM, nicht auf "Winkel"
    if (pulse < pulse_min_us) pulse = pulse_min_us;
    if (pulse > pulse_max_us) pulse = pulse_max_us;

    uint32_t duty = static_cast<uint32_t>((int64_t)pulse * 65535LL / 20000LL);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNELS[index], duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNELS[index]);
}
