#include "servo_driver.hpp"

const gpio_num_t ServoDriver::SERVO_PINS[] = {GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4}; // 19 - 5 leg one, 17 - 4 leg two
const ledc_channel_t ServoDriver::LEDC_CHANNELS[] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5};
const ledc_timer_t ServoDriver::LEDC_TIMERS[] = {LEDC_TIMER_0, LEDC_TIMER_1, LEDC_TIMER_2, LEDC_TIMER_3, LEDC_TIMER_4, LEDC_TIMER_5};

void ServoDriver::initializePWM() {
    for (size_t i = 0; i < NUM_SERVOS; ++i) {
        ledc_timer_config_t timer_conf{};
        timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
        timer_conf.timer_num = LEDC_TIMERS[i];
        timer_conf.duty_resolution = LEDC_TIMER_16_BIT;
        timer_conf.freq_hz = 50;
        ledc_timer_config(&timer_conf);

        ledc_channel_config_t ch_conf{};
        ch_conf.channel = LEDC_CHANNELS[i];
        ch_conf.gpio_num = SERVO_PINS[i];
        ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
        ch_conf.timer_sel = LEDC_TIMERS[i];
        ch_conf.duty = 0;
        ledc_channel_config(&ch_conf);
    }
}

void ServoDriver::setAngle(int angle, int index) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint32_t duty = static_cast<uint32_t>(( (int64_t)angle * (65535LL) / 2000LL ) + (65535LL * 500LL / 20000LL));
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNELS[index], duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNELS[index]);
}
