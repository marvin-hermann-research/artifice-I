#pragma once
#include <atomic>
#include "servo_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class MotionController {
public:
    MotionController(ServoDriver& driver); // constcuctor mit Referenz auf ServoDriver
    void initialize();
    void setTargetAngle(int angle, int index);
    void spin();

    static MotionController* globalInstance;

    static const size_t NUM_SERVOS = 6;


private:
    void allServosLoop();           // Neuer gemeinsamer Loop
    static void servoTaskWrapper(void*); // FreeRTOS Wrapper

    // Homing als Instanzmethoden
    bool homeServo(int index);
    void homeAllServos();

    static const int START_ANGLE = 90;

    ServoDriver* driver;
    std::atomic<int> current_angles[NUM_SERVOS];
    std::atomic<int> target_angles[NUM_SERVOS];

    int raw_angle[NUM_SERVOS];             // interner "Rohwinkel" wird gebraucht für die take steps methode in servo driver


    // Homing-Daten pro Servo
    int  homed_min[NUM_SERVOS];
    int  homed_max[NUM_SERVOS];
    bool homed[NUM_SERVOS];

    static const gpio_num_t DEAD_STOP_BUTTON;
};
