#pragma once
#include <atomic>
#include "servo_driver.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class MotionController {
public:
    MotionController(ServoDriver& driver); // Konstruktor mit Referenz auf ServoDriver
    void initialize();
    void setTargetAngle(int angle, int index);
    void spin();

    static MotionController* globalInstance;

    static const size_t NUM_SERVOS = 6;

private:
    // Haupt-Loop für einen Servo
    void servoLoop(int index);

    // Wrapper, der von FreeRTOS Task aufgerufen wird
    static void taskWrapper(void* param);
    static const int START_ANGLE = 100;

    ServoDriver* driver;
    std::atomic<int> current_angles[NUM_SERVOS];
    std::atomic<int> target_angles[NUM_SERVOS];
    int task_indices[NUM_SERVOS];
};
