#include "../include/motion_controller.hpp"
#include "../include/servo_driver.hpp"
#include <cstdio>

MotionController* MotionController::globalInstance = nullptr;

MotionController::MotionController(ServoDriver& driver) 
    : driver(&driver) // speichere Pointer intern
{
    // Optional: setze globale Instance
    globalInstance = this;
}

MotionController::~MotionController() {
    // Hier könnten wir später Cleanup machen, falls nötig
}

void MotionController::initialize() {
    // Initialisiere aktuelle und Zielwinkel
    for (size_t i = 0; i < ServoDriver::NUM_SERVOS; ++i) {
        current_angles[i].store(START_ANGLE);
        target_angles[i].store(START_ANGLE);
        task_indices[i] = static_cast<int>(i);

        // Setze Servo auf Startwinkel über ServoDriver
        driver->setAngle(START_ANGLE, i); // 'driver->' statt 'servoDriver.'
    }

    // Starte FreeRTOS Task für jeden Servo
    for (size_t i = 0; i < ServoDriver::NUM_SERVOS; ++i) {
        BaseType_t ok = xTaskCreate(taskWrapper, "servo_task", 4096, &task_indices[i], 5, nullptr);
        if (ok != pdPASS) {
            printf("Failed to create task for servo %zu\n", i);
        }
    }
}

void MotionController::spin() {
    while (true) {
        // print a message every 10 seconds to show that we are alive
        static int counter = 0;
        if (++counter % 200 == 0) {
            printf("MotionController alive\n");
            counter = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void MotionController::setTargetAngle(int angle, int index) {
    if (index < 0 || index >= static_cast<int>(ServoDriver::NUM_SERVOS)) return;
    target_angles[index].store(angle);
    printf("Target angle set for servo %d: %d\n", index, angle);
}

void MotionController::servoLoop(int index) {
    while (true) {
        int current = current_angles[index].load();
        int target  = target_angles[index].load();

        if (current < target) current_angles[index].fetch_add(1);
        else if (current > target) current_angles[index].fetch_sub(1);

        driver->setAngle(current_angles[index].load(), index);

        vTaskDelay(pdMS_TO_TICKS(7));
    }
}

void MotionController::taskWrapper(void* param) {
    int index = *static_cast<int*>(param);
    if (globalInstance) globalInstance->servoLoop(index);
    vTaskDelete(nullptr);
}
