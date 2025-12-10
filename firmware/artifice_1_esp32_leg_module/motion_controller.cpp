#include "motion_controller.hpp"
#include "servo_driver.hpp"
#include <cstdio>

MotionController* MotionController::globalInstance = nullptr;

MotionController::MotionController(ServoDriver& driver) 
    : driver(&driver) // speichere Pointer intern
{
    // Optional: setze globale Instance
    globalInstance = this;
}

void MotionController::initialize() {
    // Initialisiere Startwinkel für alle Servos
    for (size_t i = 0; i < NUM_SERVOS; ++i) {
        current_angles[i].store(START_ANGLE);
        target_angles[i].store(START_ANGLE);
        driver->setAngle(START_ANGLE, i);
    }

    // Nur ein Task für alle Servos
    BaseType_t ok = xTaskCreate(taskWrapper, "servo_task", 4096, nullptr, 5, nullptr);
    if (ok != pdPASS) {
        printf("Failed to create main servo task\n");
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

void MotionController::allServosLoop() {
    const int stepSize = 1;     // Schrittweite
    const int loopDelayMs = 20; // Zykluszeit für alle Servos (momentan extra langsam für Testzwecke)

    while (true) {

        for (size_t i = 0; i < NUM_SERVOS; ++i) {

            int current = current_angles[i].load();
            int target  = target_angles[i].load();

            if (current < target) {
                current += stepSize;
                if (current > target) current = target; // overshoot vermeiden
            }
            else if (current > target) {
                current -= stepSize;
                if (current < target) current = target;
            }

            current_angles[i].store(current);
            driver->setAngle(current, i);
        }

        // EIN Delay für ALLE Servos
        vTaskDelay(pdMS_TO_TICKS(loopDelayMs));
    }
}

void MotionController::taskWrapper(void* param) {
    if (globalInstance) globalInstance->allServosLoop();
    vTaskDelete(nullptr);
}
