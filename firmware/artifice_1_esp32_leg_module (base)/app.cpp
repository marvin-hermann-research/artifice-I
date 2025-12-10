#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

#include "include/motion_controller.hpp"
#include "include/ros_interface.hpp"
#include "include/servo_driver.hpp"

// Haupt-Entry für ESP32 FreeRTOS
extern "C" void appMain(void* arg) {

    // ServoDriver erstellen und PWM initialisieren
    ServoDriver* driver = new ServoDriver();
    driver->initializePWM();

    // MotionController erstellen und starten
    MotionController* motionController = new MotionController(*driver);
    MotionController::globalInstance = motionController;
    motionController->initialize();

    // RosInterface erstellen und starten
    RosInterface* rosInterface = new RosInterface(motionController);
    RosInterface::globalInstance = rosInterface;
    rosInterface->initialize();

    printf("ESP32 Multi-File Servo Controller started!\n");

    // Optional: MotionController in eigenem Task spinnen (dient nur für Alive-Prints)
    xTaskCreate([](void* param){
        MotionController* mc = static_cast<MotionController*>(param);
        mc->spin();
    }, "motion_spin_task", 4096, motionController, 5, nullptr);

    // RosInterface spinn in eigenem Task
    xTaskCreate([](void* param){
        RosInterface* ri = static_cast<RosInterface*>(param);
        ri->spin();
    }, "ros_spin_task", 4096, rosInterface, 5, nullptr);

    // Die appMain Task kann nun selbst enden oder weiterleben
    while(true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
