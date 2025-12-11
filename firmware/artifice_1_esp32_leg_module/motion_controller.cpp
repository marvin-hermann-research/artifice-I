#include "motion_controller.hpp"
#include "servo_driver.hpp"
#include <cstdio>

MotionController* MotionController::globalInstance = nullptr;

const gpio_num_t MotionController::DEAD_STOP_BUTTON = GPIO_NUM_27;

MotionController::MotionController(ServoDriver& driver) 
    : driver(&driver) // speichere Pointer intern
{
    // Optional: setze globale Instance
    globalInstance = this;

    for (size_t i = 0; i < NUM_SERVOS; ++i) {
        homed[i]        = false;
        homed_min[i]    = 0;
        homed_max[i]    = 0;

        raw_angle[i]    = 0;

        current_angles[i].store(0);
        target_angles[i].store(0);
    }
}

void MotionController::initialize() {
    // 1) Dead-Stop-GPIO konfigurieren (vor homing!)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << DEAD_STOP_BUTTON);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);   // jetzt ist gpio_get_level safe [web:2]

    // 2) Homing einmalig ausführen (blockierend, noch keine Tasks)
    homeAllServos();

    // 3) Servo-Task starten
    BaseType_t ok = xTaskCreate(servoTaskWrapper, "servo_task", 4096, nullptr, 5, nullptr);
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

void MotionController::servoTaskWrapper(void* param) {
    if (globalInstance) globalInstance->allServosLoop();
    vTaskDelete(nullptr);
}

bool MotionController::homeServo(int index) {
    const int step        = 1;    // gleicher step wie im allServosLoop
    const int delay_ms    = 20;   // gleiche Zykluszeit
    const int max_steps   = 400;  // Sicherheitslimit gegen Endlosschleife

    // 1) Positive Richtung suchen
    int steps_pos = 0;
    while (gpio_get_level(DEAD_STOP_BUTTON) == 1 && steps_pos < max_steps) {
        // raw angle des servos anpassen 
        raw_angle[index] += step;
        driver->moveToRawAngle(index, raw_angle[index]);
        steps_pos++;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    if (gpio_get_level(DEAD_STOP_BUTTON) == 1) {
        printf("No + deadstop for servo %d\n", index);
        return false;
    }
       
    // gehe alle positiven schritte zurück bis zur ausgangsposition
    for (int i = 0; i < steps_pos; ++i) {
        raw_angle[index] -= step;
        driver->moveToRawAngle(index, raw_angle[index]); //negativ step senden
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // 2) Negative Richtung suchen
    int steps_neg = 0;
    while (gpio_get_level(DEAD_STOP_BUTTON) == 1 && steps_neg < max_steps) {
        raw_angle[index] -= step;
        driver->moveToRawAngle(index, raw_angle[index]);
        steps_neg++;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    if (gpio_get_level(DEAD_STOP_BUTTON) == 1) {
        printf("No - deadstop for servo %d\n", index);
        return false;
    }
       
    // nun berechne wo die mitte ist anhand der gesamt genommenen steps
    int allSteps = steps_pos + steps_neg;
    int stepToCenter = allSteps / 2;

    //fahre die mitte an
    for (int i = 0; i < stepToCenter; ++i) {
        raw_angle[index] += step;
        driver->moveToRawAngle(index, raw_angle[index]); //positiv step senden
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // setze den aktuellen angle auf 90 das ist die mittelposition und hier befindet man sich nun
    current_angles[index].store(90);
    target_angles[index].store(90);

    // berechne die Grenzwinkel: anzahl nehmbarer schritte von der mitte aus
    int neg_limit = 90 - stepToCenter;
    int pos_limit = 90 + stepToCenter;

    // speichere die homing daten
    homed_min[index] = neg_limit;
    homed_max[index] = pos_limit;
    homed[index] = true;

    return true;
}

void MotionController::homeAllServos() {
    for (size_t i = 0; i < NUM_SERVOS; ++i) {
        homed[i] = false;
        if (!homeServo(i)) {
            printf("Homing failed on servo %d\n", (int)i);
            // homed[i] bleibt false
        }
    }
}


