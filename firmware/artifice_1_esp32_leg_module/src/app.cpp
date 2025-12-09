#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>

#include <cstdio>
#include <atomic>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){printf("Failed on line %d: %d\n",__LINE__,(int)temp_rc); vTaskDelete(nullptr);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){printf("Soft fail on line %d: %d\n",__LINE__,(int)temp_rc);}}

class ParallelServoController {
private:
    static constexpr gpio_num_t SERVO_PINS[] = {GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_16, GPIO_NUM_19};
    static constexpr size_t NUM_SERVOS = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);
    static constexpr int START_ANGLE = 100;
    static constexpr ledc_channel_t LEDC_CHANNELS[] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};
    static constexpr ledc_timer_t LEDC_TIMERS[] = {LEDC_TIMER_0, LEDC_TIMER_1, LEDC_TIMER_2, LEDC_TIMER_3};

    std::atomic<int> current_angles[NUM_SERVOS];
    std::atomic<int> target_angles[NUM_SERVOS];

    int task_indices[NUM_SERVOS];

    rcl_node_t node{};
    rcl_subscription_t subscriber_left{};
    rcl_subscription_t subscriber_right{};
    rclc_executor_t executor{};
    std_msgs__msg__Int32MultiArray msg_left{};
    std_msgs__msg__Int32MultiArray msg_right{};

    rclc_support_t support{};
    rcl_allocator_t allocator{};

public:
    ParallelServoController() = default;
    ~ParallelServoController() { cleanup(); }

    void initialize() {

        // Initialisiere die aktuellen und Zielwinkel
       for (size_t i = 0; i < NUM_SERVOS; ++i) {
            current_angles[i].store(START_ANGLE);
            target_angles[i].store(START_ANGLE);
            task_indices[i] = static_cast<int>(i);
        }

        // Hardware-PWM Initialisierung
        initializePWM();

         // Setze Servos auf Startwinkel
        for (size_t i = 0; i < NUM_SERVOS; ++i) {
            setAngle(START_ANGLE, (int)i);
            printf("Servo %zu initialized at angle %d\n", i, START_ANGLE);
        }

        // Micro-ROS
        allocator = rcl_get_default_allocator();
        RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

        // Node Creation
        RCCHECK(rclc_node_init_default(&node, "servo_subscriber_cpp", "", &support));
        printf("Node created successfully\n");

        // MANUELLE INITIALISIERUNG DES LINKEN NACHRICHTEN-ARRAYS
        // Weist Speicher für das Daten-Array der Nachricht zu
        msg_left.data.capacity = 10; // Kapazität für bis zu 10 Integer-Werte
        msg_left.data.data = (int32_t*) allocator.allocate(sizeof(int32_t) * msg_left.data.capacity, allocator.state);
        msg_left.data.size = 0; // Zu Beginn sind keine Daten enthalten
        if (msg_left.data.data == NULL) {
            printf("FEHLER: Speicherallokation für msg_left fehlgeschlagen!\n");
        }

        // MANUELLE INITIALISIERUNG DES RECHTEN NACHRICHTEN-ARRAYS
        // Weist Speicher für das Daten-Array der Nachricht zu
        msg_right.data.capacity = 10; // Kapazität für bis zu 10 Integer-Werte
        msg_right.data.data = (int32_t*) allocator.allocate(sizeof(int32_t) * msg_right.data.capacity, allocator.state);
        msg_right.data.size = 0; // Zu Beginn sind keine Daten enthalten
        if (msg_right.data.data == NULL) {
            printf("FEHLER: Speicherallokation für msg_right fehlgeschlagen!\n");
        }

        // Subscription Creation
        // LEFT LEG
        RCCHECK(rclc_subscription_init_default(
            &subscriber_left,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "/left/angles"));
        printf("LEFT subscription created successfully.\n");

        // RIGHT LEG
        RCCHECK(rclc_subscription_init_default(
            &subscriber_right,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "/right/angles"));
        printf("RIGHT subscription created successfully.\n");

        // Executor Creation and Adding Subscription
        RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

        RCCHECK(rclc_executor_add_subscription(
            &executor, &subscriber_left, &msg_left,
            &ParallelServoController::static_left_callback, ON_NEW_DATA));
        printf("Left leg subscription added to executor.\n");
        
        RCCHECK(rclc_executor_add_subscription(
            &executor, &subscriber_right, &msg_right,
            &ParallelServoController::static_right_callback, ON_NEW_DATA));
        printf("Right leg subscription added to executor.\n");

        // Start FreeRTOS Task für jedes Servo
        for (size_t i = 0; i < NUM_SERVOS; ++i) {
            // Übergabe eines Zeigers auf ein dauerhaftes Element (task_indices[i])
            BaseType_t ok = xTaskCreate(taskWrapper, "servo_task", 4096, &task_indices[i], 5, nullptr);
            if (ok != pdPASS) {
                printf("Failed to create task for servo %zu\n", i);
            }
        }
    }

    void spin() {
        while(true) {
            // print a message every 10 seconds to show that we are alive
            static int counter = 0;
            if (++counter % 200 == 0) {
                printf("Servo controller is alive\n");
                counter = 0;
            }

            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

private:
    void servoLoop(int index) {
        while (true) {
            int current = current_angles[index].load();
            int target  = target_angles[index].load();

            if (current < target) current_angles[index].fetch_add(1);
            else if (current > target) current_angles[index].fetch_sub(1);

            setAngle(current_angles[index].load(), index);
            vTaskDelay(pdMS_TO_TICKS(7));
        }
    }

    void initializePWM() {
        //configure timers and channels for each servo
        for (size_t i = 0; i < NUM_SERVOS; ++i) {
            ledc_timer_config_t timer_conf{};
            timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
            timer_conf.timer_num = LEDC_TIMERS[i];
            timer_conf.duty_resolution = LEDC_TIMER_16_BIT;
            timer_conf.freq_hz = 50; // 50 Hz Servo
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

    void setAngle(int angle, int index) {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;

        // 0° -> 500 µs, 180° -> 2500 µs bei 50 Hz
        // Achtung integer division: berechne mit 64-bit um Rundungsfehler zu minimieren
        uint32_t duty = static_cast<uint32_t>(( (int64_t)angle * (65535LL) / 2000LL ) + (65535LL * 500LL / 20000LL));
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNELS[index], duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNELS[index]);
    }

    void setTargetAngle(int angle, int index) {
        target_angles[index].store(angle);
        printf("Target angle set for servo %d: %d\n", index, angle);
    }

    void cleanup() {
        // Executor zuerst schließen
        rclc_executor_fini(&executor);

        // Subscription freigeben
        RCCHECK(rcl_subscription_fini(&subscriber_left, &node));
        RCCHECK(rcl_subscription_fini(&subscriber_right, &node));

        // Node freigeben
        RCCHECK(rcl_node_fini(&node));

        // Nachrichtenspeicher manuell freigeben
        if (msg_left.data.data != NULL) {
            allocator.deallocate(msg_left.data.data, allocator.state);
            msg_left.data.data = NULL;
            msg_left.data.size = 0;
            msg_left.data.capacity = 0;
        }

        // RECHTES NACHRICHTEN-ARRAY FREIGEBEN
        if (msg_right.data.data != NULL) {
            allocator.deallocate(msg_right.data.data, allocator.state);
            msg_right.data.data = NULL;
            msg_right.data.size = 0;
            msg_right.data.capacity = 0;
        }
    }


    static void static_left_callback(const void* msgin) {
        const std_msgs__msg__Int32MultiArray* msg = static_cast<const std_msgs__msg__Int32MultiArray*>(msgin);

        if(msg->data.size == 0) {
            printf("Left leg received empty angle array\n");
            return;
        }
        
        for (size_t i = 0; i < msg->data.size; ++i) {
            printf("Left leg received angle[%zu]: %d\n", i, msg->data.data[i]);
        }
        
        // Set target angles for each servo based on received data and skip if out of bounds hier manuel 2 gesetzt da es nur 2 servos pro bein gibt
        for (size_t i = 0; i < 2; ++i) {
            if (i >= msg->data.size) {
                printf("No angle provided for servo %zu, skipping\n", i);
                continue;
            }
            int angle = static_cast<int>(msg->data.data[i]);
            if(globalInstance) globalInstance->setTargetAngle(angle, i);
        }
    }

    static void static_right_callback(const void* msgin) {
        const std_msgs__msg__Int32MultiArray* msg = static_cast<const std_msgs__msg__Int32MultiArray*>(msgin);

        if(msg->data.size == 0) {
            printf("Right Leg received empty angle array\n");
            return;
        }
        
        for (size_t i = 0; i < msg->data.size; ++i) {
            printf("Right Leg received angle[%zu]: %d\n", i, msg->data.data[i]);
        }
        
        // Set target angles for each servo based on received data and skip if out of bounds
        for (size_t i = 0; i < 2; ++i) {
            if (i >= msg->data.size) {
                printf("No angle provided for servo %zu, skipping\n", i);
                continue;
            }
            int angle = static_cast<int>(msg->data.data[i]);
            if(globalInstance) globalInstance->setTargetAngle(angle, i+2); // Right leg servos start at index 2
        }
    }

    static void taskWrapper(void* param) {
        int index = *static_cast<int*>(param);
        if (globalInstance) globalInstance->servoLoop(index);
        vTaskDelete(nullptr);
    }

public:
    static ParallelServoController* globalInstance;
};

constexpr gpio_num_t ParallelServoController::SERVO_PINS[];
constexpr ledc_channel_t ParallelServoController::LEDC_CHANNELS[];
constexpr ledc_timer_t ParallelServoController::LEDC_TIMERS[];

ParallelServoController* ParallelServoController::globalInstance = nullptr;

extern "C" void appMain(void* arg) {
    ParallelServoController* controller = new ParallelServoController();
    ParallelServoController::globalInstance = controller;
    controller->initialize();
    printf("ESP32 Parallel Servo Subscriber started (C++11 clean)!\n");
    controller->spin();

    delete controller; // theoretisch nie erreicht
}