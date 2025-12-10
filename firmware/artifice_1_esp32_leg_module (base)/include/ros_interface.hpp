#pragma once
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <atomic>
#include "motion_controller.hpp"

class RosInterface {
public:
    RosInterface(MotionController* controller); // Referenz auf MotionController
    ~RosInterface();

    static RosInterface* globalInstance; // Zugriff für statische Callbacks

    void initialize();
    void spin(); // Spin-Methode für die ROS-Executor-Schleife

private:
    MotionController* motionController;

    rcl_node_t node{};
    rcl_subscription_t subscriber_left{};
    rcl_subscription_t subscriber_right{};
    rclc_executor_t executor{};
    rclc_support_t support{};
    rcl_allocator_t allocator{};

    std_msgs__msg__Int32MultiArray msg_left{};
    std_msgs__msg__Int32MultiArray msg_right{};

    void cleanup();

    static void static_left_callback(const void* msgin);
    static void static_right_callback(const void* msgin);
};
