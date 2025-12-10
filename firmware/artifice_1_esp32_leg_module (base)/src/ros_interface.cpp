#include "../include/ros_interface.hpp"
#include <cstdio>

RosInterface* RosInterface::globalInstance = nullptr;

RosInterface::RosInterface(MotionController* controller)
    : motionController(controller)
{
    globalInstance = this;
}

RosInterface::~RosInterface() {
    cleanup();
}

void RosInterface::initialize() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

    // Node erstellen
    RCCHECK(rclc_node_init_default(&node, "servo_subscriber_cpp", "", &support));
    printf("Node created successfully\n");

    // Speicher für Nachrichten initialisieren
    msg_left.data.capacity = 10;
    msg_left.data.data = (int32_t*) allocator.allocate(sizeof(int32_t) * msg_left.data.capacity, allocator.state);
    msg_left.data.size = 0;

    msg_right.data.capacity = 10;
    msg_right.data.data = (int32_t*) allocator.allocate(sizeof(int32_t) * msg_right.data.capacity, allocator.state);
    msg_right.data.size = 0;

    // Subscriptions erstellen
    RCCHECK(rclc_subscription_init_default(
        &subscriber_left,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/left/angles"));

    RCCHECK(rclc_subscription_init_default(
        &subscriber_right,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/right/angles"));

    // Executor erstellen und Subscriptions hinzufügen
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_left, &msg_left,
        &RosInterface::static_left_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_right, &msg_right,
        &RosInterface::static_right_callback, ON_NEW_DATA));
}

void RosInterface::spin() {
    while(true) {
        // print a message every 10 seconds to show that we are alive
        static int counter = 0;
        if (++counter % 200 == 0) {
            printf("Ros Interface alive\n");
            counter = 0;
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void RosInterface::cleanup() {
    rclc_executor_fini(&executor);
    RCCHECK(rcl_subscription_fini(&subscriber_left, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_right, &node));
    RCCHECK(rcl_node_fini(&node));

    if(msg_left.data.data) allocator.deallocate(msg_left.data.data, allocator.state);
    if(msg_right.data.data) allocator.deallocate(msg_right.data.data, allocator.state);
}

void RosInterface::static_left_callback(const void* msgin) {
    const std_msgs__msg__Int32MultiArray* msg = static_cast<const std_msgs__msg__Int32MultiArray*>(msgin);
    if(!msg || msg->data.size == 0) return;

    for(size_t i = 0; i < 2 && i < msg->data.size; ++i) {
        if(globalInstance && globalInstance->motionController)
            globalInstance->motionController->setTargetAngle(static_cast<int>(msg->data.data[i]), i);
    }
}

void RosInterface::static_right_callback(const void* msgin) {
    const std_msgs__msg__Int32MultiArray* msg = static_cast<const std_msgs__msg__Int32MultiArray*>(msgin);
    if(!msg || msg->data.size == 0) return;

    for(size_t i = 0; i < 2 && i < msg->data.size; ++i) {
        if(globalInstance && globalInstance->motionController)
            globalInstance->motionController->setTargetAngle(static_cast<int>(msg->data.data[i]), i+2);
    }
}
