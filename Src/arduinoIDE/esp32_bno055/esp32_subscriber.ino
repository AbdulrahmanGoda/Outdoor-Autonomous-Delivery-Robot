#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>    
#include <std_msgs/msg/int64.h>


#define TX2_BAUDRATE 115200  

rcl_node_t node;
rcl_subscription_t subscription;
std_msgs__msg__Int64 msg;
rclc_executor_t executor;  

// Callback function for received numbers
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int64 *received_msg = (const std_msgs__msg__Int64 *)msgin;
  
  Serial2.println(received_msg->data);
}

void setup() {
  
  Serial2.begin(TX2_BAUDRATE);
  delay(1000);  
  
  set_microros_transports();
  
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);
  
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);

  rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "numbers"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscription, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
