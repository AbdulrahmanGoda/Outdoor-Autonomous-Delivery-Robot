#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/u_int16_multi_array.h>
#include <Arduino.h>

rcl_publisher_t publisher;
std_msgs__msg__UInt16MultiArray msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Buffer to hold raw serial data (32 bytes).
byte buffer[32];
int bufferIndex = 0;
// Static array to serve as the data storage for the published message (16 uint16_t elements).
static uint16_t message_data[16];

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "micro_ros_arduino_node_publisher"
  ));
  
  // Initialize the message: fixed-size sequence for 16 uint16_t elements.
  msg.layout.data_offset = 0;
  msg.data.data = message_data;
  msg.data.size = 0;
  msg.data.capacity = 16;
  
  // Initialize UART2 (RX2 = GPIO16, TX2 = GPIO17).
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
  // Read from UART2 (RX2) and store incoming bytes into the buffer.
  while (Serial2.available() > 0) {
    buffer[bufferIndex] = Serial2.read();
    bufferIndex++;
    
    // When 32 bytes have been collected, convert them into 16 uint16_t values.
    if (bufferIndex >= 32) {
      for (int i = 0; i < 16; i++) {
        // Combine two bytes into one uint16_t (little-endian):
        message_data[i] = ((uint16_t)buffer[2*i]) | (((uint16_t)buffer[2*i+1]) << 8);
      }
      msg.data.size = 16;
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
      bufferIndex = 0;
    }
  }
}
