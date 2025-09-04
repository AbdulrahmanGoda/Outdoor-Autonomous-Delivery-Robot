#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <Adafruit_Sensor.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/u_int8_multi_array.h>
#include <micro_ros_utilities/type_utilities.h>

#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h>

#define LED_PIN 13
#define BNO055_I2C_SDA 21
#define BNO055_I2C_SCL 22

// ----- Error Checking Macros -----
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) { \
    error_loop(); \
  } \
}
#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if ((temp_rc != RCL_RET_OK)) {} \
}

// ----- BNO055 Setup -----
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); 

// ----- Error Loop Function -----
void error_loop() {
  while (1) {
    delay(100);
  }
}

// ----- Wraps theta in Range [-PI, PI] -----
float wrapTheta(float theta) {
  theta = fmod(theta + PI, 2*PI);
  if (theta < 0) theta += 2*PI;
  return theta - PI;
}

// ----- Global micro-ROS Entities -----
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t publisher;
std_msgs__msg__UInt8MultiArray pub_msg;

rcl_subscription_t subscription;
std_msgs__msg__UInt8MultiArray sub_msg;

// ----- Buffer definitions -----
#define RX_BUFFER_SIZE 24

bool FIRST_TIME = true;

uint8_t rxBuffer[RX_BUFFER_SIZE];
int rxBufferIndex = 0;

uint8_t packet[16];

// ----- Global odometry variables -----
float posX = 0.0;
float posY = 0.0;
float accumTheta = 0.0;
unsigned long lastUpdateMicros = 0;

static uint8_t desired_data[8];

////////////////////////////////////////
float number = 0.0;
uint32_t bits;

void subscription_callback(const void * msgin) {
  const std_msgs__msg__UInt8MultiArray *received_msg = (const std_msgs__msg__UInt8MultiArray *)msgin;
  
  if (received_msg->data.size != 8) {
    return;
  }
  
  memcpy(desired_data, received_msg->data.data, 8);
  
  packet[0] = 0xAA;
  for (int i = 0; i < 8; i++) {
    packet[i + 1] = desired_data[i];
  }
  uint8_t xorVal = packet[0];
  for (int i = 1; i <= 8; i++) {
    xorVal ^= packet[i];
  }
  packet[9] = xorVal;
  packet[10] = 0xEE;
  packet[11] = 0xEF;
  
  Serial2.write(packet, 12);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  set_microros_transports();

  
  Wire.setClock(400000); 

  if (!bno.begin()) {
    while (1);
  }

  bno.setExtCrystalUse(true);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "/current_pose"
  ));

  std_msgs__msg__UInt8MultiArray__init(&pub_msg);
  pub_msg.layout.data_offset = 0;
  pub_msg.data.data = rxBuffer;
  pub_msg.data.size = 0;
  pub_msg.data.capacity = RX_BUFFER_SIZE;

  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "/desired_theta"
  ));

  static micro_ros_utilities_memory_conf_t conf = {
    .max_string_capacity = 128,
    .max_ros2_type_sequence_capacity = 8,
    .max_basic_type_sequence_capacity = 8
  };

  if (!micro_ros_utilities_create_message_memory(
         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
         &sub_msg,
         conf))
  {
    error_loop();
  }

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscription, &sub_msg, &subscription_callback, ON_NEW_DATA));

  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  delay(2000);

  lastUpdateMicros = micros();
}

void loop() {
  while (Serial2.available() > 0 && rxBufferIndex < RX_BUFFER_SIZE) {
    rxBuffer[rxBufferIndex++] = Serial2.read();
  }

  int i = 0;
  while (i <= rxBufferIndex - 8) {
    if (rxBuffer[i] == 0xAA && rxBuffer[i+6] == 0xEE && rxBuffer[i+7] == 0xEF) {
      uint8_t calcXor = rxBuffer[i] ^ rxBuffer[i+1] ^ rxBuffer[i+2] ^ rxBuffer[i+3] ^ rxBuffer[i+4];
      if (calcXor == rxBuffer[i+5]) {
        int16_t rawSpeedL = (int16_t)((rxBuffer[i + 2] << 8) | rxBuffer[i + 1]);
        int16_t rawSpeedR = (int16_t)((rxBuffer[i + 4] << 8) | rxBuffer[i + 3]);

        const float convFactor = (PI) * 0.165 / 120.0;
        float vL = rawSpeedL * convFactor;
        float vR = rawSpeedR * convFactor;
        float meanSpeed = (vL + vR) / 2.0;
        float difference = vR - vL;

        unsigned long nowMicros = micros();
        float dt = (nowMicros - lastUpdateMicros) / 1000000.0;
        lastUpdateMicros = nowMicros;

        float omega = difference / 0.441f;
        float motorTheta = wrapTheta(accumTheta + omega * dt); 

        //sensors_event_t event; 
        //bno.getEvent(&event);
        //float bnoTheta = wrapTheta(event.orientation.z * DEG_TO_RAD); 
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        float bnoTheta = -1*wrapTheta(euler.x() * DEG_TO_RAD);
        float alpha = 0.9f;
        accumTheta = alpha * bnoTheta + (1.0f - alpha) * motorTheta;
        
        float distance = meanSpeed * dt;
        posX += distance * cos(accumTheta);
        posY += distance * sin(accumTheta);

        uint8_t outPacket[16];
        outPacket[0] = 0xAA;
        memcpy(&outPacket[1], &posX, 4);
        memcpy(&outPacket[5], &posY, 4);
        memcpy(&outPacket[9], &accumTheta, 4);
        uint8_t outXor = outPacket[0];
        for (int j = 1; j < 13; j++) {
          outXor ^= outPacket[j];
        }
        outPacket[13] = outXor;
        outPacket[14] = 0xEE;
        outPacket[15] = 0xEF;

        memcpy(rxBuffer, outPacket, 16);
        pub_msg.data.size = 16;
        RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
        i += 8;
        continue;
      }
    }
    i++;
  }

 if (i < rxBufferIndex) {
    int remaining = rxBufferIndex - i;
    memmove(rxBuffer, rxBuffer + i, remaining);
    rxBufferIndex = remaining;
  } else {
    rxBufferIndex = 0;
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  if (millis() - (lastUpdateMicros / 1000) > 5000 || FIRST_TIME) {
    lastUpdateMicros = micros();
    FIRST_TIME = false;

    memcpy(&bits, &number, sizeof(bits)); // Copy float bits to uint32_t

    rxBuffer[0] = 0xAA;

    rxBuffer[4] = (bits >> 24) & 0xFF;
    rxBuffer[3] = (bits >> 16) & 0xFF;
    rxBuffer[2] = (bits >> 8)  & 0xFF;
    rxBuffer[1] = bits & 0xFF;

    rxBuffer[5] = 0;
    rxBuffer[6] = 0;
    rxBuffer[7] = 0;
    rxBuffer[8] = 0;

    rxBuffer[9] = 0;
    rxBuffer[10] = 0;
    rxBuffer[11] = 0;
    rxBuffer[12] = 0;

    rxBuffer[13] = rxBuffer[0]^rxBuffer[1]^rxBuffer[2]^rxBuffer[3]^rxBuffer[4];
    rxBuffer[14] = 0xEE;
    rxBuffer[15] = 0xEF;


    pub_msg.data.size = 16;
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
    rxBufferIndex = 0;
    number += 1;
  }
}