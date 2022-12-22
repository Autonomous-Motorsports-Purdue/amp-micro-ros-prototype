#include <Arduino.h>

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define FS1 2
#define FWD 3
#define REV 4
#define BRK 13

#define STR 7
#define THR 10
#define FBR 11
#define BRK_0 14
#define BRK_1 15

#define ERR_LED 13

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      digitalWrite(ERR_LED, HIGH); \
      delay(200);                  \
      digitalWrite(ERR_LED, LOW);  \
      delay(200);                  \
      digitalWrite(ERR_LED, HIGH); \
      delay(200);                  \
      digitalWrite(ERR_LED, LOW);  \
    }                              \
  }

// Error handle loop
void error_loop() {
  digitalWrite(FS1, LOW);
  digitalWrite(ERR_LED, HIGH);
  while (1) {
    delay(100);
  }
}

// Twist message Callback
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  double linear = msg->linear.x;
  double angular = msg->angular.z * 1000 + 3000;

  analogWrite(THR, abs(linear * 100));
  if (linear >= 0) {
    digitalWrite(FWD, HIGH);
    digitalWrite(REV, LOW);
  } else {
    digitalWrite(FWD, LOW);
    digitalWrite(REV, HIGH);
  }

  if (angular > 0) {
    digitalWrite(BRK_0, HIGH);
    digitalWrite(BRK_1, LOW);
  } else if (angular < 0) {
    digitalWrite(BRK_0, LOW);
    digitalWrite(BRK_1, HIGH);
  } else {
    digitalWrite(BRK_0, LOW);
    digitalWrite(BRK_1, LOW);
  }
}

// TODO: Add code that enables reconnection when agent is disconnected
// https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino

void setup() {
  // Configure pins
  pinMode(FS1, OUTPUT);
  pinMode(FWD, OUTPUT);
  pinMode(REV, OUTPUT);
  pinMode(BRK, OUTPUT);
  pinMode(STR, OUTPUT);
  pinMode(THR, OUTPUT);
  pinMode(FBR, OUTPUT);
  pinMode(BRK_0, OUTPUT);
  pinMode(BRK_1, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  analogWriteResolution(12);

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  digitalWrite(FS1, HIGH);
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
