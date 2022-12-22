#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>

#define FS1 2
#define BRK 13
#define FWD 3
#define REV 4

#define STR 7
#define THR 10
#define FBR 11
#define BRK_0 14
#define BRK_1 15

#define LED_PIN 13

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      return false;                \
    }                              \
  }
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1;    \
  if (init == -1) {                     \
    init = uxr_millis();                \
  }                                     \
  if (uxr_millis() - init > MS) {       \
    X;                                  \
    init = uxr_millis();                \
  }                                     \
} while (0)

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

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

bool create_entities() {
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
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  return true;
}

bool destroy_entities() {
  digitalWrite(FWD, LOW);
  digitalWrite(REV, LOW);

  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));

  return true;
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Configure pins
  pinMode(FS1, OUTPUT);
  pinMode(BRK, OUTPUT);
  pinMode(FWD, OUTPUT);
  pinMode(REV, OUTPUT);
  pinMode(STR, OUTPUT);
  pinMode(THR, OUTPUT);
  pinMode(FBR, OUTPUT);
  pinMode(BRK_0, OUTPUT);
  pinMode(BRK_1, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  analogWriteResolution(12);

  digitalWrite(FS1, HIGH);
  digitalWrite(BRK, HIGH);
  digitalWrite(FWD, LOW);
  digitalWrite(REV, LOW);
  digitalWrite(STR, LOW);
  digitalWrite(THR, LOW);
  digitalWrite(FBR, LOW);
  digitalWrite(BRK_0, LOW);
  digitalWrite(BRK_1, LOW);
  digitalWrite(LED_PIN, LOW);

  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}
