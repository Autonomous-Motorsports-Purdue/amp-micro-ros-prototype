#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <SPI.h>
#include <RH_RF95.h>

#define FS1 0
#define FWD 1
// #define REV 2
#define THR 2
// #define THR 3
#define FB 4
#define BRK_1 5
#define BRK_2 6
#define STR 7
#define BRK_EN 18

#define LED_PIN 21
#define SET 22
#define RST 23

// LoRa definitions
#define RFM95_RST     9
#define RFM95_CS      10
#define RFM95_INT     8
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// throttle and steering variables
static uint8_t thr_jetson;
static uint8_t thr_lora;
static uint8_t str_jetson;
static uint8_t str_lora;
static bool jetson_enabled;
static bool estop;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      return false;                \
    }                              \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do {                                 \
    static volatile int64_t init = -1; \
    if (init == -1) {                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS) {    \
      X;                               \
      init = uxr_millis();             \
    }                                  \
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

double clamp(double var, double min, double max) {
  return var > max ? max : var < min ? min : var;
}

// Twist message Callback
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  double linear = msg->linear.x;
  double angular = msg->angular.z;

  str_jetson = map(constrain(angular * -1000, -300, 300), -400, 400, 0, 255);
  thr_jetson = map(constrain(linear * 1000, -700, 700), -700, 700, 0, 255);
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
  // digitalWrite(REV, LOW);

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));

  return true;
}

void estop() {
  analogWrite(THR, 128);                      // 50% duty cycle for frequency modulation
  analogWriteFrequency(THR, 500 / 2.729);  // divide 500 / throttle to get time high (in ms)

  analogWrite(STR, 128);  // 50% duty cycle for frequency modulation
  analogWriteFrequency(STR, 35000); // hard code middle

  // digitalWrite(BRK_1, LOW);
  // digitalWrite(BRK_2, HIGH);

  while (1) { };
}

void setup() {
  // setup serial and LoRa
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  Serial.println("Feather LoRa TX Test2!");

  // Serial.println(rf95.init());

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Configure serial transport
  // Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Configure pins
  pinMode(FS1, OUTPUT);
  pinMode(BRK_EN, OUTPUT);
  pinMode(FWD, OUTPUT);
  // pinMode(REV, OUTPUT);
  pinMode(STR, OUTPUT);
  pinMode(THR, OUTPUT);
  pinMode(FB, OUTPUT);
  pinMode(BRK_1, OUTPUT);
  pinMode(BRK_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SET, OUTPUT);
  pinMode(RST, OUTPUT);
  analogWriteResolution(8);

  digitalWrite(FS1, HIGH);
  digitalWrite(BRK_EN, HIGH);
  digitalWrite(FWD, LOW);
  // digitalWrite(REV, LOW);
  digitalWrite(STR, LOW);
  // digitalWrite(THR, LOW);
  digitalWrite(FB, LOW);
  digitalWrite(BRK_1, LOW);
  digitalWrite(BRK_2, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(SET, HIGH);
  digitalWrite(RST, LOW);

  state = WAITING_AGENT;
}

void loop() {
  // for (double_t i = 2.729; i <= 5.002; i+=0.05) {
  //   tone(THR, 500/i);
  //   delay(200);
  // }

  uint8_t buf[4];
  uint8_t len = sizeof(buf);

  if (rf95.recv(buf, &len))
  {
    // RH_RF95::printBuffer("Received: ", buf, len);
    Serial.print(" | buf[3]: ");
    Serial.print((uint8_t) buf[3]);
    Serial.print(" | buf[2]: ");
    Serial.print((uint8_t) buf[2]);
    Serial.print(" | buf[1]: ");
    Serial.print((uint8_t) buf[1]);
    Serial.print(" | buf[0]: ");
    Serial.print((uint8_t) buf[0]);
    Serial.print(" | ");
    thr_lora = buf[0];
    str_lora = buf[1];
    estop = buf[2] & 0x10;
    jetson_enabled = buf[2] & 0x08;
    // Serial.print("Got: ");
    // Serial.println((char*)buf);
    // Serial.print("RSSI: ");
    // Serial.println(rf95.lastRssi(), DEC);

    // Send a reply
    uint8_t data[] = "$";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
    // Serial.println("Sent a reply");

    Serial.print("Throttle: ");
    Serial.print(thr_lora);
    Serial.print(" | Steering: ");
    Serial.print(str_lora);
    Serial.print(" | Jetson : ");
    Serial.print(jetson_enabled);
    Serial.print(" | ESTOP: ");
    Serial.println(estop);
  }
  // else
  // {
  //   Serial.println("Receive failed");
  // }

  digitalWrite(BRK_1, HIGH);
  digitalWrite(BRK_2, LOW);

  if (estop) {
    estop();
  }

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT);
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

  if (jetson_enabled) {  // alan write in here
    analogWrite(THR, 128);                      // 50% duty cycle for frequency modulation
    analogWriteFrequency(THR, 500 / map(thr_jetson, 0, 255, 1.729, 3.729));  // divide 500 / throttle to get time high (in ms)

    analogWrite(STR, 128);  // 50% duty cycle for frequency modulation
    analogWriteFrequency(STR, map(str_jetson, 0, 255, 10000, 60000));

  } else {
    analogWrite(THR, 128);                      // 50% duty cycle for frequency modulation
    // analogWriteFrequency(THR, 500.0 / map((double) thr_lora, 0, 255, 1.729, 3.729));  // divide 500 / throttle to get time high (in ms)
    analogWriteFrequency(THR, 500.0 / map((double) thr_lora, 0, 255, 0.456, 5.002));  // divide 500 / throttle to get time high (in ms)

    analogWrite(STR, 128);  // 50% duty cycle for frequency modulation
    analogWriteFrequency(STR, map(str_lora, 0, 255, 10000, 60000));
  }
}
