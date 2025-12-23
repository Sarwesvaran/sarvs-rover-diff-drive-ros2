#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <RMCS2303drive.h>
#include "CytronMotorDriver.h"
#include <math.h>

// --- CONFIGURATION ---
#define ODEM_RESET 1

// 1. PIN DEFINITIONS
#define LEFT_PWM_PIN 32
#define LEFT_DIR_PIN 25
#define RIGHT_PWM_PIN 33
#define RIGHT_DIR_PIN 26
#define RMCS_RX_PIN 16
#define RMCS_TX_PIN 17

// 2. ROBOT & MOTOR PHYSICAL CONSTANTS
// RMCS-3014 specific: 4680 CPR at Output Shaft
const float TICKS_PER_REV = 4680.0;
float WHEEL_RADIUS = 0.055;
float TRACK_WIDTH = 0.384;
// Max speed: 200 RPM = (200 * 2 * PI / 60) ≈ 20.94 rad/s
float MAX_WHEEL_RAD_S = 20.94;

// --- OBJECTS ---
CytronMD left_motor(PWM_DIR, LEFT_PWM_PIN, LEFT_DIR_PIN);
CytronMD right_motor(PWM_DIR, RIGHT_PWM_PIN, RIGHT_DIR_PIN);
RMCS2303 rmcs;
byte slave_id_left = 1;
byte slave_id_right = 2;

// --- ROS 2 ENTITIES ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Int32MultiArray msg_wheels;

// Variables
int32_t memory[2];
#if ODEM_RESET
int32_t temp_encoder[2] = {0, 0};
bool reset_trigger = false;
#endif

// Velocity Tracking
unsigned long last_time = 0;
int32_t last_encoder_left = 0;
int32_t last_encoder_right = 0;
float left_rpm = 0;
float right_rpm = 0;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void error_loop()
{
  while (1)
  {
    digitalWrite(2, !digitalRead(2));
    delay(100);
  }
}

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  float wheel_left_rad_s = (linear_x - (angular_z * (TRACK_WIDTH / 2.0))) / WHEEL_RADIUS;
  float wheel_right_rad_s = (linear_x + (angular_z * (TRACK_WIDTH / 2.0))) / WHEEL_RADIUS;

  int left_pwm = (int)mapFloat(fabs(wheel_left_rad_s), 0.0, MAX_WHEEL_RAD_S, 0, 255);
  int right_pwm = (int)mapFloat(fabs(wheel_right_rad_s), 0.0, MAX_WHEEL_RAD_S, 0, 255);

  if (left_pwm > 255)
    left_pwm = 255;
  if (right_pwm > 255)
    right_pwm = 255;

  // Left Motor
  if (wheel_left_rad_s > 0.01)
    left_motor.setSpeed(left_pwm);
  else if (wheel_left_rad_s < -0.01)
    left_motor.setSpeed(-left_pwm);
  else
    left_motor.setSpeed(0);

  // Right Motor
  if (wheel_right_rad_s > 0.01)
    right_motor.setSpeed(right_pwm);
  else if (wheel_right_rad_s < -0.01)
    right_motor.setSpeed(-right_pwm);
  else
    right_motor.setSpeed(0);

#if ODEM_RESET
  if (msg->angular.x == 1.0)
    reset_trigger = true;
#endif
}

void setup()
{
  pinMode(2, OUTPUT);
  Serial2.begin(9600, SERIAL_8N1, RMCS_RX_PIN, RMCS_TX_PIN);
  rmcs.begin(&Serial2, 9600);

  left_motor.setSpeed(0);
  right_motor.setSpeed(0);

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    error_loop();
  if (rclc_node_init_default(&node, "esp32_driver", "", &support) != RCL_RET_OK)
    error_loop();

  rclc_publisher_init_default(&publisher, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "value_wheels");

  rclc_subscription_init_default(&subscriber, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, &subscription_callback, ON_NEW_DATA);

  msg_wheels.data.capacity = 2;
  msg_wheels.data.size = 2;
  msg_wheels.data.data = memory;

#if ODEM_RESET
  temp_encoder[0] = rmcs.Position_Feedback(slave_id_left);
  temp_encoder[1] = rmcs.Position_Feedback(slave_id_right);
#endif
  last_time = millis();
}

void loop()
{
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;

  // 1. READ RAW ENCODERS
  int32_t raw_left = rmcs.Position_Feedback(slave_id_left);
  int32_t raw_right = rmcs.Position_Feedback(slave_id_right);

  // 2. VELOCITY CALCULATION (RPM)
  if (dt >= 0.05)
  {
    int32_t d_left = raw_left - last_encoder_left;
    int32_t d_right = raw_right - last_encoder_right;

    // RPM based on 4680 CPR
    left_rpm = (d_left / TICKS_PER_REV) / (dt / 60.0);
    right_rpm = (d_right / TICKS_PER_REV) / (dt / 60.0);

    last_time = current_time;
    last_encoder_left = raw_left;
    last_encoder_right = raw_right;
  }

  // 3. ODOMETRY DATA & DIRECTION NORMALIZATION
#if ODEM_RESET
  if (reset_trigger)
  {
    temp_encoder[0] = raw_left;
    temp_encoder[1] = raw_right;
    reset_trigger = false;
  }
  // Normalizing directions: Flipping left to match forward convention
  msg_wheels.data.data[0] = -1 * (raw_left - temp_encoder[0]);
  msg_wheels.data.data[1] = (raw_right - temp_encoder[1]);
#else
  msg_wheels.data.data[0] = -1 * raw_left;
  msg_wheels.data.data[1] = raw_right;
#endif

  // 4. PUBLISH TO ROS 2
  rcl_ret_t ret = rcl_publish(&publisher, &msg_wheels, NULL);
  if (ret != RCL_RET_OK)
  {
    // Optional: Flash LED or handle error if publish fails
  }

  // 5. SPIN EXECUTOR
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}
