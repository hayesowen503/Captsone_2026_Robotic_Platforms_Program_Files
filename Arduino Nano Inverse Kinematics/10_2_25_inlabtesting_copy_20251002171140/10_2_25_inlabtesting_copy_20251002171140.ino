#include <micro_ros_arduino.h>
#include "kinematics3dof.h"
#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>

#include "pid.h"

geometry_msgs__msg__Vector3 msg_cmd;     // in: {z, roll, pitch}
geometry_msgs__msg__Vector3 msg_lengths; // out: {L1, L2, L3}

// ----------- micro‑ROS entities -----------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t sub_cmd;
rcl_publisher_t pub_lengths;

static void cb_cmd(const void * msgin) {
  auto * in = (const geometry_msgs__msg__Vector3 *)msgin;

  float L[3];
  (void)kin3_lengths_simple(in->x, in->y, in->z, L);


  msg_lengths.x = L[0];
  msg_lengths.y = L[1];
  msg_lengths.z = L[2];

  micro_ros_commanded_length = msg_lengths.x;

  msg_lengths.z = micro_ros_commanded_length * 1000;
  msg_lengths.y = (float)used_sp;

  rcl_publish(&pub_lengths, &msg_lengths, NULL);

}

void setup() {
  // USB CDC
  Serial.begin(115200);
  // Wait briefly for host to open the port (optional but handy on RP2040)
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { ; }

  // micro‑ROS over serial (uses the default Serial instance)
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "nano_rp2040_ik_serial_node", "", &support);

  // Publisher: leg lengths (mm)
  rclc_publisher_init_default(
    &pub_lengths,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/platform/leg_lengths_mm");

  // Subscription: pose command {x=z_mm, y=roll_deg, z=pitch_deg}
  rclc_subscription_init_default(
    &sub_cmd,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/platform/cmd_rpyz");

  // Executor (1 subscription)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cb_cmd, ON_NEW_DATA);

  rcl_publish(&pub_lengths, &msg_lengths, NULL);


  pidSetup();

}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));

  pidLoop();
}
