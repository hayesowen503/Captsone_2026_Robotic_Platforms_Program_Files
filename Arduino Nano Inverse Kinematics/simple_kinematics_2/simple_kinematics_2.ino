// Nano RP2040 Connect — 3‑DOF Stewart (half‑Stewart) IK + micro‑ROS over USB (serial)
// Platform: Arduino Nano RP2040 Connect (USB CDC)
// Core: Arduino Mbed OS Nano Boards
// Libraries: micro_ros_arduino
// ROS 2 msgs:
//   IN  /platform/cmd_rpyz : geometry_msgs/msg/Vector3 (x=z_mm, y=roll_deg, z=pitch_deg)
//   OUT /platform/leg_lengths_mm : geometry_msgs/msg/Vector3 (x=L1_mm, y=L2_mm, z=L3_mm)
// Notes:
//  - No Wi‑Fi used. Transport is USB CDC serial via micro‑ROS agent.
//  - Edit geometry below to match your rig (mm). Publish pose from the Pi; lengths come back over USB.

#include "simple_kinematics_2.h"


                                                                                                                     
// Last command (filtered)
static volatile float g_z_mm = Z_HOME_MM;
static volatile float g_roll_deg = 0.0f;
static volatile float g_pitch_deg = 0.0f;

// Low-pass filter (simple 1st order)
static const float ALPHA = 0.25f; // 0..1; higher = snappier

// micro‑ROS subscription callback
static void cb_cmd(const void * msgin) {
  auto * in = (const geometry_msgs__msg__Vector3 *)msgin;
  const float z_cmd    = clampf((float)in->x, Z_MIN_MM, Z_MAX_MM);
  const float roll_cmd = clampf((float)in->y, -ROLL_MAX_DEG, ROLL_MAX_DEG);
  const float pitch_cmd= clampf((float)in->z, -PITCH_MAX_DEG, PITCH_MAX_DEG);

  // Light filtering
  g_z_mm      += ALPHA * (z_cmd    - g_z_mm);
  g_roll_deg  += ALPHA * (roll_cmd - g_roll_deg);
  g_pitch_deg += ALPHA * (pitch_cmd- g_pitch_deg);

  float L[3];
  ik_lengths(g_z_mm, g_roll_deg, g_pitch_deg, L);

  // Publish lengths (mm) as Vector3
  msg_lengths.x = L[0];
  msg_lengths.y = L[1];
  msg_lengths.z = L[2];
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

  // Publish initial leg lengths at home pose
  float L0[3];
  ik_lengths(Z_HOME_MM, 0.0f, 0.0f, L0);
  msg_lengths.x = L0[0];
  msg_lengths.y = L0[1];
  msg_lengths.z = L0[2];
  rcl_publish(&pub_lengths, &msg_lengths, NULL);
}

void loop() {
  // Run executor frequently; adjust timeout as desired
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
  // TODO: Drive your actuators to msg_lengths using your chosen motor driver(s)
}

/* ----------------------
PI SIDE QUICK START (Agent over serial + test)

# 1) Identify the device when you plug in the Nano
ls /dev/ttyACM*

# 2) Start the micro‑ROS agent (native install)
micro-ros-agent serial --dev /dev/ttyACM0 -b 115200 -v6

#    Or in Docker (needs device passthrough):
docker run -it --rm \
  --privileged \
  -v /dev/ttyACM0:/dev/ttyACM0 \
  ghcr.io/micro-ros/micro-ros-agent:humble \
  serial --dev /dev/ttyACM0 -b 115200 -v6

# 3) In another terminal on the Pi (ROS 2 Humble):
ros2 topic echo /platform/leg_lengths_mm

# 4) Send a test pose (z=125 mm, roll=5°, pitch=-3°):
ros2 topic pub /platform/cmd_rpyz geometry_msgs/Vector3 '{x: 125.0, y: 5.0, z: -3.0}' -1

TROUBLESHOOTING
- Make sure you selected: Tools → Board → Arduino Mbed OS Nano Boards → Arduino Nano RP2040 Connect.
- Library: Install `micro_ros_arduino` via Library Manager.
- If the agent can’t open the device, check permissions (e.g., `sudo usermod -aG dialout $USER`, then re‑login) or run with sudo/`--privileged` in Docker.
- If messages don’t appear, keep the agent running before resetting the board; the client will try to connect at startup.
- Baud rate on USB CDC is largely nominal but keep Serial.begin and agent `-b` matched (115200 here).

NOTES
- This publishes only the computed lengths; scale to encoder ticks or microsteps in your motor loop.
- For 6‑DOF in the future, switch the outgoing type to `sensor_msgs/JointState` or `Float32MultiArray` and expand to 6 legs.
*/
