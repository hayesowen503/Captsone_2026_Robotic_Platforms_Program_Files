#include "ros_interface.h"

// ----------- micro‑ROS entities -----------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t sub_cmd;
rcl_publisher_t pub_lengths;

geometry_msgs__msg__Vector3 msg_cmd;     // in: {z, roll, pitch}
geometry_msgs__msg__Vector3 msg_lengths; // out: {L1, L2, L3}