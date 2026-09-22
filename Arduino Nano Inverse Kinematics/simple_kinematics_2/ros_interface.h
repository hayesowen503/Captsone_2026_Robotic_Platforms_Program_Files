

#ifndef ROS_INTERFACE_H_
#define ROS_INTERFACE_H_

#include <rcl/rcl.h>
#include <rclc/rclc.h>

// ----------- micro‑ROS entities -----------
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_subscription_t sub_cmd;
extern rcl_publisher_t pub_lengths;

extern geometry_msgs__msg__Vector3 msg_cmd;     // in: {z, roll, pitch}
extern geometry_msgs__msg__Vector3 msg_lengths; // out: {L1, L2, L3}

#endif