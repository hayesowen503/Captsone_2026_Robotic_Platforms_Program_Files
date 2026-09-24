#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from std_msgs.msg import Bool


class Orchestrator(Node):
	def __init__(self):
		super().__init__('orchestrator')
		self.client_aim = self.create_client(ChangeState, '/managed_aim/change_state')
		self.client_spin = self.create_client(ChangeState, '/managed_spin/change_state')
		
		self.client_aim.wait_for_service()
		self.client_spin.wait_for_service()
		
		self.change_node_state(self.client_aim, Transition.TRANSITION_CONFIGURE)
		self.change_node_state(self.client_spin, Transition.TRANSITION_CONFIGURE)
		
		self.sub = self.create_subscription(Bool, 'spin_aim_toggle', self.toggle_callback, 10)
		
		self.aim_state = False
		self.spin_state = False
		
	
	def change_node_state(self, client, transition_id):
		request = ChangeState.Request()
		request.transition.id = transition_id
		client.call_async(request)
	
		
	def toggle_callback(self, msg):
		if msg.data:
			self.get_logger().info("Aiming...")
			
			if self.spin_state:
				self.change_node_state(self.client_spin, Transition.TRANSITION_DEACTIVATE)
				self.spin_state = False
			if not self.aim_state:
				self.change_node_state(self.client_aim, Transition.TRANSITION_ACTIVATE)
				self.aim_state = True
		else:
			self.get_logger().info("Spinning...")
		
			if self.aim_state:
				self.change_node_state(self.client_aim, Transition.TRANSITION_DEACTIVATE)
				self.aim_state = False
			if not self.spin_state:
				self.change_node_state(self.client_spin, Transition.TRANSITION_ACTIVATE)
				self.spin_state = True
		
def main():
	rclpy.init()
	node = Orchestrator()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if rclpy.ok():
			rclpy.shutdown()
   

if __name__ == '__main__':
	main()
