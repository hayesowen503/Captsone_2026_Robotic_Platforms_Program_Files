#!/usr/bin/env python3
import argparse
import math
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from geometry_msgs.msg import Vector3



class ManagedAimController(LifecycleNode):
	def __init__(self, x, y, z):
		super().__init__('managed_aim')
		self.pub_ant = None
		self.timer = None
		self.height = 0.0
		self.tilt = 0.0
		self.dir = 0.0
		self.x = float(x)
		self.y = float(y)
		self.z = float(z)
		
		"""
		self.declare_parameter('x', 0.0)        
		self.declare_parameter('y', 0.0)   
		self.declare_parameter('z', 0.0)           
		"""
		
	def on_configure(self, state: State) -> TransitionCallbackReturn:
		self.get_logger().info("on_configure() called.")
		"""
		x = self.get_parameter('x').get_parameter_value().float_value
		y = self.get_parameter('y').get_parameter_value().float_value
		z = self.get_parameter('z').get_parameter_value().float_value
		"""
		
		# Publish Vector3 commands to the platform
		self.pub_ant = self.create_lifecycle_publisher(Vector3, '/ANT/platform/cmd_rpyz', 10)
		period = 0.5
		self.timer = self.create_timer(period, self.aim)
		return TransitionCallbackReturn.SUCCESS
	
	def on_activate(self, state: State) -> TransitionCallbackReturn:
		self.get_logger().info("on_activate() called. Starting...")
		return super().on_activate(state)
		
		
	def on_deactivate(self, state: State) -> TransitionCallbackReturn:
		self.get_logger().info("on_deactivate() called. Pausing..")
		return super().on_deactivate(state)

	
	def on_cleanup(self, state: State) -> TransitionCallbackReturn:
		self.get_logger().info("on_cleanup() called. Freeing resources..")
		self.destroy_timer(self.timer)
		self.destroy__publisher(self.pub_ant)
		return TransitionCallbackReturn.SUCCESS
		
		
	def on_shutdown(self, state: State) -> TransitionCallbackReturn:
		self.get_logger().info("on_shutdown() called. Shutting down..")
		if self.timer is not None:
			self.destroy_timer(self.timer)
		if self.pub_ant is not None:
			self.destroy_publisher(self.pub_ant)
		return TransitionCallbackReturn.SUCCESS


	def aim(self):#, x, y, z):
		if self.pub_ant is not None and self.pub_ant.is_activated:
			"""
			Compute local aiming coordinates for a module given a global target (x, y, z).
			Publishes direction (self.dir) and tilt angle (self.tilt) which are passed to postion msg.
			"""
			r = math.sqrt(self.x**2 + self.y**2)
			# Tilt angle from vertical
			self.tilt = math.atan2(r, self.z)
		
			# Direction (+x axis is 0 degrees)
			self.dir = math.atan2(self.y, self.x)
			if self.dir < 0:
				self.dir += 6.2832
		
			# Publish current vector
			msg = Vector3(x=self.height, y=self.tilt, z=self.dir)
			self.pub_ant.publish(msg)
			self.get_logger().info(f"Tilt Angle: {self.tilt}, Direction: {self.dir}")
	
	"""
	def update_target(self, height, distance, direction_deg)
		# Update target coordinates.
		# The full target height is stored for tilt calculations.
		
		self._target_height = height
		self.distance = distance
		self.direction_deg = direction_deg
	"""

# -------------------------------------------------------------------------
def main():
	
	parser = argparse.ArgumentParser(description="Publish Vector3 to /platform/cmd_rpyz converted from cartesian x, y, and z coordinates")
	parser.add_argument('--x', type=float, required=True)
	parser.add_argument('--y',  type=float, required=True)
	parser.add_argument('--z',   type=float, required=True)
	args = parser.parse_args()
	
	
	rclpy.init()
	node = ManagedAimController(args.x, args.y, args.z)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if rclpy.ok():
			rclpy.shutdown()

# -------------------------------------------------------------------------
if __name__ == "__main__":
    main()
