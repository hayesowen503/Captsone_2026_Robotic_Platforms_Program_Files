#!/usr/bin/env python3
import math
import argparse
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from geometry_msgs.msg import Vector3

class ManagedSpinner(LifecycleNode):
	def __init__(self, height, angle, step, start):
		super().__init__('managed_spin')
		self.pub_ant = None
		#self.pub_unc = None

		self.height = float(height)          # x = height
		self.angle  = float(angle)           # y = tilt angle (rad)
		self.step   = float(step)            # Δz per tick (rad)
		self.start  = float(start) % (2*math.pi)
		self.z = self.start



	def tick(self):
		if self.pub_ant is not None and self.pub_ant.is_activated:
			# Publish current vector
			msg = Vector3(x=self.height, y=self.angle, z=self.z % (2*math.pi))
			self.pub_ant.publish(msg)
			#self.pub_unc.publish(msg)

			prev = self.z
			self.z += self.step


	def on_configure(self, state: State) -> TransitionCallbackReturn:
		self.get_logger().info("on_configure() called.")
		
		# Publish Vector3 commands to the platform
		self.pub_ant = self.create_lifecycle_publisher(Vector3, '/ANT/platform/cmd_rpyz', 10)
		#self.pub_unc = self.create_lifecycle_publisher(Vector3, '/UNC/platform/cmd_rpyz', 10)
		period = 1.0/40.0
		self.timer = self.create_timer(period, self.tick)
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


def main():
	parser = argparse.ArgumentParser(description="Publish Vector3 to /platform/cmd_rpyz with direction sweeping 0..2π.")
	parser.add_argument('--height', type=float, required=True, help='platform height in meters')
	parser.add_argument('--angle',  type=float, required=True, help='tilt angle in radians')
	parser.add_argument('--step',   type=float, default=0.03,   help='Direction increment per tick in radians (default: 0.03)')
	parser.add_argument('--start',  type=float, default=0.0,    help='Starting direction z in radians (default: 0)')
	args = parser.parse_args()

	rclpy.init()
	node = ManagedSpinner(args.height, args.angle, args.step, args.start)
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if rclpy.ok():
			rclpy.shutdown()
   

if __name__ == '__main__':
	main()
