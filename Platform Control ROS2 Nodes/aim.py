#!/usr/bin/env python3
import argparse
import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3



class AimController(Node):
	def __init__(self, x, y, z):
		super().__init__('aim')

		# Publish Vector3 commands to each module
		self.pub_ant = self.create_publisher(Vector3, '/UNC/platform/cmd_rpyz', 10)
     
		period = 0.5
		self.timer = self.create_timer(period, self.aim)
	
		self.x = float(x)         
		self.y = float(y)        
		self.z = float(z)        

		self.height = 0.0
		self.tilt = 0.0
		self.dir = 0.0


	def aim(self):
		"""
		Compute local aiming coordinates for a module given a global target (x, y, z) in meters.
		Publishes direction (self.dir) and tilt angle (self.tilt) (sel (to pass to postion msg)
		"""
		r = math.sqrt(self.x**2 + self.y**2)
		# Tilt angle from vertical
		self.tilt = math.atan2(r, self.z)
	
		# Direction (+x is 0)
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
	node = AimController(args.x, args.y, args.z)
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
