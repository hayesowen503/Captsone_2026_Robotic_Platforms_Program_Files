#!/usr/bin/env python3
import math
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class DirectionSpinner(Node):
    def __init__(self, height, angle, step, rate_hz, start, rotations):
        super().__init__('direction_spinner')
        self.pub_ant = self.create_publisher(Vector3, '/ANT/platform/cmd_rpyz', 10)
        self.pub_unc = self.create_publisher(Vector3, '/UNC/platform/cmd_rpyz', 10)

        self.height = float(height)          # x = height
        self.angle  = float(angle)           # y = tilt angle (rad)
        self.step   = float(step)            # Δz per tick (rad)
        self.start  = float(start) % (2*math.pi)
        self.rate_hz = float(rate_hz)
        self.rotations = None if rotations is None else float(rotations)

        self.z = self.start
        self.lap_count = 0
        self.prev_wrap_zone = int(self.z // (2*math.pi))

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            f"Spinning direction: x={self.height:.3f} m, y={self.angle:.3f} rad, "
            f"z starts at {self.z:.6f} rad; step={self.step:.6f} rad @ {self.rate_hz:.2f} Hz"
            + (f"; target rotations={self.rotations}" if self.rotations is not None else "")
        )

    def tick(self):
        # Publish current vector
        msg = Vector3(x=self.height, y=self.angle, z=self.z % (2*math.pi))
        self.pub_ant.publish(msg)
        self.pub_unc.publish(msg)

        # Update z and count full wraps (laps)
        prev = self.z
        self.z += self.step
        if int(self.z // (2*math.pi)) > self.prev_wrap_zone:
            self.lap_count += 1
            self.prev_wrap_zone = int(self.z // (2*math.pi))
            self.get_logger().info(f"Completed rotations: {self.lap_count}")

            if self.rotations is not None and self.lap_count >= self.rotations:
                self.get_logger().info("Reached target rotations. Shutting down.")
                rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description="Publish Vector3 to /platform/cmd_rpyz with direction sweeping 0..2π.")
    parser.add_argument('--height', type=float, required=True, help='x: platform height')
    parser.add_argument('--angle',  type=float, required=True, help='y: tilt angle (radians)')
    parser.add_argument('--rate',   type=float, default=40.0,   help='Publish rate in Hz (default: 40)')
    parser.add_argument('--step',   type=float, default=0.03,   help='Direction increment per tick in radians (default: 0.03)')
    parser.add_argument('--start',  type=float, default=0.0,    help='Starting direction z in radians (default: 0)')
    parser.add_argument('--rotations', type=float, default=None,
                        help='Stop after this many full 2π rotations (default: run continuously)')
    args = parser.parse_args()

    rclpy.init()
    node = DirectionSpinner(args.height, args.angle, args.step, args.rate, args.start, args.rotations)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
           

if __name__ == '__main__':
    main()
