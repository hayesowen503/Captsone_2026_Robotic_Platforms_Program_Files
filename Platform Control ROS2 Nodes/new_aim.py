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

        self.pub_ant = self.create_publisher(Vector3, '/ANT/platform/cmd_rpyz', 10)
        
        period = 0.5
        self.timer = self.create_timer(period, self.aim)
    
        self.global_x = float(x)         
        self.global_y = float(y)        
        self.global_z = float(z)        

        self.height = 0.0 
        
        # Platform geometry
        self.R_p = 0.0782  # 78.2 mm in meters

        # Local coordinates of the platform joints (at 30, 150, 270 degrees)
        self.p_l1 = [self.R_p * math.cos(math.radians(30)), self.R_p * math.sin(math.radians(30)), 0.0]
        self.p_l2 = [self.R_p * math.cos(math.radians(150)), self.R_p * math.sin(math.radians(150)), 0.0]
        self.p_l3 = [self.R_p * math.cos(math.radians(270)), self.R_p * math.sin(math.radians(270)), 0.0]

    def get_rotation_matrix(self, roll, pitch, yaw):
        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)

        r11 = cy * cz
        r12 = cz * sx * sy - cx * sz
        r13 = cx * cz * sy + sx * sz
        r21 = cy * sz
        r22 = cx * cz + sx * sy * sz
        r23 = cx * sy * sz - cz * sx
        r31 = -sy
        r32 = cy * sx
        r33 = cx * cy

        return [[r11, r12, r13],
                [r21, r22, r23],
                [r31, r32, r33]]

    def multiply_matrix_vector(self, M, v):
        return [
            M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
            M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
            M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]
        ]

    def get_parasitic_translation(self, roll, pitch):
        def yaw_error(yaw):
            R = self.get_rotation_matrix(roll, pitch, yaw)
            d1 = self.multiply_matrix_vector(R, self.p_l1)
            d2 = self.multiply_matrix_vector(R, self.p_l2)
            d3 = self.multiply_matrix_vector(R, self.p_l3)

            dx1, dy1 = d1[0], d1[1]
            dx2, dy2 = d2[0], d2[1]
            dx3 = d3[0]

            target_dx3 = 0.5 * (dx1 + dx2) - (math.sqrt(3) / 2.0) * (dy1 - dy2)
            return dx3 - target_dx3

        yaw0 = 0.0
        yaw1 = 0.001
        for _ in range(15):
            e0 = yaw_error(yaw0)
            e1 = yaw_error(yaw1)
            
            if abs(e1 - e0) < 1e-9:
                break
                
            yaw_next = yaw1 - e1 * (yaw1 - yaw0) / (e1 - e0)
            yaw0 = yaw1
            yaw1 = yaw_next

        final_yaw = yaw1

        R = self.get_rotation_matrix(roll, pitch, final_yaw)
        d1 = self.multiply_matrix_vector(R, self.p_l1)
        d2 = self.multiply_matrix_vector(R, self.p_l2)
        d3 = self.multiply_matrix_vector(R, self.p_l3)

        x_p = -d3[0]
        y_p = (0.5 * (d1[0] - d2[0]) - (math.sqrt(3) / 2.0) * (d1[1] + d2[1])) / math.sqrt(3)
        
        return x_p, y_p

    def vector_to_roll_pitch(self, vx, vy, vz):
        pitch = math.atan2(vx, vz)
        roll = math.atan2(-vy, math.sqrt(vx**2 + vz**2))
        return roll, pitch

    def aim(self):
        eff_x, eff_y, eff_z = self.global_x, self.global_y, self.global_z
        current_roll, current_pitch = 0.0, 0.0

        # 1. Iteratively solve the coupled aiming/translation problem using Euler math
        for _ in range(5):
            current_roll, current_pitch = self.vector_to_roll_pitch(eff_x, eff_y, eff_z)
            dx, dy = self.get_parasitic_translation(current_roll, current_pitch)
            eff_x = self.global_x - dx
            eff_y = self.global_y - dy

        # 2. Convert the final, compensated effective target into Spherical Coordinates
        r = math.sqrt(eff_x**2 + eff_y**2)
        
        # Tilt angle from vertical
        final_tilt = math.atan2(r, eff_z)
        
        # Direction (+x is 0)
        final_dir = math.atan2(eff_y, eff_x)
        if final_dir < 0:
            final_dir += 6.2832  # 2 * pi

        # 3. Publish in the specific format required: x=height, y=tilt, z=direction
        msg = Vector3(x=self.height, y=final_tilt, z=final_dir)
        self.pub_ant.publish(msg)
        
        self.get_logger().info(
            f"Aim Target: ({self.global_x}, {self.global_y}, {self.global_z}) | "
            f"Compensated Tilt: {final_tilt:.4f} rad, Dir: {final_dir:.4f} rad"
        )


def main():
    parser = argparse.ArgumentParser(description="Publish height, tilt, and direction to aim at Cartesian coordinates")
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
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


if __name__ == "__main__":
    main()
