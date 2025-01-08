#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for robot's position and velocity (could be a custom message or simple message)
        self.pose_velocity_pub = self.create_publisher(Twist, '/robot_pose_velocity', 10)
        
        # Subscriber for robot's odometry (position feedback)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Timer to periodically publish velocity commands
        self.timer = self.create_timer(1.0, self.move_robot)
        
        # Initialize the velocity command
        self.cmd_vel = Twist()
        
        # Initialize position and velocity variables
        self.position_x = 0.0
        self.position_y = 0.0
        self.linear_velocity = 0.0

    def move_robot(self):
        # Set linear and angular velocities for robot movement
        self.cmd_vel.linear.x = 0.5  # Move forward at 0.5 m/s
        self.cmd_vel.angular.z = 0.1  # Rotate with a small angular velocity

        # Publish the velocity command
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.get_logger().info(f"Publishing velocity: linear.x={self.cmd_vel.linear.x}, angular.z={self.cmd_vel.angular.z}")

    def odom_callback(self, msg):
        # Here, we receive and print the robot's position from the Odometry message
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear  # Getting linear velocity from Odometry message

        self.position_x = position.x
        self.position_y = position.y
        self.linear_velocity = velocity.x  # Assuming linear velocity along the x-axis

        # Log the position and velocity
        self.get_logger().info(f"Position: x={self.position_x}, y={self.position_y}")
        self.get_logger().info(f"Linear Velocity: {self.linear_velocity}")

        # Create a message to publish the robot's position and velocity
        pose_velocity_msg = Twist()
        pose_velocity_msg.linear.x = self.linear_velocity  # Publish the linear velocity
        # You can also add angular velocity if needed
        pose_velocity_msg.angular.z = 0.0  # Assuming no angular velocity for simplicity

        # Publish the position and velocity
        self.pose_velocity_pub.publish(pose_velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()

    rclpy.spin(robot_mover)

    robot_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

