#!/usr/bin/env python3
"""
Random walk obstacle controller node.

This node controls a moving obstacle in Gazebo simulation using seeded
random number generation for reproducible experiments.
"""

import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RandomWalkObstacle(Node):
    """ROS2 node that publishes random velocity commands to an obstacle."""

    def __init__(self):
        super().__init__('random_walk_obstacle')

        # Declare parameters
        self.declare_parameter('seed', 42)
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 0.8)
        self.declare_parameter('direction_change_interval', 2.5)
        self.declare_parameter('cmd_vel_topic', '/obstacle/cmd_vel')

        # Get parameters
        seed = self.get_parameter('seed').value
        self.max_linear = self.get_parameter('max_linear_velocity').value
        self.max_angular = self.get_parameter('max_angular_velocity').value
        self.interval = self.get_parameter('direction_change_interval').value
        topic = self.get_parameter('cmd_vel_topic').value

        # Initialize RNG with seed for reproducibility
        random.seed(seed)
        self.get_logger().info(f'Random walk initialized with seed: {seed}')

        # Publisher
        self.pub = self.create_publisher(Twist, topic, 10)

        # Current velocity state
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Timers
        self.create_timer(0.1, self.publish_velocity)  # 10Hz velocity publishing
        self.create_timer(self.interval, self.change_direction)

        # Set initial direction
        self.change_direction()

    def change_direction(self):
        """Randomly change movement direction."""
        self.current_linear = random.uniform(-self.max_linear, self.max_linear)
        self.current_angular = random.uniform(-self.max_angular, self.max_angular)
        self.get_logger().debug(
            f'New direction: linear={self.current_linear:.2f}, '
            f'angular={self.current_angular:.2f}'
        )

    def publish_velocity(self):
        """Publish current velocity command."""
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
