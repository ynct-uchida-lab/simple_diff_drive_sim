#!/usr/bin/env python3
"""
Navigation test script for gng_local_planner_rs experiments.

This script sends a goal pose and monitors for goal_reached,
outputting test results in JSON format for automated testing.
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray


class NavigationTest(Node):
    """ROS2 node for testing navigation to a goal."""

    def __init__(self, goal_x: float, goal_y: float, timeout: float):
        super().__init__('navigation_test')

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.timeout = timeout

        # State
        self.goal_reached = False
        self.start_time = None
        self.framerate_samples = []

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscribers
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10
        )
        self.framerate_sub = self.create_subscription(
            MarkerArray, '/ais_gng_framerate', self.framerate_callback, 10
        )

        # Timer to publish goal after a short delay
        self.create_timer(1.0, self.publish_goal_once)
        self.goal_published = False

    def publish_goal_once(self):
        """Publish goal pose once."""
        if self.goal_published:
            return

        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)
        self.start_time = time.time()
        self.goal_published = True
        self.get_logger().info(
            f'Goal published: ({self.goal_x}, {self.goal_y})'
        )

    def goal_reached_callback(self, msg: Bool):
        """Handle goal_reached message."""
        if msg.data and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info('Goal reached signal received!')

    def framerate_callback(self, msg: MarkerArray):
        """Extract framerate from marker text."""
        for marker in msg.markers:
            if marker.ns == 'ais_gng_framerate' and marker.text:
                try:
                    # Parse "AiS-GNG\nHz: XX.X\nms: X.XX\nsteps: NNNN"
                    lines = marker.text.split('\n')
                    for line in lines:
                        if line.startswith('Hz:'):
                            hz = float(line.split(':')[1].strip())
                            self.framerate_samples.append(hz)
                            break
                except (ValueError, IndexError):
                    pass

    def run_test(self) -> dict:
        """Run the navigation test and return results."""
        rate = self.create_rate(10)  # 10Hz check

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.goal_reached:
                elapsed = time.time() - self.start_time if self.start_time else 0
                avg_hz = (
                    sum(self.framerate_samples) / len(self.framerate_samples)
                    if self.framerate_samples
                    else 0.0
                )
                return {
                    'success': True,
                    'time_sec': round(elapsed, 2),
                    'avg_hz': round(avg_hz, 1),
                    'samples': len(self.framerate_samples),
                }

            if self.start_time and (time.time() - self.start_time) > self.timeout:
                avg_hz = (
                    sum(self.framerate_samples) / len(self.framerate_samples)
                    if self.framerate_samples
                    else 0.0
                )
                return {
                    'success': False,
                    'time_sec': self.timeout,
                    'avg_hz': round(avg_hz, 1),
                    'samples': len(self.framerate_samples),
                    'error': 'timeout',
                }

        return {'success': False, 'error': 'interrupted'}


def main(args=None):
    parser = argparse.ArgumentParser(description='Navigation test for gng_local_planner')
    parser.add_argument('--goal-x', type=float, default=2.0, help='Goal X coordinate')
    parser.add_argument('--goal-y', type=float, default=0.0, help='Goal Y coordinate')
    parser.add_argument('--timeout', type=float, default=60.0, help='Timeout in seconds')

    # Parse known args to allow ROS2 args to pass through
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = NavigationTest(parsed_args.goal_x, parsed_args.goal_y, parsed_args.timeout)

    try:
        result = node.run_test()
        print(json.dumps(result))
        sys.exit(0 if result.get('success') else 1)
    except KeyboardInterrupt:
        print(json.dumps({'success': False, 'error': 'interrupted'}))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
