#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
# Fix import here - add PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
import os

from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool
import math

class WayPointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.navigator = BasicNavigator()
        self.waypoints = []
        self.is_collecting = True
        self.is_initial_pose_set = False

        # Fix message type here
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.waypoint_callback, 10
        )

        self.start_sub = self.create_subscription(
            Bool, '/start_navigation', self.start_callback, 10
        )

        self.navigator.waitUntilNav2Active()

    # Update callback to use PoseWithCovarianceStamped
    def initial_pose_callback(self, msg):
        # Only set initial pose if it hasn't been set already
        if not self.is_initial_pose_set:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.navigator.setInitialPose(pose)
            self.is_initial_pose_set = True
            yaw = math.atan2(2.0*msg.pose.pose.orientation.z * msg.pose.pose.orientation.w, 1.0 - 2.0*msg.pose.pose.orientation.z**2)

            self.get_logger().info(
                f"Set initial pose: "
                f"x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, yaw={yaw:.2f}"
            )
        else:
            self.get_logger().warn("Initial pose already set. To set a new initial pose, restart the waypoint_follower node.")

    def waypoint_callback(self, msg:PoseStamped):
        if self.is_collecting:
            self.waypoints.append(msg)
            yaw = math.atan2(2.0*msg.pose.orientation.z * msg.pose.orientation.w, 1.0 - 2.0*msg.pose.orientation.z**2)

            self.get_logger().info(
                f"Added waypoint {len(self.waypoints)}: "
                f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, yaw={yaw:.2f}"
            )

    def start_callback(self, msg):
        if msg.data and self.is_collecting:
            self.is_collecting = False
            self.navigate_waypoint()
    
    def navigate_waypoint(self):
        if not self.waypoints:
            self.get_logger().warn("No waypoints collected")
            self.is_collecting = True
            return
        
        if not self.is_initial_pose_set:
            self.get_logger().warn("Initial pose not set! Please set initial pose first.")
            self.is_collecting = True
            return

        self.get_logger().info(f"Starting navigation through {len(self.waypoints)} waypoints...")

        for i, goal_pose in enumerate(self.waypoints):
            yaw = math.atan2(2.0*goal_pose.pose.orientation.z * goal_pose.pose.orientation.w, 1.0 - 2.0*goal_pose.pose.orientation.z**2)

            self.get_logger().info(
                f"Navigating to waypoint {i+1}/{len(self.waypoints)}: "
                f"x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}, yaw={yaw:.2f}"
            )

            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback and Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.get_logger().warn("Navigator time out, canceling task ...")
                    self.navigator.cancelTask()
                    break
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Waypoint {i+1} reached successfully!")
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f"Navigation to waypoint {i+1} was canceled")
                break
            elif result == TaskResult.FAILED:
                self.get_logger().warn(f"Failed to reach waypoint {i+1}")
                break

        self.get_logger().info("Finished navigating all waypoints!")
        self.waypoints = []
        self.is_collecting = True

def main():
    rclpy.init()
    node = WayPointFollower()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

