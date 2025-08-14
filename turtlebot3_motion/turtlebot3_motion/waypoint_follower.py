#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from std_msgs.msg import Bool
import threading
import time
import math
from turtlebot3_msgs.msg import Goal
from action_msgs.msg import GoalStatusArray
class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Khởi tạo navigator
        self.navigator = BasicNavigator()
        
        # Khởi tạo danh sách waypoints trống
        self.goal_poses = []
        
        # Biến trạng thái
        self.is_navigating = False
        self.initial_pose_set = False
        
        # Subscribe để nhận initial pose từ RViz
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )
        
        self.goal_success_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10
        )

        self.status = 0

        # Sửa: Thay đổi loại message từ PoseStamped sang PointStamped
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
       
        # Subscribe để bắt đầu navigation
        self.start_sub = self.create_subscription(
            Goal,
            '/start_navigation',
            self.start_navigation_callback,
            10
        )
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_timer(0.5, self.get_status)
        
        # Chờ Nav2 active
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Navigation đã sẵn sàng! Hãy đặt initial pose và thêm waypoints')
    
    def status_callback(self, msg):
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        self.status = status
        self.get_logger().info(f"Status now is: {self.status}")

    def initial_pose_callback(self, msg):
        """Callback khi nhận initial pose từ RViz"""
        # Nếu đã set initial pose rồi thì bỏ qua
        if  self.initial_pose_set:
            self.get_logger().warn('initial pose has set.')
            return
        if not msg:
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.navigator.setInitialPose(pose)
        self.initial_pose_set = True
        
        # Tính góc yaw để hiển thị
        orientation = pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
        
        self.get_logger().info(
            f'Đã đặt initial pose: x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}, yaw={yaw:.2f}'
        )
  
    
    # Thêm hàm callback mới cho clicked_point
    def clicked_point_callback(self, msg):
        """Chuyển đổi clicked point thành PoseStamped"""
        # Không cho thêm waypoint khi đang điều hướng
        if self.is_navigating:
            self.get_logger().warn('Đang điều hướng, không thể thêm waypoint mới!')
            return
            
        # Chuyển đổi từ PointStamped thành PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # Hướng mặc định
        
        # Thêm vào danh sách goal poses
        self.goal_poses.append(pose)
        
        self.get_logger().info(
            f'Đã thêm waypoint {len(self.goal_poses)} (từ clicked point): '
            f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}'
        )
    
    def start_navigation_callback(self, msg:Goal):
        """Callback khi nhận lệnh bắt đầu điều hướng"""
        if not msg:
            return
            
        if not self.initial_pose_set:
            self.get_logger().error('Chưa đặt initial pose! Hãy đặt initial pose trước.')
            return
            
        if len(self.goal_poses) == 0:
            self.get_logger().warn('Không có waypoint nào để điều hướng!')
            return
            
        if self.is_navigating:
            self.get_logger().warn('Đang trong quá trình điều hướng rồi!')
            return
        pose = msg.pose
        check_navigation = msg.flag
        self.get_logger().info(f'Bắt đầu điều hướng qua {self.goal_poses[pose]} waypoint')
        
        # Tạo thread riêng để điều hướng
        if check_navigation:
            self.goal_pose_pub.publish(self.goal_poses[pose])
            self.is_navigating = True
            check_navigation = False
        
    def get_status(self):
        if not self.is_navigating:
            return
        if self.status == 4:
            self.get_logger().info("Goal reached")
            self.is_navigating = False
        elif self.status == 1 or self.status == 2:
            self.get_logger().info("Navigating!")
        elif self.status == 5:
            self.is_navigating = False
            self.get_logger().warn("Navigation cancel")
        elif self.status == 3:
            self.is_navigating = False
            self.get_logger().warn("Goal aborted")
    
    
def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    
    try:
        rclpy.spin(waypoint_follower)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()