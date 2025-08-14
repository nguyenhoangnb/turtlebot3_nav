#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist, TransformStamped
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from typing import List, Optional, Tuple

from tf2_ros import TransformBroadcaster
import numpy as np 
import math
import copy


from sensor_properties import *
from grid_mapper import *
from particle import *
from cloud_alightment import *

def deg2rad(d):
    return d * math.pi / 180.0

class DiffDrive:
    def __init__(self, x: float, 
                 y: float, 
                 theta: float = 0.0,
                 wheel_base: float = 1.0, 
                 wheel_radius: float = 1.0):
        self.pose = Transform2D(x, y, theta)
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.prev_left = 0.0
        self.prev_right = 0.0
    
    def update_odometry(self, left: float, right: float):
        """Update pose using differential drive kinematics"""
        dl = left - self.prev_left
        dr = right - self.prev_right

        self.prev_left = left
        self.prev_right = right
        
        d_theta = self.wheel_radius * (dr - dl) / self.wheel_base
        d_s = self.wheel_radius * (dr + dl) / 2.0  
        self.last_ds = d_s
        self.last_dtheta = d_theta
        
        if abs(d_theta) < 1e-10:
            self.pose.x += d_s * math.cos(self.pose.theta)
            self.pose.y += d_s * math.sin(self.pose.theta)
        else:
            radius = d_s / d_theta
            
            self.pose.x += radius * (math.sin(self.pose.theta + d_theta) - math.sin(self.pose.theta))
            self.pose.y += radius * (-math.cos(self.pose.theta + d_theta) + math.cos(self.pose.theta))
            
        self.pose.theta = normalize_angle_pi(self.pose.theta + d_theta)

    def get_twist(self)->Tuple[float, float]:
        return 0.0, 0.0
class RBPFNode(Node):
    def __init__(self):
        super().__init__('rbpf_slam')

        self.declare_node_parameters()

        self.get_parameters()

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.pf_left_wheel_pos = 0.0
        self.pf_right_wheel_pos = 0.0
        self.wheel_odom_flag = False
        self.scan_data = []
        self.scan_update = False
        self.gazebo_robot_pose = PoseStamped()

        self.odom_pose = Transform2D()
        self.robot_pose = Transform2D()

        self.drive = DiffDrive(0.0, 0.0, 0.0, self.wheel_base, self.wheel_radius)
        self.pf_drive = DiffDrive(0.0, 0.0, 0.0, self.wheel_base, self.wheel_radius)

        self.setup_slam_componments()
        self.setup_ros_interface()
        
        self.odom_path = Path()
        self.slam_path = Path()
        self.gazebo_path = Path()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("RBPF SLAM node initialized successfully")

    def declare_node_parameters(self):
        
        # Frame Id
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('body_frame_id', 'base_footprint')

        # Joint name
        self.declare_parameter('wheel_left_joint', 'wheel_left_joint')
        self.declare_parameter('wheel_right_joint', 'wheel_right_joint')

        #Robot parameters
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('wheel_radius', 0.1)

        #Lidar parameters
        self.declare_parameter('beam_min', -90.0)
        self.declare_parameter('beam_max', 90.0)
        self.declare_parameter('beam_delta', 1.0)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)

        #Laser model parameters
        self.declare_parameter('z_hit', 0.8)
        self.declare_parameter('z_short', 0.1)
        self.declare_parameter('z_max', 0.05)
        self.declare_parameter('z_rand', 0.05)
        self.declare_parameter('sigma_hit', 0.1)
        
        # Particle filter parameters
        self.declare_parameter('num_particles', 100)
        self.declare_parameter('num_samples_mode', 10)

        # Motion model noise parameters
        self.declare_parameter('srr', 0.1)
        self.declare_parameter('srt', 0.1)
        self.declare_parameter('str', 0.1)
        self.declare_parameter('stt', 0.1)
        self.declare_parameter('motion_nosie_theta', 0.01)
        self.declare_parameter('motion_nosie_x', 0.01)
        self.declare_parameter('motion_nosie_y', 0.01)

        # Sample range parameters
        self.declare_parameter('sample_range_theta', 0.1)
        self.declare_parameter('sample_range_x', 0.1)
        self.declare_parameter('sample_range_y', 0.1)

        #Likelihood parameters
        self.declare_parameter('scan_likelihood_min', 0.01)
        self.declare_parameter('scan_likelihood_max', 0.99)
        self.declare_parameter('pose_likelihood_min', 0.01)
        self.declare_parameter('pose_likelihood_max', 0.99)

        #Map parameters

        self.declare_parameter('map_min', -10.0)
        self.declare_parameter('map_max', 10.0)
        self.declare_parameter('map_resolution', 0.05)

    def get_parameters(self):
        # Frame Id
        self.map_frame_id = self.get_parameter('map_frame_id').value    
        self.odom_frame_id = self.get_parameter('odom_frame_id').value  
        self.body_frame_id = self.get_parameter('body_frame_id').value  

        # Joint name
        self.wheel_left_joint = self.get_parameter('wheel_left_joint').value
        self.wheel_right_joint = self.get_parameter('wheel_right_joint').value

        #Robot parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        #Lidar parameters
        self.beam_min = self.get_parameter('beam_min').value
        self.beam_max = self.get_parameter('beam_max').value
        self.beam_delta = self.get_parameter('beam_delta').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        #Laser model parameters
        self.z_hit = self.get_parameter('z_hit').value
        self.z_short = self.get_parameter('z_short').value
        self.z_max = self.get_parameter('z_max').value
        self.z_rand = self.get_parameter('z_rand').value
        self.sigma_hit = self.get_parameter('sigma_hit').value
        
        # Particle filter parameters
        self.num_particles = self.get_parameter('num_particles').value
        self.num_samples_mode = self.get_parameter('num_samples_mode').value

        # Motion model noise parameters
        self.srr = self.get_parameter('srr').value
        self.srt = self.get_parameter('srt').value
        self.str = self.get_parameter('str').value
        self.stt = self.get_parameter('stt').value
        self.motion_nosie_theta = self.get_parameter('motion_nosie_theta').value
        self.motion_nosie_x = self.get_parameter('motion_nosie_x').value
        self.motion_nosie_y = self.get_parameter('motion_nosie_y').value

        # Sample range parameters
        self.sample_range_theta = self.get_parameter('sample_range_theta').value
        self.sample_range_x = self.get_parameter('sample_range_x').value
        self.sample_range_y = self.get_parameter('sample_range_y').value

        #Likelihood parameters
        self.scan_likelihood_min = self.get_parameter('scan_likelihood_min').value
        self.scan_likelihood_max = self.get_parameter('scan_likelihood_max').value
        self.pose_likelihood_min = self.get_parameter('pose_likelihood_min').value
        self.pose_likelihood_max = self.get_parameter('pose_likelihood_max').value

        #Map parameters
        self.map_min = self.get_parameter('map_min').value
        self.map_max = self.get_parameter('map_max').value
        self.map_resolution = self.get_parameter('map_resolution').value

    def setup_slam_componments(self):
        """Initialize SLAM components"""
        # Transform robot to lidar (identity for now)
        self.Trs = Transform2D()
        
        # Lidar properties  
        self.laser_props = LaserProperties(
            beam_min=deg2rad(self.beam_min),
            beam_max=deg2rad(self.beam_max), 
            beam_delta=deg2rad(self.beam_delta),
            range_min=self.range_min,
            range_max=self.range_max,
            z_hit=self.z_hit,
            z_short=self.z_short,
            z_max=self.z_max,
            z_rand=self.z_rand,
            sigma_hit=self.sigma_hit
        )
        
        # Grid mapper
        self.grid_mapper = GridMapper(
            resolution=self.map_resolution,
            xmin=self.map_min,
            ymin=self.map_min,
            xmax=self.map_max,
            ymax=self.map_max,
            props=self.laser_props,
            Trs=self.Trs
        )
        
        # Scan alignment for ICP
        self.scan_alignment = ScanAlightment(self.laser_props, self.Trs)
        
        # Particle filter
        self.particle_filter = ParticleFilter(
            num_particles=self.num_particles,
            k=self.num_samples_mode,
            srr=self.srr,
            srt=self.srt,
            str_=self.str,
            stt=self.stt,
            motion_noise_theta=self.motion_nosie_theta,
            motion_noise_x=self.motion_nosie_x,
            motion_noise_y=self.motion_nosie_y,
            sample_range_theta=self.sample_range_theta,
            sample_range_x=self.sample_range_x,
            sample_range_y=self.sample_range_y,
            scan_likelihood_min=self.scan_likelihood_min,
            scan_likelihood_max=self.scan_likelihood_max,
            pose_likelihood_min=self.pose_likelihood_min,
            pose_likelihood_max=self.pose_likelihood_max,
            scan_matcher=self.scan_alignment,
            pose=self.robot_pose,
            mapper=self.grid_mapper
        )

    def setup_ros_interface(self):
        """Setup ROS2 publishers and subscribers"""
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )
        
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            qos_profile
        )
        
        # Publishers
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            'map',
            qos_profile
        )
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            qos_profile
        )
        
        self.slam_path_publisher = self.create_publisher(
            Path,
            'slam_path',
            qos_profile
        )
        
        self.odom_path_publisher = self.create_publisher(
            Path,
            'odom_path',
            qos_profile
        )
        
        # Timer for main processing loop
        self.timer = self.create_timer(0.1, self.slam_loop)  # 10Hz

    def scan_callback(self, msg: LaserScan):
        """Handle incoming laser scan data"""
        self.scan_data = list(msg.ranges)
        self.scan_update = True

    def joint_states_callback(self, msg: JointState):
        """Handle incoming joint state data"""
        try:
            left_idx = msg.name.index(self.wheel_left_joint)
            right_idx = msg.name.index(self.wheel_right_joint)
            
            self.left_wheel_pos = msg.position[left_idx]
            self.right_wheel_pos = msg.position[right_idx]
            self.wheel_odom_flag = True
            
        except ValueError as e:
            self.get_logger().warn(f"Joint not found: {e}")

    def euler_from_quaternion(self, quat):
        """Convert quaternion to euler angles (custom implementation for NumPy 2.0 compatibility)"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert euler angles to quaternion (custom implementation for NumPy 2.0 compatibility)"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat

    def slam_loop(self):
        """Main SLAM processing loop"""
        if not self.wheel_odom_flag:
            return
            
        # Update odometry
        self.drive.update_odometry(self.left_wheel_pos, self.right_wheel_pos)
        
        # Store current pose for particle filter
        self.pf_left_wheel_pos = self.left_wheel_pos
        self.pf_right_wheel_pos = self.right_wheel_pos
        
        # SLAM update with scan data
        if self.scan_update and len(self.scan_data) > 0:
            # Update particle filter odometry
            self.pf_drive.update_odometry(self.pf_left_wheel_pos, self.pf_right_wheel_pos)
            
            # Get twist for motion model
            linear_vel, angular_vel = self.pf_drive.get_twist()
            twist = Twist2D(vx=linear_vel, w=angular_vel)
            
            # Current and previous poses for likelihood model
            cur_pose = Pose(x=self.pf_drive.pose.x, y=self.pf_drive.pose.y, theta=self.pf_drive.pose.theta)
            prev_pose = Pose(x=self.odom_pose.x, y=self.odom_pose.y, theta=self.odom_pose.theta)
            
            # Run particle filter SLAM
            self.particle_filter.SLAM(self.scan_data, twist, cur_pose, prev_pose)
            
            # Update stored pose
            self.odom_pose = Transform2D(cur_pose.x, cur_pose.y, cur_pose.theta)
            
            # Get robot state from best particle
            self.robot_pose = self.particle_filter.get_robot_state()
            
            # Publish map
            self.publish_map()
            
            self.scan_update = False
        
        # Publish odometry and transforms
        self.publish_odometry()
        self.publish_transforms()
        self.publish_paths()
        
        self.wheel_odom_flag = False

    def publish_map(self):
        """Publish occupancy grid map"""
        try:
            # Get map from best particle
            map_data = self.particle_filter.new_map()
            
            # Create OccupancyGrid message
            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = self.map_frame_id
            
            # Map metadata
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = int((self.map_max - self.map_min) / self.map_resolution)
            map_msg.info.height = int((self.map_max - self.map_min) / self.map_resolution)
            
            # Map origin
            map_msg.info.origin.position.x = self.map_min
            map_msg.info.origin.position.y = self.map_min
            map_msg.info.origin.position.z = 0.0
            map_msg.info.origin.orientation.w = 1.0
            
            # Map data
            map_msg.data = map_data
            
            self.map_publisher.publish(map_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to publish map: {e}")

    def publish_odometry(self):
        """Publish odometry information"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.body_frame_id
        
        # Position
        odom_msg.pose.pose.position.x = self.drive.pose.x
        odom_msg.pose.pose.position.y = self.drive.pose.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        odom_msg.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.drive.pose.theta)
        
        # Velocity (for now set to zero, can be computed from wheel velocities)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        self.odom_publisher.publish(odom_msg)

    def publish_transforms(self):
        """Publish TF transforms"""
        from geometry_msgs.msg import TransformStamped
        
        current_time = self.get_clock().now().to_msg()
        
        # Map to odom transform
        # Get transform from odometry pose
        odom_transform = Transform2D(self.drive.pose.x, self.drive.pose.y, self.drive.pose.theta)
        
        # Calculate map to odom transform
        # T_map_odom = T_map_robot * T_robot_odom
        # T_robot_odom = T_odom_robot^-1
        odom_pose_inv = self.drive.pose.inverse()
        map_to_odom = self.robot_pose.compose(odom_pose_inv)
        
        # Broadcast map to odom
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = self.map_frame_id
        t.child_frame_id = self.odom_frame_id
        t.transform.translation.x = map_to_odom.x
        t.transform.translation.y = map_to_odom.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.quaternion_from_euler(0, 0, map_to_odom.theta)
        
        self.tf_broadcaster.sendTransform(t)
        
        # Broadcast odom to base_footprint
        t2 = TransformStamped()
        t2.header.stamp = current_time
        t2.header.frame_id = self.odom_frame_id
        t2.child_frame_id = self.body_frame_id
        t2.transform.translation.x = self.drive.pose.x
        t2.transform.translation.y = self.drive.pose.y
        t2.transform.translation.z = 0.0
        t2.transform.rotation = self.quaternion_from_euler(0, 0, self.drive.pose.theta)
        
        self.tf_broadcaster.sendTransform(t2)

    def publish_paths(self):
        """Publish robot paths"""
        current_time = self.get_clock().now().to_msg()
        
        # SLAM path
        slam_pose = PoseStamped()
        slam_pose.header.stamp = current_time
        slam_pose.header.frame_id = self.map_frame_id
        slam_pose.pose.position.x = self.robot_pose.x
        slam_pose.pose.position.y = self.robot_pose.y
        slam_pose.pose.position.z = 0.0
        slam_pose.pose.orientation = self.quaternion_from_euler(0, 0, self.robot_pose.theta)
        
        self.slam_path.header.stamp = current_time
        self.slam_path.header.frame_id = self.map_frame_id
        self.slam_path.poses.append(slam_pose)
        
        self.slam_path_publisher.publish(self.slam_path)
        
        # Odometry path
        odom_pose = PoseStamped()
        odom_pose.header.stamp = current_time
        odom_pose.header.frame_id = self.map_frame_id
        odom_pose.pose.position.x = self.drive.pose.x
        odom_pose.pose.position.y = self.drive.pose.y
        odom_pose.pose.position.z = 0.0
        odom_pose.pose.orientation = self.quaternion_from_euler(0, 0, self.drive.pose.theta)
        
        self.odom_path.header.stamp = current_time
        self.odom_path.header.frame_id = self.map_frame_id
        self.odom_path.poses.append(odom_pose)
        
        self.odom_path_publisher.publish(self.odom_path)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = RBPFNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

