#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist, TransformStamped
from sensor_msgs.msg import JointState, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TestNode(Node):
    def __init__(self):
        super().__init__('test_rbpf')
        
        print("Node created")
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        print("QoS profile created")
        
        # Test subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )
        
        print("Scan subscriber created")
        
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            qos_profile
        )
        
        print("Joint states subscriber created")
        
        # Test publishers
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            'map',
            qos_profile
        )
        
        print("Map publisher created")
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            qos_profile
        )
        
        print("Odom publisher created")
        
        print("Test node initialized successfully!")

    def scan_callback(self, msg):
        print(f"Received scan with {len(msg.ranges)} points")
        
    def joint_states_callback(self, msg):
        print(f"Received joint states")

def main():
    rclpy.init()
    
    try:
        node = TestNode()
        print("Spinning node...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Shutting down...")
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
