#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookupException
from queue import PriorityQueue
class GraphNode:
    def __init__(self, x, y, cost=0, heuristic=0, prev=None):
        self.x = x
        self.y = y
        self.heuristic = heuristic
        self.cost = cost
        self.prev = prev
    
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pb = self.create_publisher(Path, 'a_star/path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'a_star/visited_map', 10)

        self.map_ = None
        self.visited_map_ = OccupancyGrid()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header.frame_id = map_msg.header.frame_id
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.width * map_msg.info.height)

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().error("Map not received yet.")
            return
        
        self.visited_map_.data = [-1] * (self.map_.info.width * self.map_.info.height)
        target_frame = self.map_.header.frame_id
        source_frame = "base_footprint"
        if not self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
            self.get_logger().error(f"Transform from {target_frame} to {source_frame} not available.")
            return
        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return
        
        map_to_base_pose = Pose()
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
        map_to_base_pose.position.z = map_to_base_tf.transform.translation.z
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation

        path = self.plan(map_to_base_pose, pose.pose)
        if path.poses:
            self.get_logger().info(f"Shortest path found")
            self.path_pb.publish(path)
        else:
            self.get_logger().warn("No path found to the goal.")

    def plan(self, start, goal):
        explore_directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]
        pending_nodes = PriorityQueue()
        visited_nodes = set()
        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)
        start_node.heuristic = self.manhattan_distance(start_node, goal_node)
        pending_nodes.put(start_node)
        visited_nodes.add(start_node)

        goal_node = self.world_to_grid(goal)
        current_node = None
        while not pending_nodes.empty() and rclpy.ok():
            current_node = pending_nodes.get()

            if current_node == goal_node:
                break

            for dir_x, dir_y in explore_directions:
                new_node: GraphNode = current_node + (dir_x, dir_y)
                if new_node not in visited_nodes and self.pose_to_map(new_node) and self.map_.data[self.pose_to_cell(new_node)] == 0:
                    new_node.cost = current_node.cost + 1
                    new_node.heuristic = self.manhattan_distance(new_node, goal_node)
                    new_node.prev = current_node
                    pending_nodes.put(new_node)
                    visited_nodes.add(new_node)
         
            self.visited_map_.data[self.pose_to_cell(current_node)] = 10
            self.map_pub.publish(self.visited_map_)

        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        path.poses = []
        if current_node is None or current_node != goal_node:
            return path
        while current_node and current_node.prev and rclpy.ok():
            last_pose:Pose = self.grid_to_world(current_node)
            last_pose_stamped = PoseStamped()
            last_pose_stamped.header.frame_id = self.map_.header.frame_id
            last_pose_stamped.pose = last_pose
            path.poses.append(last_pose_stamped)
            current_node = current_node.prev

        path.poses.reverse()
        return path
    
    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x
 
    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def manhattan_distance(self, node:GraphNode, goal:GraphNode):
        return abs(node.x - goal.x) + abs(node.y - goal.y)
def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
