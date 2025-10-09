#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Pose
from mbf_msgs.action import MoveBase
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from alert_exploration.frontier_utils import * 
from copy import deepcopy

EXPANSION_SIZE = 2
UNEXPLORED_EDGES_SIZE = 6
FREE_SPACE_RADIUS = 0.5
DISTANCE_THRESHOLD = 0.75 
ROBOT_BASE_FRAME = 'base_link'
MAP_FRAME = 'map'


def detect_frontier_cells(matrix):
    frontier_matrix = np.copy(matrix)
    rows, cols = matrix.shape
    for i in range(rows):
        for j in range(cols):
            if matrix[i, j] == 0:
                is_frontier = False
                if (i > 0 and matrix[i - 1, j] == -1) or \
                   (i < rows - 1 and matrix[i + 1, j] == -1) or \
                   (j > 0 and matrix[i, j - 1] == -1) or \
                   (j < cols - 1 and matrix[i, j + 1] == -1):
                    is_frontier = True
                
                if is_frontier:
                    frontier_matrix[i, j] = 2 
    return frontier_matrix

def group_frontiers_dfs(matrix):
    groups = {}
    visited = set()
    rows, cols = matrix.shape
    
    for r in range(rows):
        for c in range(cols):
            if matrix[r, c] == 2 and (r, c) not in visited:
                group_id = len(groups) + 1
                groups[group_id] = []
                stack = [(r, c)]
                
                while stack:
                    curr_r, curr_c = stack.pop()
                    if (curr_r, curr_c) in visited or matrix[curr_r, curr_c] != 2:
                        continue
                    
                    visited.add((curr_r, curr_c))
                    groups[group_id].append((curr_r, curr_c))
                    
                    for dr in [-1, 0, 1]:
                        for dc in [-1, 0, 1]:
                            if dr == 0 and dc == 0: continue
                            nr, nc = curr_r + dr, curr_c + dc
                            if 0 <= nr < rows and 0 <= nc < cols:
                                stack.append((nr, nc))
    return groups

def get_top_frontier_groups(groups, min_size=5, top_n=5):
    large_groups = {k: v for k, v in groups.items() if len(v) >= min_size}
    sorted_groups = sorted(large_groups.items(), key=lambda item: len(item[1]), reverse=True)
    return sorted_groups[:top_n]

def get_frontiers(map_msg):
    data = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)
    frontier_matrix = detect_frontier_cells(data)
    groups = group_frontiers_dfs(frontier_matrix)
    return get_top_frontier_groups(groups)

def calculate_centroid(points):
    x_coords = [p[1] for p in points]
    y_coords = [p[0] for p in points]
    return (int(np.mean(x_coords)), int(np.mean(y_coords)))


class MBFFrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('mbf_frontier_exploration_node')
        
        self.create_subscription(OccupancyGrid, '/mapUGV', self.map_callback, 1)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "/mapUGV_inflated", 1)
        
        self.action_client = ActionClient(self, MoveBase, '/move_base_flex/move_base')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.create_timer(1.0, self.exploration_timer_callback)
        
        self.inflated_map = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.is_navigating = False
        
        self.get_logger().info("Frontier Exploration Node has started.")

    def map_callback(self, msg):
        map_with_edges = add_unexplored_edges(msg, UNEXPLORED_EDGES_SIZE)
        self.inflated_map = costmap(map_with_edges, EXPANSION_SIZE)

    def update_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_BASE_FRAME, rclpy.time.Time())
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform {ROBOT_BASE_FRAME} to {MAP_FRAME}: {e}")
            return False

    def send_navigation_goal(self, goal_point):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server '/move_base_flex/move_base' not available.")
            return

        goal_msg = MoveBase.Goal()
        goal_msg.target_pose.header.frame_id = MAP_FRAME
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = float(goal_point[0])
        goal_msg.target_pose.pose.position.y = float(goal_point[1])
        goal_msg.target_pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal: ({goal_point[0]:.2f}, {goal_point[1]:.2f})")
        self.is_navigating = True
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by the action server.')
            self.is_navigating = False
            return
        
        self.get_logger().info('Goal accepted. Waiting for result.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Navigation succeeded: {result.message}')
        else:
            self.get_logger().warn(f'Navigation failed with status {status}: {result.message}')
        self.is_navigating = False

    def exploration_timer_callback(self):
        if self.is_navigating:
            return
        if self.inflated_map is None:
            self.get_logger().warn("Waiting for map data...")
            return
        if not self.update_robot_pose():
            return

        current_map = add_free_space_at_robot(self.inflated_map, self.robot_x, self.robot_y, FREE_SPACE_RADIUS)
        frontier_groups = get_frontiers(current_map)
        
        if not frontier_groups:
            self.get_logger().warn("No frontiers found. Exploration might be complete.")
            return

        centroids_map = [calculate_centroid(group[1]) for group in frontier_groups]
        frontiers_world = [map_to_world_coords(current_map, c[0], c[1]) for c in centroids_map]

        distances = [np.hypot(fw[0] - self.robot_x, fw[1] - self.robot_y) for fw in frontiers_world]
        
        valid_frontiers = []
        for dist, world_coords in zip(distances, frontiers_world):
            if dist > DISTANCE_THRESHOLD:
                valid_frontiers.append((dist, world_coords))
        
        if not valid_frontiers:
            self.get_logger().warn(f"Found {len(frontiers_world)} frontiers, but all are closer than {DISTANCE_THRESHOLD}m.")
            return

        valid_frontiers.sort(key=lambda x: x[0])
        closest_frontier = valid_frontiers[0][1]

        target_point, _ = get_nearest_free_space(current_map, closest_frontier)

        self.get_logger().info(f"Best frontier is {valid_frontiers[0][0]:.2f}m away. Sending goal to reachable point: {target_point}")
        self.send_navigation_goal(target_point)


def main(args=None):
    rclpy.init(args=args)
    node = MBFFrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
