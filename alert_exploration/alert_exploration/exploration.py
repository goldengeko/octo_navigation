#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist
from mbf_msgs.action import MoveBase
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from alert_exploration.frontier_utils import *
from copy import deepcopy
import math
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus

EXPANSION_SIZE = 1
SPEED = 0.3
LOOKAHEAD_DISTANCE = 0.8
TARGET_ERROR = 0.3
TARGET_ALLOWED_TIME = 10
MAP_TRIES = 20
UNEXPLORED_EDGES_SIZE = 2
FREE_SPACE_RADIUS = 0.2


def detect_all_frontiers(occupancy_grid):
    """
    Detect all frontiers in the occupancy grid.
    A frontier is the interface between free cells (0) and unknown cells (-1).
    """
    data = np.array(occupancy_grid.data).reshape(occupancy_grid.info.height, occupancy_grid.info.width)
    height, width = data.shape
    frontiers = []
    
    # 8-directional connectivity
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    
    for i in range(1, height-1):  # Avoid edges
        for j in range(1, width-1):
            if data[i, j] == 0:  # Free cell
                is_frontier = False
                for di, dj in directions:
                    ni, nj = i + di, j + dj
                    if data[ni, nj] == -1:  # Unknown cell
                        is_frontier = True
                        break
                
                if is_frontier:
                    frontiers.append((i, j))

    print(f"Detected {len(frontiers)} frontiers")
    return frontiers


def detect_all_frontiers_with_centroids(occupancy_grid):
    """
    Detect all frontiers in the occupancy grid and calculate their centroids.
    A frontier is the interface between free cells (0) and unknown cells (-1).
    """
    data = np.array(occupancy_grid.data).reshape(occupancy_grid.info.height, occupancy_grid.info.width)
    height, width = data.shape
    frontiers = []
    frontier_points = []

    # 8-directional connectivity
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    for i in range(1, height - 1):  # Avoid edges
        for j in range(1, width - 1):
            if data[i, j] == 0:  # Free cell
                is_frontier = False
                for di, dj in directions:
                    ni, nj = i + di, j + dj
                    if data[ni, nj] == -1:  # Unknown cell
                        is_frontier = True
                        break

                if is_frontier:
                    frontier_points.append((i, j))

    # Group frontier points into clusters
    if frontier_points:
        frontier_groups = group_frontiers(frontier_points, min_group_size=1)
        for group in frontier_groups:
            centroid = np.mean(group['points'], axis=0)
            frontiers.append((centroid[0], centroid[1]))

    return frontiers


def group_frontiers(frontiers, min_group_size=3):
    """
    Group nearby frontier points together using simple clustering.
    """
    if not frontiers:
        return []
    
    frontiers_array = np.array(frontiers)
    groups = []
    visited = set()
    
    for i, frontier in enumerate(frontiers_array):
        if i in visited:
            continue
            
        # Start a new group
        current_group = [frontier]
        queue = [i]
        visited.add(i)
        
        while queue:
            current_idx = queue.pop(0)
            current_point = frontiers_array[current_idx]
            
            # Find nearby points
            for j, other_point in enumerate(frontiers_array):
                if j not in visited:
                    distance = np.linalg.norm(current_point - other_point)
                    if distance <= 2.0:  # Grouping radius
                        visited.add(j)
                        current_group.append(other_point)
                        queue.append(j)
        
        if len(current_group) >= min_group_size:
            centroid = np.mean(current_group, axis=0)
            groups.append({
                'centroid': centroid,
                'size': len(current_group),
                'points': current_group
            })
    
    return groups


def select_best_frontier(frontier_groups, robot_x, robot_y, robot_yaw, occupancy_grid, visited_areas, attempted_frontiers):
    """
    Select the best frontier based on distance, size, and heading direction.
    """
    if not frontier_groups:
        return None
    
    best_frontier = None
    best_score = -1
    
    for group in frontier_groups:
        # Convert map coordinates to world coordinates
        world_x, world_y = map_to_world_coords(
            occupancy_grid, 
            int(group['centroid'][1]), 
            int(group['centroid'][0])
        )
        
        # Calculate distance
        distance = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)
        
        # Skip if too close
        if distance < 0.4:
            continue

        is_attempted = any(
            math.sqrt((world_x - ax)**2 + (world_y - ay)**2) < 0.5
            for ax, ay in attempted_frontiers
        )
        if is_attempted:
            continue
        
        # Check if area has been visited
        is_visited = any(
            math.sqrt((world_x - vx)**2 + (world_y - vy)**2) < 1.0 
            for vx, vy in visited_areas
        )
        if is_visited:
            continue
        
        # Calculate scores
        distance_score = 1.0 / (1.0 + distance * 0.1)
        size_score = min(group['size'] / 20.0, 1.0)
        
        # Heading score
        if robot_yaw is not None:
            angle_to_frontier = math.atan2(world_y - robot_y, world_x - robot_x)
            angle_diff = abs(angle_to_frontier - robot_yaw)
            angle_diff = min(angle_diff, 2*math.pi - angle_diff)
            heading_score = 1.0 - (angle_diff / math.pi)
        else:
            heading_score = 0.5
        
        # Combined score
        score = distance_score * 0.4 + size_score * 0.3 + heading_score * 0.3
        
        if score > best_score:
            best_score = score
            best_frontier = (world_x, world_y)
    
    return best_frontier


def select_farthest_frontier(frontier_groups, robot_x, robot_y, occupancy_grid):
    """
    Select the farthest frontier based on distance from the robot.
    """
    if not frontier_groups:
        return None

    farthest_frontier = None
    max_distance = -1

    for group in frontier_groups:
        # Convert map coordinates to world coordinates
        world_x, world_y = map_to_world_coords(
            occupancy_grid, 
            int(group['centroid'][1]), 
            int(group['centroid'][0])
        )

        # Calculate distance
        distance = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)

        if distance > max_distance:
            max_distance = distance
            farthest_frontier = (world_x, world_y)

    return farthest_frontier


def find_and_select_frontier(occupancy_grid, robot_x, robot_y, robot_yaw, visited_areas, attempted_frontiers):
    """
    Detect and select the farthest frontier without strict filtering.
    """
    # Detect frontiers
    frontiers = detect_all_frontiers(occupancy_grid)
    if not frontiers:
        return None

    # Group frontiers
    frontier_groups = group_frontiers(frontiers, min_group_size=1)  # Allow smaller groups
    if not frontier_groups:
        return None

    # Directly select the farthest frontier
    farthest_frontier = select_farthest_frontier(
        frontier_groups, robot_x, robot_y, occupancy_grid
    )
    
    return farthest_frontier


def is_surrounded_by_free_space(occupancy_grid, centroid, radius=0.1):
    """
    Check if the given centroid is surrounded by free space within a radius.
    """
    data = np.array(occupancy_grid.data).reshape(occupancy_grid.info.height, occupancy_grid.info.width)
    cx, cy = int(centroid[0]), int(centroid[1])
    radius = int(radius)
    
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < data.shape[0] and 0 <= ny < data.shape[1]:
                if data[nx, ny] > 0:  # Obstacle or inflated cell
                    return False
    return True


class OpenCVFrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        
        # Subscriptions
        self.create_subscription(OccupancyGrid, '/mapUGV', self.map_callback, 10)
        self.create_subscription(Odometry, '/Go2/odometry', self.odom_callback, 10)
        
        # Publishers
        self.targetspub = self.create_publisher(MarkerArray, "/detected_markers", 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "/projected_map_1m_inflated", 10)
        
        # Action client
        self.action_client = ActionClient(self, MoveBase, '/move_base_flex/move_base')
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State variables
        self.map_data = None
        self.inflated_map = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.robot_position_updated = False
        self.in_motion = False
        
        # Exploration tracking
        self.visited_areas = []
        self.attempted_frontiers = []
        
        # Timer for exploration
        self.create_timer(2.0, self.exploration_timer_callback)
        
        self.get_logger().info("Frontier detector initialized")

    def odom_callback(self, msg):
        """Handle odometry messages."""
        try:
            # Try to transform from odom to map frame
            transform = self.tf_buffer.lookup_transform(
                'map', 
                msg.header.frame_id, 
                msg.header.stamp,
                timeout=Duration(seconds=0.1)
            )
            
            # Create pose stamped
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            
            # Transform to map frame
            transformed_pose = self.tf_buffer.transform(
                pose_stamped, 
                'map',
                timeout=Duration(seconds=0.1)
            )
            
            self.x = transformed_pose.pose.position.x
            self.y = transformed_pose.pose.position.y
            
            # Extract yaw from quaternion
            q = transformed_pose.pose.orientation
            self.yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            
        except Exception as e:
            # Fallback: use odometry directly (assume it's in map frame)
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            
            q = msg.pose.pose.orientation
            self.yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            
            self.get_logger().debug(f"Transform failed, using odometry directly: {e}")
        
        self.robot_position_updated = True

    def map_callback(self, msg):
        """Handle occupancy grid messages."""
        try:
            self.map_data = add_unexplored_edges(msg, UNEXPLORED_EDGES_SIZE)
            self.inflated_map = costmap(self.map_data, EXPANSION_SIZE)
            self.get_logger().debug("Map updated and inflated")
        except Exception as e:
            self.get_logger().error(f"Error processing map: {e}")

    def exploration_timer_callback(self):
        """Timer callback for exploration logic."""
        if not self.robot_position_updated:
            self.get_logger().debug("Waiting for robot position...")
            return
            
        if self.inflated_map is None:
            self.get_logger().debug("Waiting for map...")
            return
            
        if self.in_motion:
            self.get_logger().debug("Robot in motion, waiting...")
            return
        
        self.explore_next_frontier()

    def explore_next_frontier(self):
        """Find and navigate to the next frontier."""
        self.get_logger().info(f"Robot at: ({self.x:.2f}, {self.y:.2f}), yaw: {self.yaw:.2f}")
        
        # Combine visited areas and attempted frontiers
        all_avoided_areas = self.visited_areas + self.attempted_frontiers
        
        # Find best frontier
        best_frontier = find_and_select_frontier(
            self.inflated_map, self.x, self.y, self.yaw, all_avoided_areas, self.attempted_frontiers
        )
        
        if best_frontier is None:
            self.get_logger().info("No suitable frontiers found - exploration complete or stuck")
            return
        
        # Add to attempted frontiers
        self.attempted_frontiers.append(best_frontier)
        if len(self.attempted_frontiers) > 20:
            self.attempted_frontiers = self.attempted_frontiers[-20:]
        
        # Adjust frontier to nearest free space
        try:
            adjusted_frontier, is_adjusted = get_nearest_free_space(self.inflated_map, best_frontier)
            self.get_logger().info(f"Targeting frontier: ({adjusted_frontier[0]:.2f}, {adjusted_frontier[1]:.2f})")
            self.send_goal(adjusted_frontier)
        except Exception as e:
            self.get_logger().error(f"Error adjusting frontier: {e}")

    def send_goal(self, goal_point):
        """Send navigation goal."""
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Move base action server not available!")
            return
        
        # Create goal message
        goal_msg = MoveBase.Goal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = float(goal_point[0])
        goal_msg.target_pose.pose.position.y = float(goal_point[1])
        goal_msg.target_pose.pose.position.z = 0.0
        goal_msg.target_pose.pose.orientation.w = 1.0
        
        # Send goal
        self.in_motion = True
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info(f"Goal sent: ({goal_point[0]:.2f}, {goal_point[1]:.2f})")

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.in_motion = False
            return
        
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle goal completion."""
        try:
            result = future.result().result
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Goal succeeded!")
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn("Goal aborted by the action server.")
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn("Goal was canceled.")
            else:
                self.get_logger().warn(f"Goal finished with unexpected status: {status}")

            # Add current position to visited areas
            self.visited_areas.append((self.x, self.y))
            if len(self.visited_areas) > 15:  # Limit the size of visited areas
                self.visited_areas = self.visited_areas[-15:]

            self.in_motion = False

            # Continue exploration after a short delay
            self.create_timer(1.0, lambda: self.explore_next_frontier())
        except Exception as e:
            self.get_logger().error(f"Error in goal result callback: {e}")
            self.in_motion = False


def main(args=None):
    rclpy.init(args=args)
    detector = OpenCVFrontierDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
