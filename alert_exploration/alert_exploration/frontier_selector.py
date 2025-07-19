#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Pose, Twist
import numpy as np
import tf2_ros
from alert_exploration.frontier_utils import *
from copy import deepcopy

EXPANSION_SIZE = 2
SPEED = 0.3
LOOKAHEAD_DISTANCE = 0.4
TARGET_ERROR = 0.3
TARGET_ALLOWED_TIME = 10
MAP_TRIES = 20
UNEXPLORED_EDGES_SIZE = 6
FREE_SPACE_RADIUS = 0.5


def frontierB(matrix):
    for i, row in enumerate(matrix):
        for j, value in enumerate(row):
            if value == 0.0:
                if (i > 0 and matrix[i - 1][j] < 0) or \
                    (i < len(matrix) - 1 and matrix[i + 1][j] < 0) or \
                    (j > 0 and matrix[i][j - 1] < 0) or \
                    (j < len(row) - 1 and matrix[i][j + 1] < 0):
                    matrix[i][j] = 2

    return matrix


def assign_groups(matrix):
    group = 1
    groups = {}
    visited = set()
    stack = []

    def dfs(matrix, i, j, group, groups):
        stack.append((i, j))
        while stack:
            x, y = stack.pop()
            if (x, y) in visited:
                continue
            visited.add((x, y))
            if matrix[x][y] != 2:
                continue
            if group in groups:
                groups[group].append((x, y))
            else:
                groups[group] = [(x, y)]
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (-1, 1), (1, -1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < len(matrix) and 0 <= ny < len(matrix[0]):
                    stack.append((nx, ny))

        return group + 1

    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2 and (i, j) not in visited:
                group = dfs(matrix, i, j, group, groups)

    return matrix, groups


def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]
    return top_five_groups


def getfrontier_groups(map):
    data = np.array(map.data).reshape(map.info.height,map.info.width) * map.info.resolution
    matrix = frontierB(data)
    matrix, groups = assign_groups(matrix)
    return fGroups(groups), matrix


def calculate_centroid(x_coords, y_coords):
    return np.mean(x_coords, dtype=int), np.mean(y_coords, dtype=int)


class OpenCVFrontierDetector(Node):
    def __init__(self):
        super().__init__('detector')
        self.create_subscription(OccupancyGrid, '/mapUGV', self.mapCallBack, 1)
        self.targetspub = self.create_publisher(MarkerArray, "/detected_markers", 1)

        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "/projected_map_1m_inflated", 1)
        self.path_publisher = self.create_publisher(Marker, 'path_marker', 1)

        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.5, self.frontier_timer_callback)
        #self.create_timer(0.1, self.path_follower_timer_callback)

        self.in_motion = False
        self.pursuit_index = 0
        self.x = 0.0
        self.y = 0.0

        #self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

    def frontier_timer_callback(self):
        if not hasattr(self, 'inflated_map'):
            self.get_logger().warn("No map received yet")
            return

        # if not hasattr(self, 'x'):
        #     self.get_logger().warn("No robot position received yet")
        #     return

        # if self.in_motion:
        #     return

        markers = MarkerArray()
        markers.markers = []

        current_map = self.inflated_map
        current_map = add_free_space_at_robot(current_map, self.x, self.y, FREE_SPACE_RADIUS)

        path_marker = Marker()
        path_marker.header = current_map.header
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1  # Line width
        path_marker.color.a = 1.0  # Alpha
        path_marker.color.r = 0.0  # Red
        path_marker.color.g = 1.0  # Green
        path_marker.color.b = 0.0  # Blue

        frontier_groups, matrix = getfrontier_groups(current_map)
        frontiers = []
        if len(frontier_groups) == 0:
            self.get_logger().warn("No frontiers found")
            return
        for i, group in enumerate(frontier_groups):
            y_coords, x_coords = zip(*group[1])
            centroid = calculate_centroid(x_coords, y_coords)
            frontiers.append(centroid)

        # Convert frontiers from map coordinates to world coordinates
        frontiers_world_coords = [map_to_world_coords(current_map, frontier[0], frontier[1]) for frontier in frontiers]

        # Calculate distances from the robot to each frontier
        distances = [
            np.linalg.norm([frontier[0] - self.x, frontier[1] - self.y])
            for frontier in frontiers_world_coords
        ]

        # Filter and sort frontiers based on distance
        filtered_frontiers = [
            frontier for distance, frontier in sorted(zip(distances, frontiers_world_coords))
            if distance > TARGET_ERROR * 2
        ]

        # Keep only the top 5 frontiers
        frontiers = filtered_frontiers[:5]

        #reachable_paths = []
        for i, frontier in enumerate(frontiers):
            adjusted_frontier, is_adjusted_frontier = get_nearest_free_space(current_map, frontier)

            reshaped_map = np.array(current_map.data).reshape(current_map.info.height, current_map.info.width)
            start_coords = world_to_map_coords(current_map, self.x, self.y)
            target_coords = world_to_map_coords(current_map, adjusted_frontier[0], adjusted_frontier[1])

        self.get_logger().info("Start: " + str(start_coords) + " target_coords: " + str(target_coords))

        #    path, is_frontier_reachable = astar(reshaped_map, start_coords, target_coords)

        #     marker = Marker()
        #     marker.header = current_map.header
        #     marker.ns = "markers"
        #     marker.id = i
        #     marker.type = Marker.POINTS
        #     marker.action = Marker.ADD
        #     marker.scale.x = marker.scale.y = 0.2
        #     if is_frontier_reachable:
        #         marker.color.g = 1.0
        #     elif is_adjusted_frontier:
        #         marker.color.b = 1.0
        #     else:
        #         marker.color.r = 1.0
        #     marker.color.a = 1.0
        #     marker.lifetime.sec = 2

        #     point = PointStamped()
        #     point.header = current_map.header
        #     point.point.x = float(adjusted_frontier[0])
        #     point.point.y = float(adjusted_frontier[1])
        #     point.point.z = 0.0

        #     marker.points.append(point.point)
        #     markers.markers.append(marker)

        #     if is_frontier_reachable:
        #         reachable_paths.append([path, pathLength(path)])

        # if len(reachable_paths) > 0:
        #     reachable_paths.sort(key=lambda x: x[1])
        #     # Sort paths by length and select the shortest one
        #     path = reachable_paths[0][0]
        #     self.current_path_map = deepcopy(path)

        #     path = bspline_planning(path, len(path)*5)

        #     self.current_path_world = []
        #     for p in path:
        #         pose = Pose()
        #         pose.position.x, pose.position.y = map_to_world_coords(current_map, p[0], p[1])
        #         path_marker.points.append(pose.position)

        #         self.current_path_world.append([pose.position.x, pose.position.y])

        #     # Start following the path
        #     self.in_motion = True
        #     self.pursuit_index = 0
        #     self.target_allowed_time = self.get_clock().now().to_msg().sec + TARGET_ALLOWED_TIME

        #     self.map_tries = MAP_TRIES

        # else:
        #     self.in_motion = False
        #     if not hasattr(self, 'map_tries'):
        #         self.map_tries = MAP_TRIES
        #     self.map_tries -= 1
        #     if self.map_tries == 0:
        #         print("No frontiers found. Exiting.")
        #         rclpy.shutdown()
        #     print(f"No reachable frontiers found. Retrying {self.map_tries} more times.")

        # self.path_publisher.publish(path_marker)
        # self.targetspub.publish(markers)
        # self.inflated_map_pub.publish(current_map)

    def mapCallBack(self, data):
        self.mapData = add_unexplored_edges(data, UNEXPLORED_EDGES_SIZE)

        self.inflated_map = costmap(self.mapData, EXPANSION_SIZE)


def main(args=None):
    rclpy.init()

    detector = OpenCVFrontierDetector()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()