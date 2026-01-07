#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import MarkerArray
from mbf_msgs.action import MoveBase
import math
import numpy as np
import tf2_ros

from utils import FrontierDetector

# SETTINGS
GRAPH_TOPIC = '/move_base_flex/graph_nodes' 
ACTION_SERVER_NAME = '/move_base_flex/move_base'
MAP_FRAME = 'map'
ROBOT_FRAME = 'base_link'
MIN_GOAL_DISTANCE = 0.5
WAIT_AFTER_SUCCESS = 1.0

class FrontierNavigationManager(Node):
    def __init__(self):
        super().__init__('frontier_navigation_manager')
        
        self.get_logger().info("Initializing Frontier Manager (Spatial Memory Enabled)...")
        self.detector = FrontierDetector()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.graph_sub = self.create_subscription(
            MarkerArray, GRAPH_TOPIC, self.graph_callback, qos)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_debug_markers', 10)

        self.action_client = ActionClient(self, MoveBase, ACTION_SERVER_NAME)
        self.get_logger().info(f"Waiting for '{ACTION_SERVER_NAME}'...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Action server not available yet.")
        else:
            self.get_logger().info("Action server connected!")

        self.exploration_timer = self.create_timer(1.0, self.control_loop)
        
        self.latest_goal_coords = None 
        self.robot_pose = None         
        self.is_navigating = False
        
        # --- MEMORY ---
        self.blacklisted_goals = [] # Failed goals (Unreachable)
        self.visited_goals = []     # Succeeded goals (Closed Boundaries / Done)
        
        self.current_active_goal = None
        self.wait_until = Time(seconds=0, clock_type=self.get_clock().clock_type)

    def graph_callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, Time())
            self.robot_pose = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])
        except Exception:
            return

        # Pass BOTH lists to the detector
        goal, markers = self.detector.process_graph_markers(
            msg, 
            self.robot_pose, 
            blacklist=self.blacklisted_goals,
            visited=self.visited_goals # <--- NEW
        )

        self.marker_pub.publish(markers)
        self.latest_goal_coords = goal

    def control_loop(self):
        if self.is_navigating: return
        
        now = self.get_clock().now()
        if now < self.wait_until: return

        if self.latest_goal_coords is None:
            self.get_logger().info("No valid frontiers found.", throttle_duration_sec=5.0)
            return
            
        if self.robot_pose is None: return

        dist = np.linalg.norm(self.latest_goal_coords - self.robot_pose)
        if dist < MIN_GOAL_DISTANCE: return

        self.current_active_goal = np.copy(self.latest_goal_coords)

        goal_point = Point()
        goal_point.x = float(self.current_active_goal[0])
        goal_point.y = float(self.current_active_goal[1])
        goal_point.z = float(self.current_active_goal[2])

        self.get_logger().info(f"Sending Goal: {self.current_active_goal}")
        self.send_navigation_goal(goal_point)

    def send_navigation_goal(self, goal_point_3d: Point):
        self.is_navigating = True
        
        goal_msg = MoveBase.Goal()
        goal_msg.target_pose.header.frame_id = MAP_FRAME
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position = goal_point_3d
        
        dx = goal_point_3d.x - self.robot_pose[0]
        dy = goal_point_3d.y - self.robot_pose[1]
        yaw = math.atan2(dy, dx)
        goal_msg.target_pose.pose.orientation = self.yaw_to_quaternion(yaw)

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0); q.w = math.cos(yaw / 2.0)
        return q

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED.')
            self.handle_failure()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal Reached! Marking area as VISITED.')
            
            # --- SUCCESS MEMORY ---
            if self.current_active_goal is not None:
                self.visited_goals.append(self.current_active_goal)
                # Keep memory from getting infinite (e.g., last 100 spots)
                if len(self.visited_goals) > 100:
                    self.visited_goals.pop(0)

            self.wait_until = self.get_clock().now() + Duration(seconds=WAIT_AFTER_SUCCESS)
            self.is_navigating = False
            self.latest_goal_coords = None # Force re-scan
            self.current_active_goal = None
        else:
            self.get_logger().warn(f'Navigation Failed (Status: {status}).')
            self.handle_failure()

    def handle_failure(self):
        # --- FAILURE MEMORY ---
        if self.current_active_goal is not None:
            self.get_logger().info(f"Blacklisting failed goal: {self.current_active_goal}")
            self.blacklisted_goals.append(self.current_active_goal)
            if len(self.blacklisted_goals) > 50: 
                self.blacklisted_goals.pop(0)
        
        self.latest_goal_coords = None 
        self.current_active_goal = None
        self.is_navigating = False

def main(args=None):
    rclpy.init(args=args)
    node = FrontierNavigationManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
