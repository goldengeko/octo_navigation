import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.time import Time

DEFAULT_PARAMS = {
    # --- TOPOLOGY ---
    'search_radius': 0.2,       
    'edge_threshold': 5,        
    
    # --- MEMORY SETTINGS ---
    'visited_radius': 3.5,      # NEW: If we visited within 3.5m, don't go back.
    'blacklist_radius': 2.5,    # Avoid failed spots by 2.5m
    
    # --- FILTERS ---
    'max_height_diff': 0.15,     
    'min_cluster_size': 0.5,    
    'min_dist_from_robot': 0.5,
    'map_frame': 'map',
    'goal_strategy': 'mean'  # NEW STRATEGY
}

class FrontierDetector:
    def __init__(self, params=None):
        self.params = DEFAULT_PARAMS.copy()
        if params:
            self.params.update(params)
        self.last_goal = None

    # UPDATED SIGNATURE: Accepts 'visited' list
    def process_graph_markers(self, marker_array_msg, robot_pose, blacklist=None, visited=None):
        if blacklist is None: blacklist = []
        if visited is None: visited = []
        
        # 1. Parse Graph Nodes (Height Filtered)
        points_list = []
        z_min = 0.0
        z_max = robot_pose[2] + self.params['max_height_diff']
        
        for marker in marker_array_msg.markers:
            if marker.type in [6, 7, 8]:
                for p in marker.points:
                    if z_min < p.z < z_max:
                        points_list.append([p.x, p.y, p.z])
        
        if not points_list: return None, MarkerArray()
        graph_pts = np.array(points_list)
        
        # 2. Topology (Edge Detection)
        tree = KDTree(graph_pts)
        neighbors = tree.query_ball_point(graph_pts, r=self.params['search_radius'])
        connectivity = np.array([len(n) for n in neighbors]) - 1
        is_frontier_node = connectivity <= self.params['edge_threshold']
        frontier_candidates = graph_pts[is_frontier_node]

        if len(frontier_candidates) == 0: return None, MarkerArray()

        # 3. Clustering
        valid_goals = []
        cand_pcd = o3d.geometry.PointCloud()
        cand_pcd.points = o3d.utility.Vector3dVector(frontier_candidates)
        labels = np.array(cand_pcd.cluster_dbscan(eps=0.4, min_points=3))
        
        if len(labels) > 0:
            max_label = labels.max()
            for i in range(max_label + 1):
                cluster_pts = frontier_candidates[labels == i]
                
                # Size Check
                min_pt = np.min(cluster_pts, axis=0)
                max_pt = np.max(cluster_pts, axis=0)
                if np.linalg.norm(max_pt - min_pt) < self.params['min_cluster_size']:
                    continue

                # Medoid
                geometric_center = np.mean(cluster_pts, axis=0)
                dists = np.linalg.norm(cluster_pts - geometric_center, axis=1)
                medoid = cluster_pts[np.argmin(dists)]
                
                valid_goals.append(medoid)

        # 4. Selection with MEMORY (Visited + Blacklist)
        best_goal = self._select_best_goal(valid_goals, robot_pose, blacklist, visited)
        self.last_goal = best_goal
        
        markers = self._generate_markers(graph_pts, frontier_candidates, valid_goals, best_goal)
        return best_goal, markers

    def _select_best_goal(self, centroids, robot_pose, blacklist, visited):
        if not centroids: return None
        strategy = self.params.get('goal_strategy', 'mean')
        
        blacklist_rad_sq = self.params['blacklist_radius'] ** 2
        visited_rad_sq = self.params['visited_radius'] ** 2
        
        filtered = []
        for c in centroids:
            # Min Dist from robot
            if np.linalg.norm(c - robot_pose) < self.params['min_dist_from_robot']: 
                continue
            
            # --- MEMORY CHECK ---
            is_invalid = False
            
            # Check Blacklist (Failures)
            for bad_pt in blacklist:
                if np.sum((c - bad_pt)**2) < blacklist_rad_sq: 
                    is_invalid = True; break
            if is_invalid: continue

            # Check Visited (Successes / Dead Ends)
            for old_pt in visited:
                if np.sum((c - old_pt)**2) < visited_rad_sq: 
                    is_invalid = True; break
            if is_invalid: continue
            
            filtered.append(c)
        
        if not filtered: return None
        
        if strategy == 'mean':
            all_c = np.array(filtered)
            mean_c = np.mean(all_c, axis=0)
            dists = np.linalg.norm(all_c - mean_c, axis=1)
            return all_c[np.argmin(dists)]
        else:
            dists = [np.linalg.norm(c - robot_pose) for c in filtered]
            return filtered[np.argmax(dists)]

    def _generate_markers(self, graph, candidates, valid_centroids, best_target):
        ma = MarkerArray()
        stamp = Time().to_msg()
        def mk(mid, ns, color, scale, pts, type=Marker.CUBE_LIST):
            m = Marker()
            m.header.frame_id = self.params['map_frame']; m.header.stamp = stamp
            m.ns = ns; m.id = mid; m.type = type; m.action = Marker.ADD
            m.scale.x = scale; m.scale.y = scale; m.scale.z = scale
            m.color.r = color[0]; m.color.g = color[1]; m.color.b = color[2]; m.color.a = 0.8
            m.pose.orientation.w = 1.0
            if type == Marker.SPHERE:
                m.pose.position.x = pts[0]; m.pose.position.y = pts[1]; m.pose.position.z = pts[2]
            else:
                stride = max(1, len(pts)//5000)
                for i in range(0, len(pts), stride):
                    p = Point(); p.x, p.y, p.z = float(pts[i,0]), float(pts[i,1]), float(pts[i,2])
                    m.points.append(p)
            return m
        
        if len(graph) > 0: ma.markers.append(mk(0, "graph", [0.3, 0.3, 0.3], 0.05, graph))
        if len(candidates) > 0: ma.markers.append(mk(1, "candidates", [0.0, 1.0, 1.0], 0.1, candidates))
        if len(valid_centroids) > 0: ma.markers.append(mk(2, "valids", [0.0, 1.0, 0.0], 0.2, np.array(valid_centroids)))
        if best_target is not None: ma.markers.append(mk(3, "best", [1.0, 1.0, 0.0], 0.5, best_target, type=Marker.SPHERE))
        return ma
