/*
 *  Copyright 2025, MASCOR
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    MASCOR
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <octomap_msgs/conversions.h>

#include <mbf_msgs/action/get_path.hpp>
#include <astar_octo_planner/astar_octo_planner.h>

#include <queue>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(astar_octo_planner::AstarOctoPlanner, mbf_octo_core::OctoPlanner);

namespace astar_octo_planner
{

// Legacy grid-based A* structures removed; planner uses graph-based A* (planOnGraph)

AstarOctoPlanner::AstarOctoPlanner()
: voxel_size_(0.1),  // double than that of octomap resolution
  z_threshold_(0.3)   // same as Z_THRESHOLD in Python
{
  // Planner now relies on octomap as the authoritative map source.
  // Set a default minimum bound; this will be updated when processing octomap messages.
  min_bound_ = {0.0, 0.0, 0.0};
}

AstarOctoPlanner::~AstarOctoPlanner() {}

uint32_t AstarOctoPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                    const geometry_msgs::msg::PoseStamped& goal,
                                    double tolerance,
                                    std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                    double& cost,
                                    std::string& message)
{
  RCLCPP_INFO(node_->get_logger(), "Start astar octo planner.");
  RCLCPP_INFO(node_->get_logger(), "Start position: x = %f, y = %f, z = %f, frame_id = %s",
              start.pose.position.x, start.pose.position.y, start.pose.position.z,
              start.header.frame_id.c_str());

  // Transform start/goal into the octomap frame (map_frame_) if necessary.
  geometry_msgs::msg::PoseStamped start_in_map = start;
  geometry_msgs::msg::PoseStamped goal_in_map = goal;

  // This planner no longer performs TF transforms. The start and goal are
  // used exactly as provided (no frame checks or automatic transforms).

  // We'll compute grid indices after we set the active voxel size (from octree).
  // Leave start_in_map and goal_in_map as the provided poses for now.
  // Planner now requires an octree as the authoritative map source. If we don't
  // have one yet, return failure instead of falling back to a dense occupancy
  // grid (which has been removed to avoid large allocations).
  if (!octree_) {
    RCLCPP_WARN(node_->get_logger(), "No octomap available; cannot plan. Waiting for /navigation/octomap_binary.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // Use octree resolution as the active voxel size to avoid mismatches
  active_voxel_size_ = std::max(voxel_size_, octree_->getResolution());

  // Use the provided start as the robot's true starting position (don't snap it).
  // Snap only the goal to the nearest occupied leaf so the planner aims at surface points.
  // RCLCPP_INFO(node_->get_logger(), "Using provided start: x=%.3f y=%.3f z=%.3f (frame=%s)",
  //             start_in_map.pose.position.x, start_in_map.pose.position.y, start_in_map.pose.position.z,
  //             start_in_map.header.frame_id.c_str());
  // Snap start/goal world coordinates to the active voxel size to avoid tiny
  // floating-point differences and reduce search effort (e.g. resolution=0.1 -> one decimal).
  double vs_snap = active_voxel_size_;
  auto snap_world = [vs_snap](geometry_msgs::msg::Point &p) {
    p.x = std::round(p.x / vs_snap) * vs_snap;
    p.y = std::round(p.y / vs_snap) * vs_snap;
    p.z = std::round(p.z / vs_snap) * vs_snap;
  };
  // Create mutable copies for snapping
  geometry_msgs::msg::Point start_world = start_in_map.pose.position;
  geometry_msgs::msg::Point goal_world = goal_in_map.pose.position;
  snap_world(start_world);
  snap_world(goal_world);
  RCLCPP_INFO(node_->get_logger(), "Snapped start to: x=%.3f y=%.3f z=%.3f",
              start_world.x, start_world.y, start_world.z);

  plan.clear();
  std::string path_frame = map_frame_.empty() ? start.header.frame_id : map_frame_;
  rclcpp::Time now = node_->now();
    // Try to use the precomputed connectivity graph if available. If graph is
    // missing or yields no route, fall back to a simple two-point plan.
    {
      // Ensure graph is built if dirty or empty (build synchronously here).
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if ((graph_nodes_.empty() || graph_dirty_)) {
        RCLCPP_INFO(node_->get_logger(), "Graph missing or dirty — building connectivity graph synchronously...");
        buildConnectivityGraph();
        graph_dirty_ = false;
        RCLCPP_INFO(node_->get_logger(), "Graph built: nodes=%zu", graph_nodes_.size());
      }
    }

    // Find closest graph nodes to start and goal
    std::string start_id;
    std::string goal_id;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      start_id = findClosestGraphNode(octomap::point3d(start_world.x, start_world.y, start_world.z));
      goal_id  = findClosestGraphNode(octomap::point3d(goal_world.x,  goal_world.y,  goal_world.z));
    }

    RCLCPP_INFO(node_->get_logger(), "Graph attempt: start_id='%s' goal_id='%s'",
                start_id.empty() ? "<none>" : start_id.c_str(),
                goal_id.empty() ? "<none>" : goal_id.c_str());

    auto valid_center = [&](const octomap::point3d &c)->bool {
      if (!std::isfinite(c.x()) || !std::isfinite(c.y()) || !std::isfinite(c.z())) return false;
      if (c.x() < min_bound_[0] - 1e-6 || c.x() > max_bound_[0] + 1e-6) return false;
      if (c.y() < min_bound_[1] - 1e-6 || c.y() > max_bound_[1] + 1e-6) return false;
      if (c.z() < min_bound_[2] - 1e-6 || c.z() > max_bound_[2] + 1e-6) return false;
      return true;
    };

    bool centers_ok = true;
    if (!start_id.empty()) {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      const auto &s = graph_nodes_.at(start_id);
      RCLCPP_INFO(node_->get_logger(), "Start graph center: (%.3f, %.3f, %.3f)", s.center.x(), s.center.y(), s.center.z());
      centers_ok &= valid_center(s.center);
    }
    if (!goal_id.empty()) {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      const auto &g = graph_nodes_.at(goal_id);
      RCLCPP_INFO(node_->get_logger(), "Goal graph center: (%.3f, %.3f, %.3f)", g.center.x(), g.center.y(), g.center.z());
      centers_ok &= valid_center(g.center);
    }

    if (!centers_ok || start_id.empty() || goal_id.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Graph path not available or invalid centers. Aborting plan (NO_PATH_FOUND) for debugging.");
      message = "Graph path not available or invalid centers";
      return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
    } else {
      // Compute path on graph under lock to ensure consistency
      std::vector<std::string> path_ids;
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        path_ids = planOnGraph(start_id, goal_id);
      }
      if (path_ids.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No path found on graph between %s and %s — aborting (NO_PATH_FOUND)", start_id.c_str(), goal_id.c_str());
        message = "No path found on graph";
        return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
      } else {
        // Convert graph node centers to PoseStamped in path order
        RCLCPP_INFO(node_->get_logger(), "Graph path ids (%zu):", path_ids.size());
        std::lock_guard<std::mutex> lock(graph_mutex_);
        for (const auto &id : path_ids) {
          RCLCPP_INFO(node_->get_logger(), "  -> %s", id.c_str());
          const auto &gn = graph_nodes_.at(id);
          geometry_msgs::msg::PoseStamped p;
          p.header.frame_id = path_frame;
          p.header.stamp = now;
          p.pose.position.x = gn.center.x();
          p.pose.position.y = gn.center.y();
          p.pose.position.z = gn.center.z();
          p.pose.orientation.w = 1.0;
          plan.push_back(p);
        }
      }
    }

  // Compute a simple cost (e.g., number of steps) – replace with a proper cost calculation if desired.
  cost = static_cast<double>(plan.size());

  // Publish the path for visualization. Before publishing sanity-check values.
  for (const auto &p : plan) {
    if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) || !std::isfinite(p.pose.position.z)) {
      RCLCPP_ERROR(node_->get_logger(), "Path contains non-finite coordinates; aborting publish.");
      return mbf_msgs::action::GetPath::Result::FAILURE;
    }
    // If coordinates are outside reasonable bounds (e.g. > 1e6), abort and log.
    if (std::fabs(p.pose.position.x) > 1e6 || std::fabs(p.pose.position.y) > 1e6 || std::fabs(p.pose.position.z) > 1e6) {
      RCLCPP_ERROR(node_->get_logger(), "Path contains extremely large coordinates (x=%.3f y=%.3f z=%.3f); aborting.",
                   p.pose.position.x, p.pose.position.y, p.pose.position.z);
      return mbf_msgs::action::GetPath::Result::FAILURE;
    }
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = node_->now();
  // Publish in the octomap map frame (if set), else fall back to start frame.
  path_msg.header.frame_id = map_frame_.empty() ? start.header.frame_id : map_frame_;
  path_msg.poses = plan;
  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path found with length: " << cost << " steps");

  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

// legacy nearest-3d helpers removed; graph-based planner uses nearest graph nodes


// ------------------ Prototype graph builder / planner ------------------

void AstarOctoPlanner::buildConnectivityGraph(double eps)
{
  graph_nodes_.clear();
  graph_adj_.clear();
  if (!octree_) return;

  unsigned int maxDepth = octree_->getTreeDepth();

  // First pass: collect candidate free leaf nodes (use leaf iterator available in this OctoMap build)
  size_t total_leafs = 0;
  size_t free_leafs = 0;
  size_t skipped_nonfinite = 0;
  size_t skipped_out_of_bounds = 0;
  size_t skipped_collision = 0;
  size_t skipped_no_surface = 0;
  size_t skipped_step_too_high = 0;
  size_t inserted = 0;
  size_t rejected_distance = 0;
  size_t rejected_los = 0;

  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    ++total_leafs;
    // consider only nodes that are free (no occupancy)
    if (octree_->isNodeOccupied(*it)) continue;
    ++free_leafs;
    // get center coordinate from iterator
    octomap::point3d center = it.getCoordinate();
    // derive key at finest resolution for this center
    // derive a safe id based on the center coordinates (avoid depending on OcTreeKey/depth id semantics)
    // skip nodes that are outside the current computed map bounds (they produce large bogus centers)
    if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z())) { ++skipped_nonfinite; continue; }
    // allow a small margin around computed bounds so free/unknown leafs close to occupied
    double margin = octree_->getResolution() * 2.0;
    if (center.x() < min_bound_[0] - margin || center.x() > max_bound_[0] + margin ||
        center.y() < min_bound_[1] - margin || center.y() > max_bound_[1] + margin ||
        center.z() < min_bound_[2] - margin || center.z() > max_bound_[2] + margin) {
      ++skipped_out_of_bounds;
      // skip leaf centers that are far outside the map bounds
      continue;
    }

    octomap::OcTreeKey key = octree_->coordToKey(center);
    // conservative approach: assume leaf at max depth for size query is acceptable for prototype
    unsigned int depth = octree_->getTreeDepth();
    double size = octree_->getNodeSize(depth);

    // skip node if the robot footprint would collide at this center
  std::tuple<int,int,int> grid_idx = { static_cast<int>(std::floor((center.x() - min_bound_[0]) / active_voxel_size_ + 0.5)),
                     static_cast<int>(std::floor((center.y() - min_bound_[1]) / active_voxel_size_ + 0.5)),
                     static_cast<int>(std::floor((center.z() - min_bound_[2]) / active_voxel_size_ + 0.5)) };
  if (!isCylinderCollisionFree(grid_idx, robot_radius_)) { ++skipped_collision; continue; }

    // skip node if there's no occupied surface within max_surface_distance_ below it
    if (max_surface_distance_ > 0.0 || max_step_height_ > 0.0) {
      bool found_surface = false;
      double step = std::max(active_voxel_size_, 0.05);
      // search down to (max_surface_distance_ + max_step_height_) to allow small steps
      double search_depth = max_surface_distance_ + max_step_height_;
      int steps = static_cast<int>(std::ceil(search_depth / step));
      double found_z = std::numeric_limits<double>::infinity();
      for (int s = 1; s <= steps; ++s) {
        double z = center.z() - s * step;
        if (z < min_bound_[2] - 1e-6) break;
        octomap::OcTreeNode* nn = octree_->search(center.x(), center.y(), z);
        if (nn && octree_->isNodeOccupied(nn)) { found_surface = true; found_z = z; break; }
      }
      if (!found_surface) { ++skipped_no_surface; continue; }
      double dz = center.z() - found_z;
      if (dz > max_surface_distance_ && dz <= (max_surface_distance_ + max_step_height_)) {
        // accept as a step — fine
      } else if (dz > (max_surface_distance_ + max_step_height_)) {
        ++skipped_step_too_high;
        continue;
      }
    }

  GraphNode gn;
    gn.key = key;
    gn.depth = depth;
    gn.center = center;
    gn.size = size;

    // create a stable id derived from the rounded center in millimeters
    char idbuf[128];
    int ix = static_cast<int>(std::round(center.x() * 1000.0));
    int iy = static_cast<int>(std::round(center.y() * 1000.0));
    int iz = static_cast<int>(std::round(center.z() * 1000.0));
    std::snprintf(idbuf, sizeof(idbuf), "c_%d_%d_%d", ix, iy, iz);
    graph_nodes_.emplace(std::string(idbuf), gn);
    ++inserted;
  }

  // RCLCPP_INFO(node_->get_logger(), "Graph build stats: total_leafs=%zu free_leafs=%zu nonfinite=%zu out_of_bounds=%zu inserted=%zu",
  //             total_leafs, free_leafs, skipped_nonfinite, skipped_out_of_bounds, inserted);
  // RCLCPP_INFO(node_->get_logger(), "Graph build extra: skipped_collision=%zu", skipped_collision);
  // RCLCPP_INFO(node_->get_logger(), "Graph build extra: skipped_no_surface=%zu", skipped_no_surface);
  // RCLCPP_INFO(node_->get_logger(), "Graph build extra: skipped_step_too_high=%zu", skipped_step_too_high);

  // Second pass: sampling-based neighbour detection (26 directions)
  const std::vector<octomap::point3d> dirs = [](){
    std::vector<octomap::point3d> d;
    for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz) {
      if (dx==0 && dy==0 && dz==0) continue;
      d.emplace_back(dx, dy, dz);
    }
    return d;
  }();

  for (const auto &kv : graph_nodes_) {
    const std::string id = kv.first;
    const GraphNode &node = kv.second;
    double node_size = node.size;
    for (const auto &dir : dirs) {
      octomap::point3d sample = node.center + dir * (node_size * 0.5 + eps);
      // bounds check using min/max bounds
      if (sample.x() < min_bound_[0] - node_size || sample.x() > max_bound_[0] + node_size ||
          sample.y() < min_bound_[1] - node_size || sample.y() > max_bound_[1] + node_size ||
          sample.z() < min_bound_[2] - node_size || sample.z() > max_bound_[2] + node_size) {
        continue;
      }
      octomap::OcTreeNode* found = octree_->search(sample.x(), sample.y(), sample.z());
      if (!found) continue;
      // get key for found node
      octomap::OcTreeKey fkey = octree_->coordToKey(sample);
      // build id
      // depth determination: try to find matching depth by probing sizes (approx)
      unsigned int fdepth = maxDepth;
      // Find a matching node in our graph by key prefix
      std::string found_id;
      for (unsigned int d = 0; d <= maxDepth; ++d) {
        octomap::OcTreeKey k = fkey;
        // try to adjust? we'll use full key and assume graph contains the exact key
      }
      // naive: search graph_nodes_ by comparing centers (slow but fine for prototype)
      octomap::point3d found_center = octree_->keyToCoord(fkey);
      double best_dist = std::numeric_limits<double>::infinity();
      std::string best_id;
      for (const auto &kv2 : graph_nodes_) {
        double dx = kv2.second.center.x() - found_center.x();
        double dy = kv2.second.center.y() - found_center.y();
        double dz = kv2.second.center.z() - found_center.z();
        double dsq = dx*dx + dy*dy + dz*dz;
        if (dsq < best_dist) { best_dist = dsq; best_id = kv2.first; }
      }
      if (!best_id.empty()) {
        // accept neighbour if size of neighbor >= this node size (equal or larger)
        if (graph_nodes_[best_id].size + 1e-9 >= node_size) {
          // ensure the matched center is not too far (avoid matching distant nodes through obstacles)
          double best_dist_sq = best_dist;
          double thresh = node_size * 1.5; // allow a bit more than one node size
          if (best_dist_sq > thresh * thresh) {
            ++rejected_distance;
            continue;
          }
          // line-of-sight check: ensure straight segment between centers does not hit an occupied voxel
          const octomap::point3d &a = node.center;
          const octomap::point3d &b = graph_nodes_.at(best_id).center;
          octomap::point3d dir = b - a;
          double dist = dir.norm();
          if (dist > 1e-9) {
            dir /= dist; // normalize
            bool hit = false;
            // safe fallback: sample along segment in steps of ~voxel size and check occupancy via search()
            // (we avoid octree::castRay entirely to prevent 'hit bounds' warnings)
            double step_size = std::max(active_voxel_size_, 0.05);
            int steps = std::max(1, static_cast<int>(std::ceil(dist / step_size)));
            // sample from 1 .. steps-1 to avoid checking the destination center (which should be free)
            for (int si = 1; si < steps; ++si) {
              double t = static_cast<double>(si) / static_cast<double>(steps);
              octomap::point3d s = a + dir * (dist * t);
              // skip samples outside bounds
              const double eps = 1e-6;
              if (s.x() < min_bound_[0] - eps || s.x() > max_bound_[0] + eps ||
                  s.y() < min_bound_[1] - eps || s.y() > max_bound_[1] + eps ||
                  s.z() < min_bound_[2] - eps || s.z() > max_bound_[2] + eps) continue;
              octomap::OcTreeNode* nn = octree_->search(s.x(), s.y(), s.z());
              if (nn && octree_->isNodeOccupied(nn)) { hit = true; break; }
            }
            if (hit) {
              ++rejected_los;
              continue;
            }
          }
          // finally accept neighbour
          auto &vec = graph_adj_[id];
          // avoid duplicates
          if (std::find(vec.begin(), vec.end(), best_id) == vec.end()) vec.push_back(best_id);
        }
      }
    }
  }

  // After building all adjacency, emit a compact diagnostic summary and check ranges
  if (!graph_nodes_.empty()) {
    double min_cx = std::numeric_limits<double>::infinity();
    double min_cy = std::numeric_limits<double>::infinity();
    double min_cz = std::numeric_limits<double>::infinity();
    double max_cx = -std::numeric_limits<double>::infinity();
    double max_cy = -std::numeric_limits<double>::infinity();
    double max_cz = -std::numeric_limits<double>::infinity();
    size_t out_of_bounds = 0;
    for (const auto &kv : graph_nodes_) {
      const auto &n = kv.second;
  min_cx = std::min(min_cx, static_cast<double>(n.center.x()));
  min_cy = std::min(min_cy, static_cast<double>(n.center.y()));
  min_cz = std::min(min_cz, static_cast<double>(n.center.z()));
  max_cx = std::max(max_cx, static_cast<double>(n.center.x()));
  max_cy = std::max(max_cy, static_cast<double>(n.center.y()));
  max_cz = std::max(max_cz, static_cast<double>(n.center.z()));
      if (n.center.x() < min_bound_[0] - 1e-6 || n.center.x() > max_bound_[0] + 1e-6 ||
          n.center.y() < min_bound_[1] - 1e-6 || n.center.y() > max_bound_[1] + 1e-6 ||
          n.center.z() < min_bound_[2] - 1e-6 || n.center.z() > max_bound_[2] + 1e-6) {
        ++out_of_bounds;
      }
    }
    // RCLCPP_INFO(node_->get_logger(), "Background graph built: nodes=%zu center_x=[%.3f .. %.3f] center_y=[%.3f .. %.3f] center_z=[%.3f .. %.3f] out_of_bounds=%zu",
    //             graph_nodes_.size(), min_cx, max_cx, min_cy, max_cy, min_cz, max_cz, out_of_bounds);
    // RCLCPP_INFO(node_->get_logger(), "Graph adjacency rejects: distance=%zu los=%zu", rejected_distance, rejected_los);
    // Log up to 5 samples
    size_t cnt = 0;
    for (const auto &kv : graph_nodes_) {
      if (cnt++ >= 5) break;
      const auto &n = kv.second;
      // RCLCPP_INFO(node_->get_logger(), "Graph node sample: id=%s center=(%.3f, %.3f, %.3f) size=%.3f",
      //             kv.first.c_str(), n.center.x(), n.center.y(), n.center.z(), n.size);
    }
  }

  // Optionally publish graph node centers as visualization markers for RViz
  if (publish_graph_markers_) {
    publishGraphMarkers();
  }
}

void AstarOctoPlanner::publishGraphMarkers()
{
  if (!graph_marker_pub_ || graph_nodes_.empty()) return;
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
  m.header.stamp = node_->now();
  m.ns = "graph_nodes";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  // set scale for cubes (use node size or default to active voxel size if available)
  double scale = active_voxel_size_ > 0.0 ? std::max(active_voxel_size_, 0.05) : 0.05;
  m.scale.x = static_cast<float>(scale);
  m.scale.y = static_cast<float>(scale);
  m.scale.z = static_cast<float>(scale);
  // color
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.color.a = 0.8f;

  for (const auto &kv : graph_nodes_) {
    geometry_msgs::msg::Point p;
    p.x = kv.second.center.x();
    p.y = kv.second.center.y();
    p.z = kv.second.center.z();
    m.points.push_back(p);
  }
  ma.markers.push_back(m);
  graph_marker_pub_->publish(ma);
  //RCLCPP_INFO(node_->get_logger(), "Published graph markers: markers=%zu points=%zu scale=%.3f", ma.markers.size(), ma.markers.empty() ? 0 : ma.markers[0].points.size(), scale);
}

std::string AstarOctoPlanner::findClosestGraphNode(const octomap::point3d& p) const
{
  if (graph_nodes_.empty()) return std::string();
  double best = std::numeric_limits<double>::infinity();
  std::string best_id;
  for (const auto &kv : graph_nodes_) {
    double dx = kv.second.center.x() - p.x();
    double dy = kv.second.center.y() - p.y();
    double dz = kv.second.center.z() - p.z();
    double d = dx*dx + dy*dy + dz*dz;
    if (d < best) { best = d; best_id = kv.first; }
  }
  return best_id;
}

std::vector<std::string> AstarOctoPlanner::planOnGraph(const std::string& start_id, const std::string& goal_id) const
{
  std::vector<std::string> empty;
  if (start_id.empty() || goal_id.empty()) return empty;
  if (graph_nodes_.find(start_id) == graph_nodes_.end() || graph_nodes_.find(goal_id) == graph_nodes_.end()) return empty;

  struct Item { std::string id; double f; double g; };
  struct Cmp { bool operator()(const Item&a, const Item&b) const { return a.f > b.f; } };

  std::priority_queue<Item, std::vector<Item>, Cmp> open;
  std::unordered_map<std::string, double> gscore;
  std::unordered_map<std::string, std::string> came_from;

  auto heuristic = [this,&goal_id](const std::string &a)->double{
    if (graph_nodes_.find(a) == graph_nodes_.end() || graph_nodes_.find(goal_id) == graph_nodes_.end()) return 0.0;
    const auto &pa = graph_nodes_.at(a).center;
    const auto &pg = graph_nodes_.at(goal_id).center;
    double dx = pa.x()-pg.x(); double dy = pa.y()-pg.y(); double dz = pa.z()-pg.z();
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  };

  open.push({start_id, heuristic(start_id), 0.0});
  gscore[start_id] = 0.0;

  while (!open.empty()) {
    Item cur = open.top(); open.pop();
    if (cur.id == goal_id) {
      // reconstruct
      std::vector<std::string> path;
      std::string u = goal_id;
      while (came_from.find(u) != came_from.end()) { path.push_back(u); u = came_from.at(u); }
      path.push_back(start_id);
      std::reverse(path.begin(), path.end());
      return path;
    }
    if (gscore.find(cur.id) != gscore.end() && cur.g > gscore.at(cur.id)) continue; // stale
    auto it = graph_adj_.find(cur.id);
    if (it == graph_adj_.end()) continue;
    for (const std::string &nb : it->second) {
      if (graph_nodes_.find(cur.id) == graph_nodes_.end() || graph_nodes_.find(nb) == graph_nodes_.end()) continue;
      double tentative = cur.g + (graph_nodes_.at(cur.id).center.distance(graph_nodes_.at(nb).center));
      if (gscore.find(nb) == gscore.end() || tentative < gscore[nb]) {
        came_from[nb] = cur.id;
        gscore[nb] = tentative;
        open.push({nb, tentative + heuristic(nb), tentative});
      }
    }
  }

  return empty;
}

// Legacy grid-based astar() removed — graph-based planning is used (planOnGraph).

// Convert a grid index triple into world coordinates (meters)
std::array<double, 3> AstarOctoPlanner::gridToWorld(const std::tuple<int, int, int>& grid_pt)
{
  int ix, iy, iz;
  std::tie(ix, iy, iz) = grid_pt;
  double vx = active_voxel_size_ > 0.0 ? active_voxel_size_ : voxel_size_;
  double wx = min_bound_[0] + static_cast<double>(ix) * vx;
  double wy = min_bound_[1] + static_cast<double>(iy) * vx;
  double wz = min_bound_[2] + static_cast<double>(iz) * vx;
  return {wx, wy, wz};
}

// The following helpers were removed from the source as they are no longer
// referenced by the graph-based planner implementation: worldToGrid,
// isWithinBounds, and isOccupied. Their declarations were removed from the
// public header to avoid exposing unused private helpers.

// Very simple cylinder collision check: sample a few radial directions at the given radius
// and ensure no occupied node is found at those sample points (at the same z).
bool AstarOctoPlanner::isCylinderCollisionFree(const std::tuple<int, int, int>& coord, double radius)
{
  // Conservative rectangular footprint check: sample points within the robot footprint
  // across several height slices and return false if any sample collides with an occupied node.
  if (!octree_) return true;
  auto w = gridToWorld(coord);
  double half_w = 0.5 * (robot_width_ + footprint_margin_);
  double half_l = 0.5 * (robot_length_ + footprint_margin_);
  // number of samples along length and width (runtime-configurable)
  const int nx = std::max(1, footprint_samples_x_);
  const int ny = std::max(1, footprint_samples_y_);
  // height sampling across robot height
  double z_bottom = w[2];
  double z_top = w[2] + robot_height_;
  const int nz = std::max(1, static_cast<int>(std::ceil(robot_height_ / std::max(0.05, active_voxel_size_))));
  for (int iz = 0; iz < nz; ++iz) {
    double frac = nz == 1 ? 0.5 : static_cast<double>(iz) / static_cast<double>(nz - 1);
    double z = z_bottom + frac * (z_top - z_bottom);
    for (int ix = 0; ix < nx; ++ix) {
      double fx = nx == 1 ? 0.0 : (static_cast<double>(ix) / static_cast<double>(nx - 1) * 2.0 - 1.0);
      double sx = w[0] + fx * half_l;
      for (int iy = 0; iy < ny; ++iy) {
        double fy = ny == 1 ? 0.0 : (static_cast<double>(iy) / static_cast<double>(ny - 1) * 2.0 - 1.0);
        double sy = w[1] + fy * half_w;
        octomap::OcTreeNode* n = octree_->search(sx, sy, z);
        if (n && octree_->isNodeOccupied(n)) return false;
      }
    }
  }
  // also check center column
  for (double z = z_bottom; z <= z_top; z += std::max(active_voxel_size_, 0.05)) {
    octomap::OcTreeNode* c = octree_->search(w[0], w[1], z);
    if (c && octree_->isNodeOccupied(c)) return false;
  }
  return true;
}

// hasNoOccupiedCellsAbove: implementation removed (legacy helper not used)

// pointcloud2Callback removed: planner now uses octree messages exclusively.

bool AstarOctoPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

bool AstarOctoPlanner::initialize(const std::string& plugin_name, const rclcpp::Node::SharedPtr& node)
{
  name_ = plugin_name;
  node_ = node;

  config_.publish_vector_field = node_->declare_parameter(name_ + ".publish_vector_field", config_.publish_vector_field);
  config_.publish_face_vectors   = node_->declare_parameter(name_ + ".publish_face_vectors", config_.publish_face_vectors);
  config_.goal_dist_offset       = node_->declare_parameter(name_ + ".goal_dist_offset", config_.goal_dist_offset);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the vertex cost limit with which it can be accessed.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.cost_limit = node_->declare_parameter(name_ + ".cost_limit", config_.cost_limit);
  }

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
  // Subscribe to octomap binary topic (primary map source)
  octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
    "/navigation/octomap_binary", 1,
    std::bind(&AstarOctoPlanner::octomapCallback, this, std::placeholders::_1));

  // Declare planner tuning parameters (can be changed at runtime)
  z_threshold_ = node_->declare_parameter(name_ + ".z_threshold", z_threshold_);
  robot_radius_ = node_->declare_parameter(name_ + ".robot_radius", robot_radius_);
  min_vertical_clearance_ = node_->declare_parameter(name_ + ".min_vertical_clearance", min_vertical_clearance_);
  max_vertical_clearance_ = node_->declare_parameter(name_ + ".max_vertical_clearance", max_vertical_clearance_);
  // robot footprint params (runtime-tunable)
  robot_width_ = node_->declare_parameter(name_ + ".robot_width", robot_width_);
  robot_length_ = node_->declare_parameter(name_ + ".robot_length", robot_length_);
  robot_height_ = node_->declare_parameter(name_ + ".robot_height", robot_height_);
  footprint_margin_ = node_->declare_parameter(name_ + ".footprint_margin", footprint_margin_);
  footprint_samples_x_ = node_->declare_parameter(name_ + ".footprint_samples_x", footprint_samples_x_);
  footprint_samples_y_ = node_->declare_parameter(name_ + ".footprint_samples_y", footprint_samples_y_);
  max_surface_distance_ = node_->declare_parameter(name_ + ".max_surface_distance", max_surface_distance_);
  max_step_height_ = node_->declare_parameter(name_ + ".max_step_height", max_step_height_);

  // Graph marker publishing (for RViz visualization of node centers)
    publish_graph_markers_ = node_->declare_parameter(name_ + ".publish_graph_markers", publish_graph_markers_);
    // always create publisher so we can toggle publishing at runtime without creating publishers later
    graph_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/graph_nodes", rclcpp::QoS(1));

  RCLCPP_INFO(node_->get_logger(), "Declared parameters: %s.z_threshold=%.3f, %s.robot_radius=%.3f, %s.min_vertical_clearance=%.3f, %s.max_vertical_clearance=%.3f",
               name_.c_str(), z_threshold_, name_.c_str(), robot_radius_, name_.c_str(), min_vertical_clearance_, name_.c_str(), max_vertical_clearance_);

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&AstarOctoPlanner::reconfigureCallback, this, std::placeholders::_1));

  // Create a periodic timer to (re)build the connectivity graph in background
  graph_build_timer_ = node_->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (!graph_dirty_) return;
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (!graph_dirty_) return;
      RCLCPP_INFO(node_->get_logger(), "Background: rebuilding connectivity graph...");
      buildConnectivityGraph();
      graph_dirty_ = false;
      RCLCPP_INFO(node_->get_logger(), "Background: connectivity graph ready. Nodes=%zu", graph_nodes_.size());
    }
  );

  return true;
}

// Helper: traverse the given octree and return the node (may be internal)
// that contains the provided world coordinate. If a child for the
// coordinate doesn't exist at some level, return the current internal node
// (coarse cell).
// octomap::OcTreeNode* AstarOctoPlanner::findNodeByCoordinateInTree(const octomap::point3d& coord)
// {
//   if (!octree_) return nullptr;
//
//   octomap::OcTreeNode* current = octree_->getRoot();
//   if (!current) return nullptr;
//
//   unsigned int maxDepth = octree_->getTreeDepth();
//
//     // Use octree's search convenience method. It will return the finest node
//     // available for the coordinate (may be internal if the tree is pruned).
//     octomap::OcTreeNode* node = octree_->search(coord.x(), coord.y(), coord.z());
//     if (node) return node;
//
//     // Fallback: return the root node if search failed (coarse cell).
//     return octree_->getRoot();
// }

void AstarOctoPlanner::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  // Deserialize octomap message into an AbstractOcTree
  octomap::AbstractOcTree* tree_ptr = octomap_msgs::fullMsgToMap(*msg);
  if (!tree_ptr) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to convert Octomap message to AbstractOcTree");
    return;
  }

  octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(tree_ptr);
  if (!tree) {
    RCLCPP_ERROR(node_->get_logger(), "Octomap message is not an OcTree type");
    delete tree_ptr;
    return;
  }

  // Apply sensor model / occupancy parameters
  tree->setProbHit(octomap_prob_hit_);
  tree->setProbMiss(octomap_prob_miss_);
  tree->setOccupancyThres(octomap_thres_);
  tree->setClampingThresMin(octomap_clamp_min_);
  tree->setClampingThresMax(octomap_clamp_max_);

  // Store the frame id of the incoming map
  map_frame_ = msg->header.frame_id;

  // Replace stored octree (unique_ptr takes ownership)
  octree_.reset(tree);

  // Compute bounds and grid sizes based on occupied leafs
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();

  size_t occupied = 0;
  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (octree_->isNodeOccupied(*it)) {
      auto c = it.getCoordinate();
      double cx = static_cast<double>(c.x());
      double cy = static_cast<double>(c.y());
      double cz = static_cast<double>(c.z());
      min_x = std::min(min_x, cx);
      min_y = std::min(min_y, cy);
      min_z = std::min(min_z, cz);
      max_x = std::max(max_x, cx);
      max_y = std::max(max_y, cy);
      max_z = std::max(max_z, cz);
      ++occupied;
    }
  }

  if (occupied == 0) {
    RCLCPP_WARN(node_->get_logger(), "Received empty octree (no occupied leaves)");
  }

  active_voxel_size_ = octree_->getResolution();
  // If we have occupied leaves, expand bounds slightly to include nearby free space
  if (occupied > 0) {
    double expand = std::max(1.0, 5.0 * active_voxel_size_); // at least 1 meter margin
    min_x -= expand; min_y -= expand; min_z -= expand;
    max_x += expand; max_y += expand; max_z += expand;
    //RCLCPP_INFO(node_->get_logger(), "Expanded octomap bounds by %.3f m to include nearby free space.", expand);
  } else {
    // No occupied leaves — set reasonable default small bounds around origin
    min_x = -1.0; min_y = -1.0; min_z = -1.0;
    max_x =  1.0; max_y =  1.0; max_z =  1.0;
    RCLCPP_WARN(node_->get_logger(), "No occupied leaves: using default small bounds for graph building.");
  }

  min_bound_ = {min_x, min_y, min_z};
  max_bound_ = {max_x, max_y, max_z};

  grid_size_x_ = static_cast<int>(std::ceil((max_x - min_x) / active_voxel_size_)) + 1;
  grid_size_y_ = static_cast<int>(std::ceil((max_y - min_y) / active_voxel_size_)) + 1;
  grid_size_z_ = static_cast<int>(std::ceil((max_z - min_z) / active_voxel_size_)) + 1;

  // RCLCPP_INFO(node_->get_logger(), "Received Octomap in frame: %s, resolution: %.3f, occupied leafs: %zu, grid sizes: [%d x %d x %d]",
  //             map_frame_.c_str(), octree_->getResolution(), occupied, grid_size_x_, grid_size_y_, grid_size_z_);

  // Mark graph dirty so background timer will rebuild connectivity graph
  graph_dirty_ = true;
}

rcl_interfaces::msg::SetParametersResult AstarOctoPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto& parameter : parameters) {
    if (parameter.get_name() == name_ + ".cost_limit") {
      config_.cost_limit = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "New cost limit parameter received via dynamic reconfigure.");
    } else if (parameter.get_name() == name_ + ".z_threshold") {
      z_threshold_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated z_threshold to " << z_threshold_);
    } else if (parameter.get_name() == name_ + ".robot_radius") {
      robot_radius_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated robot_radius to " << robot_radius_);
    } else if (parameter.get_name() == name_ + ".robot_width") {
      robot_width_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated robot_width to " << robot_width_);
    } else if (parameter.get_name() == name_ + ".publish_graph_markers") {
      bool newval = parameter.as_bool();
      if (newval != publish_graph_markers_) {
        publish_graph_markers_ = newval;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Updated publish_graph_markers to " << publish_graph_markers_);
        // if enabling now and graph is already built, immediately publish
        if (publish_graph_markers_) {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          publishGraphMarkers();
        }
      }
    } else if (parameter.get_name() == name_ + ".robot_length") {
      robot_length_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated robot_length to " << robot_length_);
    } else if (parameter.get_name() == name_ + ".robot_height") {
      robot_height_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated robot_height to " << robot_height_);
    } else if (parameter.get_name() == name_ + ".footprint_margin") {
      footprint_margin_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated footprint_margin to " << footprint_margin_);
    } else if (parameter.get_name() == name_ + ".footprint_samples_x") {
      footprint_samples_x_ = parameter.as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated footprint_samples_x to " << footprint_samples_x_);
    } else if (parameter.get_name() == name_ + ".footprint_samples_y") {
      footprint_samples_y_ = parameter.as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated footprint_samples_y to " << footprint_samples_y_);
    } else if (parameter.get_name() == name_ + ".max_surface_distance") {
      max_surface_distance_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated max_surface_distance to " << max_surface_distance_);
    } else if (parameter.get_name() == name_ + ".max_step_height") {
      max_step_height_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated max_step_height to " << max_step_height_);
    } else if (parameter.get_name() == name_ + ".min_vertical_clearance") {
      min_vertical_clearance_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated min_vertical_clearance to " << min_vertical_clearance_);
    } else if (parameter.get_name() == name_ + ".max_vertical_clearance") {
      max_vertical_clearance_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated max_vertical_clearance to " << max_vertical_clearance_);
    }
  }
  result.successful = true;
  return result;
}

} // namespace astar_octo_planner

// The following legacy helpers were removed from the source as they are no longer used
// by the graph-based planner implementation. Their declarations were already removed
// from the public header.
//
// hasNoOccupiedCellsAbove: removed (legacy helper not used)
