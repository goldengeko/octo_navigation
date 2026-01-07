#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace bring_up_alert_nav
{

class OffsetTfPublisher : public rclcpp::Node
{
public:
  OffsetTfPublisher()
  : Node("offset_tf_publisher"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    auto target_frames_param = declare_parameter<std::vector<std::string>>(
      "target_frames", std::vector<std::string>{"Linear_Inspect_KRail"});
    auto offset_frames_param = declare_parameter<std::vector<std::string>>(
      "offset_frames", std::vector<std::string>{"Linear_Inspect_KRail_offset"});
    if (target_frames_param.empty() || target_frames_param.size() != offset_frames_param.size())
    {
      throw rclcpp::exceptions::InvalidParametersException(
              "target_frames and offset_frames must be non-empty and the same length");
    }
    frame_tasks_.reserve(target_frames_param.size());
    for (size_t idx = 0; idx < target_frames_param.size(); ++idx)
    {
      frame_tasks_.push_back(FrameTask{target_frames_param[idx], offset_frames_param[idx]});
    }
    octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_full");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/offset_tf_markers");
    offset_distance_ = declare_parameter<double>("offset_distance", 0.5);
    ray_max_range_ = declare_parameter<double>("ray_max_range", 5.0);
    ray_start_offset_ = declare_parameter<double>("ray_start_offset", 0.0);
    clearance_height_ = declare_parameter<double>("clearance_height", 0.1);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    ignore_unknown_cells_ = declare_parameter<bool>("ignore_unknown_cells", true);
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    marker_scale_ = declare_parameter<double>("marker_scale", 0.12);
    raycast_once_ = declare_parameter<bool>("raycast_once", false);

    map_subscription_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OffsetTfPublisher::mapCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&OffsetTfPublisher::timerCallback, this));

    if (publish_markers_)
    {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    }

    std::ostringstream frame_stream;
    for (size_t i = 0; i < frame_tasks_.size(); ++i)
    {
      if (i > 0)
      {
        frame_stream << ", ";
      }
      frame_stream << frame_tasks_[i].target_frame << "->" << frame_tasks_[i].offset_frame;
    }

    RCLCPP_INFO(
      get_logger(),
      "Offset TF publisher ready: frames=[%s]",
      frame_stream.str().c_str());
  }

private:
  struct FrameTask
  {
    std::string target_frame;
    std::string offset_frame;
    bool has_cached_ground{false};
    double cached_ground_z{0.0};
  };

  struct FrameComputationResult
  {
    geometry_msgs::msg::TransformStamped transform;
    tf2::Vector3 ray_start;
    tf2::Vector3 ray_end;
  };

  void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> raw_tree(octomap_msgs::msgToMap(*msg));
    if (!raw_tree)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to convert Octomap message to tree");
      return;
    }

    auto *as_oc_tree = dynamic_cast<octomap::OcTree *>(raw_tree.release());
    if (!as_oc_tree)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received Octree is not an OcTree");
      return;
    }

    std::lock_guard<std::mutex> lock(octree_mutex_);
    octree_.reset(as_oc_tree);
  }

  std::shared_ptr<octomap::OcTree> getOctreeCopy() const
  {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    return octree_;
  }

  void timerCallback()
  {
    auto tree = getOctreeCopy();
    if (!tree)
    {
      clearMarkers();
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for Octomap...");
      return;
    }

    bool any_success = false;
    std::vector<FrameComputationResult> marker_results;
    if (publish_markers_)
    {
      marker_results.reserve(frame_tasks_.size());
    }

    for (auto &task : frame_tasks_)
    {
      auto computation = computeFrameTask(task, tree);
      if (!computation)
      {
        continue;
      }

      any_success = true;
      tf_broadcaster_->sendTransform(computation->transform);
      if (publish_markers_)
      {
        marker_results.push_back(*computation);
      }
    }

    if (publish_markers_)
    {
      if (any_success)
      {
        publishMarkers(marker_results);
      }
      else
      {
        clearMarkers();
      }
    }
  }

  visualization_msgs::msg::Marker createPointMarker(const tf2::Vector3 &position, int id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = now();
    marker.ns = marker_ns_;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = marker_scale_;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    return marker;
  }

  void clearMarkers()
  {
    if (!publish_markers_ || !marker_pub_)
    {
      return;
    }
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = now();
    marker.ns = marker_ns_;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(marker);
    marker_pub_->publish(arr);
  }

  std::optional<FrameComputationResult> computeFrameTask(
    FrameTask &task,
    const std::shared_ptr<octomap::OcTree> &tree)
  {
    geometry_msgs::msg::TransformStamped target_tf;
    try
    {
      target_tf = tf_buffer_.lookupTransform(map_frame_, task.target_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF lookup failed for frame '%s': %s", task.target_frame.c_str(), ex.what());
      return std::nullopt;
    }

    tf2::Transform tf_map_target;
    tf2::fromMsg(target_tf.transform, tf_map_target);

    tf2::Vector3 forward = tf_map_target.getBasis().getColumn(0);
    if (forward.length2() < 1e-6)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target frame '%s' has invalid orientation", task.target_frame.c_str());
      return std::nullopt;
    }
    forward.normalize();

    tf2::Vector3 offset_point = tf_map_target.getOrigin() + offset_distance_ * forward;
    double start_z = target_tf.transform.translation.z + ray_start_offset_;
    tf2::Vector3 ray_origin(offset_point.x(), offset_point.y(), start_z);

    double ground_z = 0.0;
    tf2::Vector3 ray_end;
    bool used_cached_ground = raycast_once_ && task.has_cached_ground;
    if (used_cached_ground)
    {
      ground_z = task.cached_ground_z;
      ray_end = tf2::Vector3(offset_point.x(), offset_point.y(), ground_z);
    }
    else
    {
      octomap::point3d octo_origin(ray_origin.x(), ray_origin.y(), ray_origin.z());
      octomap::point3d direction(0.0f, 0.0f, -1.0f);
      octomap::point3d hit_point;

      bool hit = tree->castRay(octo_origin, direction, hit_point, ignore_unknown_cells_, ray_max_range_);
      if (!hit)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Raycast failed: no occupied cell below point (frame %s)", task.target_frame.c_str());
        return std::nullopt;
      }

      ground_z = hit_point.z();
      ray_end = tf2::Vector3(hit_point.x(), hit_point.y(), hit_point.z());

      if (raycast_once_)
      {
        task.has_cached_ground = true;
        task.cached_ground_z = ground_z;
      }
    }

    FrameComputationResult result;
    result.transform.header.stamp = now();
    result.transform.header.frame_id = map_frame_;
    result.transform.child_frame_id = task.offset_frame;
    result.transform.transform.translation.x = offset_point.x();
    result.transform.transform.translation.y = offset_point.y();
    result.transform.transform.translation.z = ground_z + clearance_height_;
    result.transform.transform.rotation = target_tf.transform.rotation;
    result.ray_start = ray_origin;
    result.ray_end = ray_end;

    return result;
  }

  void publishMarkers(const std::vector<FrameComputationResult> &results)
  {
    if (!publish_markers_ || !marker_pub_)
    {
      return;
    }

    if (results.empty())
    {
      clearMarkers();
      return;
    }

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto &result : results)
    {
      arr.markers.push_back(createPointMarker(result.ray_start, id++));
      arr.markers.push_back(createPointMarker(result.ray_end, id++));
    }
    marker_pub_->publish(arr);
  }

  // Parameters / configuration
  std::string map_frame_;
  std::string octomap_topic_;
  std::string marker_topic_;
  double offset_distance_{};
  double ray_max_range_{};
  double ray_start_offset_{};
  double clearance_height_{};
  double publish_rate_hz_{};
  bool ignore_unknown_cells_{};
  bool publish_markers_{};
  double marker_scale_{};
  const std::string marker_ns_ {"offset_tf_raycast"};
  std::vector<FrameTask> frame_tasks_;
  bool raycast_once_{};

  mutable std::mutex octree_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace bring_up_alert_nav

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bring_up_alert_nav::OffsetTfPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

