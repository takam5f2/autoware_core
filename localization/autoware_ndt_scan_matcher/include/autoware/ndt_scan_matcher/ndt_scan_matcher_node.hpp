// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_NODE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_NODE_HPP_

#define FMT_HEADER_ONLY

#include "diagnostics_report.hpp"
#include "hyper_parameters.hpp"
#include "map_update_module.hpp"
#include "ndt_scan_matcher.hpp"

#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using DiagnosticsInterface = autoware_utils_diagnostics::DiagnosticsInterface;

class NdtScanMatcherNode : public rclcpp::Node
{
public:
  explicit NdtScanMatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // This function is only used in static tools to know when timer callbacks are triggered.
  std::chrono::nanoseconds time_until_trigger() const
  {
    return map_update_timer_->time_until_trigger();
  }

private:
  void callback_timer();

  void callback_initial_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr);

  void callback_regularization_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);

  void callback_sensor_points(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame);
  // The TF the scan match needs, looked up here because the module holds no TF buffer.
  [[nodiscard]] NdtScanMatcher::TransformLookup lookup_base_from_sensor(
    const std::string & sensor_frame);

  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  void service_ndt_align(
    const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr
      req,
    autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);
  void service_ndt_align_main(
    const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr
      req,
    autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);

  void publish_scan_matching_output(const NdtScanMatcher::ScanMatchingOutput & output);

  MapUpdateModule::GetDifferentialPointCloudMap::Response::SharedPtr
  get_differential_point_cloud_map(
    const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request);

  // Emits the log lines a core module recorded, one `case` per site.
  void replay_logs(const std::vector<LogRequest> & logs);

  // Forwards a diagnostics update produced by a core module to the given DiagnosticsInterface.
  static void apply_diagnostics_update(
    DiagnosticsInterface & diagnostics, const DiagnosticsReport & report);

  void publish_loaded_map_if_present(
    const std::optional<sensor_msgs::msg::PointCloud2> & loaded_pcd_map,
    const std::optional<rclcpp::Time> & stamp = std::nullopt) const;

  rclcpp::TimerBase::SharedPtr map_update_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    regularization_pose_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_ground_points_aligned_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ndt_pose_with_covariance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_with_covariance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr multi_ndt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr multi_initial_pose_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    transform_probability_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    nearest_voxel_transformation_likelihood_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_score_points_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    no_ground_transform_probability_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    no_ground_nearest_voxel_transformation_likelihood_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr iteration_num_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    initial_to_result_relative_pose_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_old_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_new_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ndt_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ndt_monte_carlo_initial_pose_marker_pub_;

  // Debug publisher and pcd loader client used by MapUpdateModule (kept on the ROS node side).
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_pcd_pub_;
  rclcpp::Client<MapUpdateModule::GetDifferentialPointCloudMap>::SharedPtr pcd_loader_client_;

  rclcpp::Service<autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped>::SharedPtr
    service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::unique_ptr<DiagnosticsInterface> diagnostics_scan_points_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_initial_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_regularization_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_map_update_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_ndt_align_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_trigger_node_;

  // Everything this node is a node for. It owns the NDT, its lock, and the modules; this class
  // owns only the ROS side of each callback.
  std::unique_ptr<NdtScanMatcher> matcher_;
  std::unique_ptr<autoware_utils_logging::LoggerLevelConfigure> logger_configure_;

  HyperParameters param_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_NODE_HPP_
