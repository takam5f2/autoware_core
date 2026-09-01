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

#include "ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_node.hpp>
#include <autoware/qos_utils/qos_compatibility.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_pcl/transforms.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <iomanip>
#include <map>
#include <thread>
#include <utility>
#include <variant>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;

using autoware::localization_util::TreeStructuredParzenEstimator;
using autoware_utils_diagnostics::DiagnosticsInterface;

autoware_internal_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = autoware_internal_debug_msgs::msg::Float32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

autoware_internal_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = autoware_internal_debug_msgs::msg::Int32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

NdtScanMatcherNode::NdtScanMatcherNode(const rclcpp::NodeOptions & options)
: Node("ndt_scan_matcher", options),
  tf2_broadcaster_(*this),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_),
  is_activated_(false),
  param_(this)
{
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;
  auto sensor_sub_opt = rclcpp::SubscriptionOptions();
  sensor_sub_opt.callback_group = sensor_callback_group;

  constexpr double map_update_dt = 1.0;
  constexpr auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&NdtScanMatcherNode::callback_timer, this),
    timer_callback_group_);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 10,
    std::bind(&NdtScanMatcherNode::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&NdtScanMatcherNode::callback_sensor_points, this, std::placeholders::_1),
    sensor_sub_opt);

  // Only if regularization is enabled, subscribe to the regularization base pose
  if (param_.ndt_regularization_enable) {
    // NOTE: The reason that the regularization subscriber does not belong to the
    // sensor_callback_group is to ensure that the regularization callback is called even if
    // sensor_callback takes long time to process.
    // Both callback_initial_pose and callback_regularization_pose must not miss receiving data for
    // proper interpolation.
    regularization_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "regularization_pose_with_covariance", 10,
        std::bind(&NdtScanMatcherNode::callback_regularization_pose, this, std::placeholders::_1),
        initial_pose_sub_opt);

    diagnostics_regularization_pose_ =
      std::make_unique<DiagnosticsInterface>(this, "regularization_pose_subscriber_status");
  }

  sensor_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  no_ground_points_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned_no_ground", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose_with_covariance", 10);
  multi_ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("multi_ndt_pose", 10);
  multi_initial_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("multi_initial_pose", 10);
  exe_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>("exe_time_ms", 10);
  transform_probability_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "transform_probability", 10);
  nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "nearest_voxel_transformation_likelihood", 10);
  voxel_score_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_score_points", 10);
  no_ground_transform_probability_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "no_ground_transform_probability", 10);
  no_ground_nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "no_ground_nearest_voxel_transformation_likelihood", 10);
  iteration_num_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>("iteration_num", 10);
  initial_to_result_relative_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_to_result_relative_pose", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  service_ =
    this->create_service<autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped>(
      "ndt_align_srv",
      std::bind(
        &NdtScanMatcherNode::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
      AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE(), sensor_callback_group);
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &NdtScanMatcherNode::service_trigger_node, this, std::placeholders::_1,
      std::placeholders::_2),
    AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE(), sensor_callback_group);

  // ROS-dependent resources for the map update module are owned by this node.
  loaded_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());
  pcd_loader_client_ =
    this->create_client<MapUpdateModule::GetDifferentialPointCloudMap>("pcd_loader_service");

  matcher_ = std::make_unique<NdtScanMatcher>(
    param_.to_core_params(),
    [this](const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request) {
      return this->get_differential_point_cloud_map(request);
    });

  diagnostics_scan_points_ = std::make_unique<DiagnosticsInterface>(this, "scan_matching_status");
  diagnostics_initial_pose_ =
    std::make_unique<DiagnosticsInterface>(this, "initial_pose_subscriber_status");
  diagnostics_map_update_ = std::make_unique<DiagnosticsInterface>(this, "map_update_status");
  diagnostics_ndt_align_ = std::make_unique<DiagnosticsInterface>(this, "ndt_align_service_status");
  diagnostics_trigger_node_ =
    std::make_unique<DiagnosticsInterface>(this, "trigger_node_service_status");

  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);
}

MapUpdateModule::GetDifferentialPointCloudMap::Response::SharedPtr
NdtScanMatcherNode::get_differential_point_cloud_map(
  const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request)
{
  if (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) || !rclcpp::ok()) {
    return nullptr;
  }

  // send a request to map_loader
  auto result{pcd_loader_client_->async_send_request(
    request, [](rclcpp::Client<MapUpdateModule::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    if (!rclcpp::ok()) {
      return nullptr;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }

  return result.get();
}

void NdtScanMatcherNode::replay_logs(const std::vector<LogRequest> & logs)
{
  // One `case` per site, so that each keeps the severity it had -- and, for the throttled sites
  // added later, its own macro expansion and therefore its own throttle clock.
  for (const auto & log : logs) {
    switch (log.site) {
      case LogSite::PoseBufferTooFewSamples:
      case LogSite::PoseBufferStampMismatch:
        RCLCPP_INFO_STREAM(this->get_logger(), log.message);
        break;
      case LogSite::PoseBufferTimeoutViolation:
      case LogSite::PoseBufferDistanceViolation:
      case LogSite::AlignUnstableScore:
        RCLCPP_WARN_STREAM(this->get_logger(), log.message);
        break;
      case LogSite::ScanTransformFailed:
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case LogSite::ScanOutOfMapRange:
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case LogSite::ScanScoreBelowThreshold:
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case LogSite::AlignNoInputTarget:
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case LogSite::AlignNoInputSource:
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, log.message);
        break;
      case LogSite::AlignPoseInput:
      case LogSite::AlignPoseOutput:
        RCLCPP_DEBUG_STREAM(this->get_logger(), log.message);
        break;
    }
  }
}

void NdtScanMatcherNode::apply_diagnostics_update(
  DiagnosticsInterface & diagnostics, const DiagnosticsReport & report)
{
  for (const auto & key_value : report.key_values) {
    std::visit(
      [&](const auto & value) { diagnostics.add_key_value(key_value.key, value); },
      key_value.value);
  }
  diagnostics.update_level_and_message(static_cast<int8_t>(report.level), report.message);
}

void NdtScanMatcherNode::publish_loaded_map_if_present(
  const std::optional<sensor_msgs::msg::PointCloud2> & loaded_pcd_map,
  const std::optional<rclcpp::Time> & stamp) const
{
  if (loaded_pcd_map.has_value()) {
    auto msg = *loaded_pcd_map;
    msg.header.stamp = stamp ? *stamp : this->now();
    loaded_pcd_pub_->publish(msg);
  }
}

void NdtScanMatcherNode::callback_timer()
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_map_update_->clear();

  diagnostics_map_update_->add_key_value("timer_callback_time_stamp", ros_time_now.nanoseconds());

  // check is_activated
  diagnostics_map_update_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  if (!is_activated_) {
    diagnostics_map_update_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Node is not activated.");
    diagnostics_map_update_->publish(ros_time_now);
    return;
  }

  auto result = matcher_->update_map_periodically();
  apply_diagnostics_update(*diagnostics_map_update_, result.diagnostics);

  publish_loaded_map_if_present(result.loaded_pcd_map, ros_time_now);
  diagnostics_map_update_->publish(ros_time_now);
}

void NdtScanMatcherNode::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->clear();

  const auto report = matcher_->push_initial_pose(initial_pose_msg_ptr, is_activated_);
  replay_logs(report.logs);
  apply_diagnostics_update(*diagnostics_initial_pose_, report);

  diagnostics_initial_pose_->publish(initial_pose_msg_ptr->header.stamp);
}

void NdtScanMatcherNode::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  diagnostics_regularization_pose_->clear();

  const auto report = matcher_->push_regularization_pose(pose_conv_msg_ptr);
  replay_logs(report.logs);
  apply_diagnostics_update(*diagnostics_regularization_pose_, report);

  diagnostics_regularization_pose_->publish(pose_conv_msg_ptr->header.stamp);
}

void NdtScanMatcherNode::callback_sensor_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  // clear diagnostics
  diagnostics_scan_points_->clear();

  // The keys, the level, the message and the log records all come back on one report rather than
  // being written as they are produced, so that they are forwarded and replayed from one place no
  // matter which of the eight gates returned.
  NdtScanMatcher::ScanInput input;
  input.scan = sensor_points_msg_in_sensor_frame;
  input.now = this->now();
  input.is_activated = is_activated_;
  input.base_from_sensor =
    lookup_base_from_sensor(sensor_points_msg_in_sensor_frame->header.frame_id);
  input.voxel_score_points_wanted = voxel_score_points_pub_->get_subscription_count() > 0;

  auto match = matcher_->match_scan(input);
  replay_logs(match.diagnostics.logs);
  if (match.output) {
    publish_scan_matching_output(*match.output);
  }
  apply_diagnostics_update(*diagnostics_scan_points_, match.diagnostics);
  diagnostics_scan_points_->publish(sensor_points_msg_in_sensor_frame->header.stamp);
}

NdtScanMatcher::TransformLookup NdtScanMatcherNode::lookup_base_from_sensor(
  const std::string & sensor_frame)
{
  NdtScanMatcher::TransformLookup lookup;
  if (sensor_frame == param_.frame.base_frame) {
    // The module short-circuits on equal frames, but it still needs a transform to be present in
    // order to get that far.
    lookup.transform = geometry_msgs::msg::TransformStamped();
    return lookup;
  }
  try {
    lookup.transform =
      tf2_buffer_.lookupTransform(param_.frame.base_frame, sensor_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    lookup.error = ex.what();
  }
  return lookup;
}

void NdtScanMatcherNode::publish_scan_matching_output(
  const NdtScanMatcher::ScanMatchingOutput & output)
{
  // `if (opt)` and nothing else. See ScanMatchingOutput's comment for why.
  if (output.multi_ndt_pose) {
    multi_ndt_pose_pub_->publish(*output.multi_ndt_pose);
  }
  if (output.multi_initial_pose) {
    multi_initial_pose_pub_->publish(*output.multi_initial_pose);
  }

  initial_pose_with_covariance_pub_->publish(output.initial_pose_with_covariance);
  exe_time_pub_->publish(make_float32_stamped(output.stamp, output.exe_time_ms));
  transform_probability_pub_->publish(
    make_float32_stamped(output.stamp, output.transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    make_float32_stamped(output.stamp, output.nearest_voxel_transformation_likelihood));
  iteration_num_pub_->publish(make_int32_stamped(output.stamp, output.iteration_num));

  tf2_broadcaster_.sendTransform(output.tf);

  if (output.ndt_pose) {
    ndt_pose_pub_->publish(*output.ndt_pose);
  }
  if (output.ndt_pose_with_covariance) {
    ndt_pose_with_covariance_pub_->publish(*output.ndt_pose_with_covariance);
  }

  ndt_marker_pub_->publish(output.ndt_marker);

  initial_to_result_relative_pose_pub_->publish(output.initial_to_result_relative_pose);
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(output.stamp, output.initial_to_result_distance));
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(output.stamp, output.initial_to_result_distance_old));
  initial_to_result_distance_new_pub_->publish(
    make_float32_stamped(output.stamp, output.initial_to_result_distance_new));

  sensor_aligned_pose_pub_->publish(output.points_aligned);

  if (output.voxel_score_points) {
    voxel_score_points_pub_->publish(*output.voxel_score_points);
  }

  if (output.no_ground) {
    no_ground_points_aligned_pose_pub_->publish(output.no_ground->points);
    no_ground_transform_probability_pub_->publish(
      make_float32_stamped(output.stamp, output.no_ground->transform_probability));
    no_ground_nearest_voxel_transformation_likelihood_pub_->publish(make_float32_stamped(
      output.stamp, output.no_ground->nearest_voxel_transformation_likelihood));
  }
}

void NdtScanMatcherNode::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_trigger_node_->clear();
  diagnostics_trigger_node_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  is_activated_ = req->data;
  if (is_activated_) {
    matcher_->clear_initial_pose_buffer();
  }
  res->success = true;

  diagnostics_trigger_node_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  diagnostics_trigger_node_->add_key_value("is_succeed_service", res->success);
  diagnostics_trigger_node_->publish(ros_time_now);
}

void NdtScanMatcherNode::service_ndt_align(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_ndt_align_->clear();

  diagnostics_ndt_align_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  service_ndt_align_main(req, res);

  // check is_succeed_service
  bool is_succeed_service = res->success;
  diagnostics_ndt_align_->add_key_value("is_succeed_service", is_succeed_service);
  if (!is_succeed_service) {
    std::stringstream message;
    message << "ndt_align_service is failed.";
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  diagnostics_ndt_align_->publish(ros_time_now);
}

void NdtScanMatcherNode::service_ndt_align_main(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  // get TF from pose_frame to map_frame
  const std::string & target_frame = param_.frame.map_frame;
  const std::string & source_frame = req->pose_with_covariance.header.frame_id;

  geometry_msgs::msg::TransformStamped transform_s2t;
  try {
    transform_s2t = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    // Note: Up to AWSIMv1.1.0, there is a known bug where the GNSS frame_id is incorrectly set to
    // "gnss_link" instead of "map". The ndt_align is designed to return identity when this issue
    // occurs. However, in the future, converting to a non-existent frame_id should be prohibited.

    diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", false);

    std::stringstream message;
    message << "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str();
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    res->success = false;
    return;
  }
  diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", true);

  // transform pose_frame to map_frame
  auto initial_pose_msg_in_map_frame =
    autoware::localization_util::transform(req->pose_with_covariance, transform_s2t);
  initial_pose_msg_in_map_frame.header.stamp = req->pose_with_covariance.header.stamp;

  auto align_result = matcher_->align({initial_pose_msg_in_map_frame, this->now()});
  replay_logs(align_result.diagnostics.logs);
  publish_loaded_map_if_present(align_result.loaded_pcd_map);
  if (align_result.search_markers) {
    ndt_monte_carlo_initial_pose_marker_pub_->publish(*align_result.search_markers);
  }
  if (align_result.best_points_aligned) {
    sensor_aligned_pose_pub_->publish(*align_result.best_points_aligned);
  }
  apply_diagnostics_update(*diagnostics_ndt_align_, align_result.diagnostics);

  if (!align_result.estimate) {
    res->success = false;
    return;
  }

  res->reliable = align_result.estimate->reliable;
  res->success = true;
  res->pose_with_covariance = align_result.estimate->pose_with_covariance;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
}

}  // namespace autoware::ndt_scan_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ndt_scan_matcher::NdtScanMatcherNode)
