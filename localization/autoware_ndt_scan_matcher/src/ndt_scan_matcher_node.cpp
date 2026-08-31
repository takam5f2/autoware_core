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
#include <autoware/ndt_scan_matcher/particle.hpp>
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
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<PoseInterpolationBuffer>(value_as_unlimited, value_as_unlimited);

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

  ndt_ptr_.with([&](const auto & ndt_ptr) { ndt_ptr->setParams(param_.ndt); });

  initial_pose_buffer_ = std::make_unique<PoseInterpolationBuffer>(
    param_.validation.initial_pose_timeout_sec,
    param_.validation.initial_pose_distance_tolerance_m);

  // ROS-dependent resources for the map update module are owned by this node.
  loaded_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());
  pcd_loader_client_ =
    this->create_client<MapUpdateModule::GetDifferentialPointCloudMap>("pcd_loader_service");

  map_update_module_ = std::make_unique<MapUpdateModule>(
    ndt_ptr_, param_.dynamic_map_loading,
    [this](const MapUpdateModule::GetDifferentialPointCloudMap::Request::SharedPtr & request) {
      return this->get_differential_point_cloud_map(request);
    });

  pose_initialization_params_ = PoseInitializationSearch::Params{
    param_.initial_pose_estimation.particles_num, param_.initial_pose_estimation.n_startup_trials,
    param_.frame.map_frame,
    param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood};

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

void NdtScanMatcherNode::publish_pose_initialization_progress(
  const PoseInitializationSearch::Progress & progress)
{
  // publish the estimated poses in 20 times to see the progress and to avoid dropping data
  constexpr int64_t publish_num = 20;
  const int64_t publish_interval =
    std::max<int64_t>(param_.initial_pose_estimation.particles_num / publish_num, 1);

  push_debug_markers(
    monte_carlo_marker_array_, get_clock()->now(), param_.frame.map_frame, progress.particle,
    progress.index);
  if (
    (progress.index + 1) % publish_interval == 0 ||
    (progress.index + 1) == param_.initial_pose_estimation.particles_num) {
    ndt_monte_carlo_initial_pose_marker_pub_->publish(monte_carlo_marker_array_);
    monte_carlo_marker_array_.markers.clear();
  }

  sensor_aligned_pose_pub_->publish(progress.sensor_points_in_map);
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
  const MapUpdateModule::UpdateResult & result, const std::optional<rclcpp::Time> & stamp) const
{
  if (result.loaded_pcd_map.has_value()) {
    auto msg = result.loaded_pcd_map.value();
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

  // check is_set_last_update_position
  const auto latest_ekf_position = latest_ekf_position_.with([](const auto & pos) { return pos; });
  const bool is_set_last_update_position = (latest_ekf_position != std::nullopt);
  diagnostics_map_update_->add_key_value(
    "is_set_last_update_position", is_set_last_update_position);
  if (!is_set_last_update_position) {
    diagnostics_map_update_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Cannot find the reference position for map update."
      "Please check if the EKF odometry is provided to NDT.");
    diagnostics_map_update_->publish(ros_time_now);
    return;
  }

  auto result = map_update_module_->callback_timer(latest_ekf_position.value());
  apply_diagnostics_update(*diagnostics_map_update_, result.diagnostics);

  publish_loaded_map_if_present(result, ros_time_now);
  diagnostics_map_update_->publish(ros_time_now);
}

void NdtScanMatcherNode::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->clear();

  callback_initial_pose_main(initial_pose_msg_ptr);

  diagnostics_initial_pose_->publish(initial_pose_msg_ptr->header.stamp);
}

void NdtScanMatcherNode::callback_initial_pose_main(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->add_key_value(
    "topic_time_stamp",
    static_cast<rclcpp::Time>(initial_pose_msg_ptr->header.stamp).nanoseconds());

  // check is_activated
  diagnostics_initial_pose_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  if (!is_activated_) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // check is_expected_frame_id
  const bool is_expected_frame_id =
    (initial_pose_msg_ptr->header.frame_id == param_.frame.map_frame);
  diagnostics_initial_pose_->add_key_value("is_expected_frame_id", is_expected_frame_id);
  if (!is_expected_frame_id) {
    std::stringstream message;
    message << "Received initial pose message with frame_id "
            << initial_pose_msg_ptr->header.frame_id << ", but expected " << param_.frame.map_frame
            << ". Please check the frame_id in the input topic and ensure it is correct.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    return;
  }

  initial_pose_buffer_->push_back(initial_pose_msg_ptr);

  latest_ekf_position_.with([&](auto & pos) { pos = initial_pose_msg_ptr->pose.pose.position; });
}

void NdtScanMatcherNode::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  diagnostics_regularization_pose_->clear();

  diagnostics_regularization_pose_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(pose_conv_msg_ptr->header.stamp).nanoseconds());

  regularization_pose_buffer_->push_back(pose_conv_msg_ptr);

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
  DiagnosticsReport report;
  const bool is_succeed_scan_matching =
    callback_sensor_points_main(sensor_points_msg_in_sensor_frame, report);
  replay_logs(report.logs);

  // check skipping_publish_num
  static int64_t skipping_publish_num = 0;
  skipping_publish_num =
    ((is_succeed_scan_matching || !is_activated_) ? 0 : (skipping_publish_num + 1));
  report.add_key_value({"skipping_publish_num", skipping_publish_num});
  if (skipping_publish_num >= param_.validation.skipping_publish_num) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit (" << skipping_publish_num << " times).";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }

  apply_diagnostics_update(*diagnostics_scan_points_, report);
  diagnostics_scan_points_->publish(sensor_points_msg_in_sensor_frame->header.stamp);
}

bool NdtScanMatcherNode::callback_sensor_points_main(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame,
  DiagnosticsReport & report)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  // check topic_time_stamp
  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;
  report.add_key_value({"topic_time_stamp", sensor_ros_time.nanoseconds()});

  // check sensor_points_size
  const size_t sensor_points_size = sensor_points_msg_in_sensor_frame->width;
  report.add_key_value({"sensor_points_size", static_cast<int64_t>(sensor_points_size)});
  if (sensor_points_size == 0) {
    std::stringstream message;
    message << "Sensor points is empty.";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return false;
  }

  // check sensor_points_delay_time_sec
  const double sensor_points_delay_time_sec =
    (this->now() - sensor_points_msg_in_sensor_frame->header.stamp).seconds();
  report.add_key_value({"sensor_points_delay_time_sec", sensor_points_delay_time_sec});
  if (sensor_points_delay_time_sec > param_.sensor_points.timeout_sec) {
    std::stringstream message;
    message << "sensor points is experiencing latency."
            << "The delay time is " << sensor_points_delay_time_sec << "[sec] "
            << "(the tolerance is " << param_.sensor_points.timeout_sec << "[sec]).";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());

    // If the delay time of the LiDAR topic exceeds the delay compensation time of ekf_localizer,
    // even if further processing continues, the estimated result will be rejected by ekf_localizer.
    // Therefore, it would be acceptable to exit the function here.
    // However, for now, we will continue the processing as it is.

    // return false;
  }

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;

  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link
  try {
    transform_sensor_measurement(
      sensor_frame, param_.frame.base_frame, sensor_points_in_sensor_frame,
      sensor_points_in_baselink_frame);
  } catch (const std::exception & ex) {
    std::stringstream message;
    message << ex.what() << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    report.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    report.logs.push_back({LogSite::ScanTransformFailed, message.str()});
    report.add_key_value({"is_succeed_transform_sensor_points", false});
    return false;
  }
  report.add_key_value({"is_succeed_transform_sensor_points", true});

  // check sensor_points_max_distance
  double max_distance = 0.0;
  for (const auto & point : sensor_points_in_baselink_frame->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }

  report.add_key_value({"sensor_points_max_distance", max_distance});
  if (max_distance < param_.sensor_points.required_distance) {
    std::stringstream message;
    message << "Max distance of sensor points = " << std::fixed << std::setprecision(3)
            << max_distance << " [m] < " << param_.sensor_points.required_distance << " [m]";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return false;
  }

  // Filled inside the lock, published once it is released. The align service shares the sensor
  // callback group with this one, so nothing else can reach the NDT in between; the map update
  // timer can, and swapping the map while already-built messages go out is harmless.
  std::optional<ScanMatchingOutput> scan_matching_output;

  const bool is_succeed = ndt_ptr_.with([&](const auto & ndt_ptr) {
    // store sensor points for ndt alignment
    sensor_points_in_baselink_frame_ = sensor_points_in_baselink_frame;

    if (!report.check(
          "is_activated", is_activated_, DiagnosticLevel::WARN, "Node is not activated.")) {
      return false;
    }

    // calculate initial pose
    std::vector<LogRequest> interpolation_logs;
    std::optional<PoseInterpolationBuffer::InterpolateResult> interpolation_result_opt =
      initial_pose_buffer_->interpolate(sensor_ros_time, interpolation_logs);
    replay_logs(interpolation_logs);

    if (!report.check(
          "is_succeed_interpolate_initial_pose", interpolation_result_opt != std::nullopt,
          DiagnosticLevel::WARN,
          "Couldn't interpolate pose. Please verify that "
          "(1) the initial pose topic (primarily come from the EKF) is being published, and "
          "(2) the timestamps of the sensor PCD messages and pose messages are synchronized "
          "correctly.")) {
      return false;
    }

    initial_pose_buffer_->pop_old(sensor_ros_time);
    const PoseInterpolationBuffer::InterpolateResult & interpolation_result =
      interpolation_result_opt.value();

    // if regularization is enabled and available, set pose to NDT for regularization
    if (param_.ndt_regularization_enable) {
      add_regularization_pose(sensor_ros_time, *ndt_ptr);
    }

    // Warn if the lidar has gone out of the map range
    if (map_update_module_->out_of_map_range(
          interpolation_result.interpolated_pose.pose.pose.position)) {
      std::stringstream msg;

      msg << "Lidar has gone out of the map range";
      report.update_level_and_message(DiagnosticLevel::WARN, msg.str());
      report.logs.push_back({LogSite::ScanOutOfMapRange, msg.str()});
    }

    if (!report.check(
          "is_set_map_points", ndt_ptr->hasTarget(), DiagnosticLevel::WARN,
          "Map points is not set.")) {
      return false;
    }

    // perform ndt scan matching
    const Eigen::Matrix4f initial_pose_matrix =
      pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
    auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
    ndt_ptr->align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame_);
    const pclomp::NdtResult ndt_result = ndt_ptr->getResult();

    const geometry_msgs::msg::Pose result_pose_msg = matrix4f_to_pose(ndt_result.pose);
    std::vector<geometry_msgs::msg::Pose> transformation_msg_array;
    for (const auto & pose_matrix : ndt_result.transformation_array) {
      geometry_msgs::msg::Pose pose_ros = matrix4f_to_pose(pose_matrix);
      transformation_msg_array.push_back(pose_ros);
    }

    // check iteration_num
    report.add_key_value({"iteration_num", static_cast<int64_t>(ndt_result.iteration_num)});
    const bool is_ok_iteration_num = (ndt_result.iteration_num < ndt_ptr->getMaximumIterations());
    if (!is_ok_iteration_num) {
      std::stringstream message;
      message << "The number of iterations has reached its upper limit. The number of iterations: "
              << ndt_result.iteration_num << ", Limit: " << ndt_ptr->getMaximumIterations() << ".";
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    }

    // check local_optimal_solution_oscillation_num
    constexpr int oscillation_num_threshold = 10;
    const int oscillation_num = count_oscillation(transformation_msg_array);
    report.add_key_value(
      {"local_optimal_solution_oscillation_num", static_cast<int64_t>(oscillation_num)});
    const bool is_local_optimal_solution_oscillation =
      (oscillation_num > oscillation_num_threshold);
    if (is_local_optimal_solution_oscillation) {
      std::stringstream message;
      message << "There is a possibility of oscillation in a local minimum";
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    }

    // check score
    report.add_key_value({"transform_probability", ndt_result.transform_probability});
    report.add_key_value(
      {"nearest_voxel_transformation_likelihood",
       ndt_result.nearest_voxel_transformation_likelihood});
    double score = 0.0;
    double score_threshold = 0.0;
    if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
      score = ndt_result.transform_probability;
      score_threshold = param_.score_estimation.converged_param_transform_probability;
    } else if (
      param_.score_estimation.converged_param_type ==
      ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
      score = ndt_result.nearest_voxel_transformation_likelihood;
      score_threshold =
        param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
    } else {
      std::stringstream message;
      message
        << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
      report.update_level_and_message(DiagnosticLevel::ERROR, message.str());
      return false;
    }

    // check score diff
    const std::vector<float> & tp_array = ndt_result.transform_probability_array;
    if (static_cast<int>(tp_array.size()) != ndt_result.iteration_num + 1) {
      // only publish warning to /diagnostics, not skip publishing pose
      std::stringstream message;
      message << "transform_probability_array size is not equal to iteration_num + 1."
              << " transform_probability_array size: " << tp_array.size()
              << ", iteration_num: " << ndt_result.iteration_num;
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    } else {
      const float diff = tp_array.back() - tp_array.front();
      report.add_key_value({"transform_probability_diff", diff});
      report.add_key_value({"transform_probability_before", tp_array.front()});
    }
    const std::vector<float> & nvtl_array =
      ndt_result.nearest_voxel_transformation_likelihood_array;
    if (static_cast<int>(nvtl_array.size()) != ndt_result.iteration_num + 1) {
      // only publish warning to /diagnostics, not skip publishing pose
      std::stringstream message;
      message
        << "nearest_voxel_transformation_likelihood_array size is not equal to iteration_num + 1."
        << " nearest_voxel_transformation_likelihood_array size: " << nvtl_array.size()
        << ", iteration_num: " << ndt_result.iteration_num;
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    } else {
      const float diff = nvtl_array.back() - nvtl_array.front();
      report.add_key_value({"nearest_voxel_transformation_likelihood_diff", diff});
      report.add_key_value({"nearest_voxel_transformation_likelihood_before", nvtl_array.front()});
    }

    bool is_ok_score = (score > score_threshold);
    if (!is_ok_score) {
      std::stringstream message;
      message << "Score is below the threshold. Score: " << score
              << ", Threshold: " << score_threshold;
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
      report.logs.push_back({LogSite::ScanScoreBelowThreshold, message.str()});
    }

    // check is_converged
    bool is_converged =
      (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;

    // covariance estimation
    CovarianceEstimate covariance_debug;
    const Eigen::Quaterniond map_to_base_link_quat = Eigen::Quaterniond(
      result_pose_msg.orientation.w, result_pose_msg.orientation.x, result_pose_msg.orientation.y,
      result_pose_msg.orientation.z);
    const Eigen::Matrix3d map_to_base_link_rotation =
      map_to_base_link_quat.normalized().toRotationMatrix();

    std::array<double, 36> ndt_covariance =
      rotate_covariance(param_.covariance.output_pose_covariance, map_to_base_link_rotation);
    if (
      param_.covariance.covariance_estimation.covariance_estimation_type !=
      CovarianceEstimationType::FIXED_VALUE) {
      CovarianceEstimate covariance_estimate = estimate_covariance(
        ndt_result, initial_pose_matrix, sensor_ros_time, *ndt_ptr,
        sensor_points_in_baselink_frame_);
      covariance_debug.multi_ndt_pose = std::move(covariance_estimate.multi_ndt_pose);
      covariance_debug.multi_initial_pose = std::move(covariance_estimate.multi_initial_pose);
      const Eigen::Matrix2d estimated_covariance_2d = covariance_estimate.covariance;
      const Eigen::Matrix2d estimated_covariance_2d_scaled =
        estimated_covariance_2d * param_.covariance.covariance_estimation.scale_factor;
      const double default_cov_xx = param_.covariance.output_pose_covariance[0];
      const double default_cov_yy = param_.covariance.output_pose_covariance[7];
      const Eigen::Matrix2d estimated_covariance_2d_adj = pclomp::adjust_diagonal_covariance(
        estimated_covariance_2d_scaled, ndt_result.pose, default_cov_xx, default_cov_yy);
      ndt_covariance[0 + 6 * 0] = estimated_covariance_2d_adj(0, 0);
      ndt_covariance[1 + 6 * 1] = estimated_covariance_2d_adj(1, 1);
      ndt_covariance[1 + 6 * 0] = estimated_covariance_2d_adj(1, 0);
      ndt_covariance[0 + 6 * 1] = estimated_covariance_2d_adj(0, 1);
    }

    // check distance_initial_to_result
    const auto distance_initial_to_result = static_cast<double>(autoware::localization_util::norm(
      interpolation_result.interpolated_pose.pose.pose.position, result_pose_msg.position));
    report.add_key_value({"distance_initial_to_result", distance_initial_to_result});
    if (distance_initial_to_result > param_.validation.initial_to_result_distance_tolerance_m) {
      std::stringstream message;
      message << "distance_initial_to_result is too large (" << distance_initial_to_result
              << " [m]).";
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    }

    // check execution_time
    const auto exe_end_time = std::chrono::system_clock::now();
    const auto duration_micro_sec =
      std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
    const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
    report.add_key_value({"execution_time", exe_time});
    if (exe_time > param_.validation.critical_upper_bound_exe_time_ms) {
      std::stringstream message;
      message << "NDT exe time is too long (took " << exe_time << " [ms]).";
      report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    }

    // Assemble everything to publish while the lock is held; the caller publishes it after the
    // lock is released. Nothing below touches a publisher, and nothing above needs to.
    ScanMatchingOutput output;
    output.stamp = sensor_ros_time;
    output.multi_ndt_pose = std::move(covariance_debug.multi_ndt_pose);
    output.multi_initial_pose = std::move(covariance_debug.multi_initial_pose);
    output.initial_pose_with_covariance = interpolation_result.interpolated_pose;
    output.exe_time_ms = exe_time;
    output.transform_probability = ndt_result.transform_probability;
    output.nearest_voxel_transformation_likelihood =
      ndt_result.nearest_voxel_transformation_likelihood;
    output.iteration_num = ndt_result.iteration_num;

    output.tf = make_tf(sensor_ros_time, result_pose_msg);
    if (is_converged) {
      output.ndt_pose = make_pose(sensor_ros_time, result_pose_msg);
      output.ndt_pose_with_covariance =
        make_pose_with_covariance(sensor_ros_time, result_pose_msg, ndt_covariance);
    }

    output.ndt_marker = make_ndt_marker(
      sensor_ros_time, transformation_msg_array, ndt_ptr->getMaximumIterations() + 2);
    fill_initial_to_result(
      output, result_pose_msg, interpolation_result.interpolated_pose,
      interpolation_result.old_pose, interpolation_result.new_pose);

    pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
      new pcl::PointCloud<PointSource>);
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
    output.points_aligned =
      make_point_cloud(sensor_ros_time, param_.frame.map_frame, sensor_points_in_map_ptr);

    // check each of point score
    const float lower_nvs = 1.0f;
    const float upper_nvs = 3.5f;
    if (voxel_score_points_pub_->get_subscription_count() > 0) {
      output.voxel_score_points = make_point_cloud(
        sensor_ros_time, param_.frame.map_frame,
        visualize_point_score(sensor_points_in_map_ptr, lower_nvs, upper_nvs, *ndt_ptr));
    }

    // whether use no ground points to calculate score
    if (param_.score_estimation.no_ground_points.enable) {
      // remove ground
      pcl::shared_ptr<pcl::PointCloud<PointSource>> no_ground_points_in_map_ptr(
        new pcl::PointCloud<PointSource>);
      no_ground_points_in_map_ptr->points.reserve(sensor_points_in_map_ptr->size());
      // The aligned pose z is constant over the loop; the translation z of the 4x4 matrix equals
      // matrix4f_to_pose(ndt_result.pose).position.z. Hoist it to avoid rebuilding a full Pose
      // (including a quaternion extraction) for every point in the scan.
      const double result_pose_z = ndt_result.pose(2, 3);
      for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); i++) {
        const float point_z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
        if (
          point_z - result_pose_z >
          param_.score_estimation.no_ground_points.z_margin_for_ground_removal) {
          no_ground_points_in_map_ptr->points.push_back(sensor_points_in_map_ptr->points[i]);
        }
      }
      ScanMatchingOutput::NoGroundScore no_ground;
      // width/height are left at what toROSMsg derives from an unorganized cloud, as before.
      no_ground.points =
        make_point_cloud(sensor_ros_time, param_.frame.map_frame, no_ground_points_in_map_ptr);
      no_ground.transform_probability = static_cast<float>(
        ndt_ptr->calculateTransformationProbability(*no_ground_points_in_map_ptr));
      no_ground.nearest_voxel_transformation_likelihood = static_cast<float>(
        ndt_ptr->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map_ptr));
      output.no_ground = std::move(no_ground);
    }

    scan_matching_output = std::move(output);
    return is_converged;
  });

  if (scan_matching_output) {
    publish_scan_matching_output(*scan_matching_output);
  }
  return is_succeed;
}

void NdtScanMatcherNode::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr)
{
  if (source_frame == target_frame) {
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw;
  }

  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    autoware_utils_geometry::transform2pose(transform);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

void NdtScanMatcherNode::publish_scan_matching_output(const ScanMatchingOutput & output)
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

geometry_msgs::msg::TransformStamped NdtScanMatcherNode::make_tf(
  const builtin_interfaces::msg::Time & sensor_time,
  const geometry_msgs::msg::Pose & result_pose_msg) const
{
  return autoware_utils_geometry::pose2transform(
    make_pose(sensor_time, result_pose_msg), param_.frame.ndt_base_frame);
}

geometry_msgs::msg::PoseStamped NdtScanMatcherNode::make_pose(
  const builtin_interfaces::msg::Time & sensor_time,
  const geometry_msgs::msg::Pose & result_pose_msg) const
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;
  return result_pose_stamped_msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped NdtScanMatcherNode::make_pose_with_covariance(
  const builtin_interfaces::msg::Time & sensor_time,
  const geometry_msgs::msg::Pose & result_pose_msg,
  const std::array<double, 36> & ndt_covariance) const
{
  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_time;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = ndt_covariance;
  return result_pose_with_cov_msg;
}

visualization_msgs::msg::MarkerArray NdtScanMatcherNode::make_ndt_marker(
  const builtin_interfaces::msg::Time & sensor_time,
  const std::vector<geometry_msgs::msg::Pose> & pose_array, const int marker_slot_num) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_time;
  marker.header.frame_id = param_.frame.map_frame;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = autoware_utils_visualization::create_marker_scale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_msg : pose_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = exchange_color_crc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }

  // TODO(Tier IV): delete old marker
  for (; i < marker_slot_num;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

void NdtScanMatcherNode::fill_initial_to_result(
  ScanMatchingOutput & output, const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg) const
{
  output.initial_to_result_relative_pose.pose = autoware_utils_geometry::inverse_transform_pose(
    result_pose_msg, initial_pose_cov_msg.pose.pose);
  output.initial_to_result_relative_pose.header.stamp = output.stamp;
  output.initial_to_result_relative_pose.header.frame_id = param_.frame.map_frame;

  output.initial_to_result_distance = static_cast<float>(autoware::localization_util::norm(
    initial_pose_cov_msg.pose.pose.position, result_pose_msg.position));
  output.initial_to_result_distance_old = static_cast<float>(autoware::localization_util::norm(
    initial_pose_old_msg.pose.pose.position, result_pose_msg.position));
  output.initial_to_result_distance_new = static_cast<float>(autoware::localization_util::norm(
    initial_pose_new_msg.pose.pose.position, result_pose_msg.position));
}

int NdtScanMatcherNode::count_oscillation(
  const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array)
{
  return autoware::ndt_scan_matcher::count_oscillation(result_pose_msg_array);
}

NdtScanMatcherNode::CovarianceEstimate NdtScanMatcherNode::estimate_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const builtin_interfaces::msg::Time & sensor_time, NormalDistributionsTransform & ndt_ref,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_baselink_frame)
{
  geometry_msgs::msg::PoseArray multi_ndt_result_msg;
  geometry_msgs::msg::PoseArray multi_initial_pose_msg;
  multi_ndt_result_msg.header.stamp = sensor_time;
  multi_ndt_result_msg.header.frame_id = param_.frame.map_frame;
  multi_initial_pose_msg.header.stamp = sensor_time;
  multi_initial_pose_msg.header.frame_id = param_.frame.map_frame;
  multi_ndt_result_msg.poses.push_back(matrix4f_to_pose(ndt_result.pose));
  multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(initial_pose_matrix));

  // Each branch fills in only the arrays it used to publish, so that a caller which publishes
  // whatever is engaged reproduces exactly what was published before: both for MULTI_NDT, only
  // the initial poses for MULTI_NDT_SCORE, neither otherwise.
  CovarianceEstimate estimate;

  if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::LAPLACE_APPROXIMATION) {
    estimate.covariance =
      pclomp::estimate_xy_covariance_by_laplace_approximation(ndt_result.hessian);
  } else if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param_.covariance.covariance_estimation.initial_pose_offset_model_x,
      param_.covariance.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation result_of_multi_ndt_covariance_estimation =
      estimate_xy_covariance_by_multi_ndt(
        ndt_result, ndt_ref, poses_to_search, sensor_points_in_baselink_frame);
    for (size_t i = 0; i < result_of_multi_ndt_covariance_estimation.ndt_initial_poses.size();
         i++) {
      multi_ndt_result_msg.poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_results[i].pose));
      multi_initial_pose_msg.poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_initial_poses[i]));
    }
    estimate.multi_ndt_pose = std::move(multi_ndt_result_msg);
    estimate.multi_initial_pose = std::move(multi_initial_pose_msg);
    estimate.covariance = result_of_multi_ndt_covariance_estimation.covariance;
  } else if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT_SCORE) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param_.covariance.covariance_estimation.initial_pose_offset_model_x,
      param_.covariance.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation
      result_of_multi_ndt_score_covariance_estimation = estimate_xy_covariance_by_multi_ndt_score(
        ndt_result, ndt_ref, poses_to_search, sensor_points_in_baselink_frame,
        param_.covariance.covariance_estimation.temperature);
    for (const auto & sub_initial_pose_matrix : poses_to_search) {
      multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(sub_initial_pose_matrix));
    }
    estimate.multi_initial_pose = std::move(multi_initial_pose_msg);
    estimate.covariance = result_of_multi_ndt_score_covariance_estimation.covariance;
  } else {
    estimate.covariance =
      Eigen::Matrix2d::Identity() * param_.covariance.output_pose_covariance[0 + 6 * 0];
  }

  return estimate;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr NdtScanMatcherNode::visualize_point_score(
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr,
  const float & lower_nvs, const float & upper_nvs, NormalDistributionsTransform & ndt_ref)
{
  pcl::PointCloud<pcl::PointXYZI> nvs_points_in_map_ptr_i;
  nvs_points_in_map_ptr_i = ndt_ref.calculateNearestVoxelScoreEachPoint(*sensor_points_in_map_ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
    new pcl::PointCloud<pcl::PointXYZRGB>};

  const float range = upper_nvs - lower_nvs;
  for (std::size_t i = 0; i < nvs_points_in_map_ptr_i.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = nvs_points_in_map_ptr_i.points[i].x;
    point.y = nvs_points_in_map_ptr_i.points[i].y;
    point.z = nvs_points_in_map_ptr_i.points[i].z;
    std_msgs::msg::ColorRGBA color =
      exchange_color_crc((nvs_points_in_map_ptr_i.points[i].intensity - lower_nvs) / range);
    point.r = static_cast<std::uint8_t>(color.r * 255);
    point.g = static_cast<std::uint8_t>(color.g * 255);
    point.b = static_cast<std::uint8_t>(color.b * 255);
    nvs_points_in_map_ptr_rgb->points.push_back(point);
  }
  return nvs_points_in_map_ptr_rgb;
}

void NdtScanMatcherNode::add_regularization_pose(
  const rclcpp::Time & sensor_ros_time, NormalDistributionsTransform & ndt_ref)
{
  ndt_ref.unsetRegularizationPose();
  std::vector<LogRequest> interpolation_logs;
  std::optional<PoseInterpolationBuffer::InterpolateResult> interpolation_result_opt =
    regularization_pose_buffer_->interpolate(sensor_ros_time, interpolation_logs);
  replay_logs(interpolation_logs);
  if (!interpolation_result_opt) {
    return;
  }
  regularization_pose_buffer_->pop_old(sensor_ros_time);
  const PoseInterpolationBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();
  const Eigen::Matrix4f pose = pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  ndt_ref.setRegularizationPose(pose);
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
    initial_pose_buffer_->clear();
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
  auto result = map_update_module_->update_map(initial_pose_msg_in_map_frame.pose.pose.position);
  apply_diagnostics_update(*diagnostics_ndt_align_, result.diagnostics);

  publish_loaded_map_if_present(result);

  PoseInitializationSearch::Result align_result;
  // The lock is held across the search, as before: it aligns against the installed NDT. Stepping
  // it here rather than handing the module a callback is what keeps the publishing below in sight
  // of the loop it belongs to.
  ndt_ptr_.with([&](auto & ndt_ptr) {
    PoseInitializationSearch search{
      pose_initialization_params_, *ndt_ptr, sensor_points_in_baselink_frame_,
      initial_pose_msg_in_map_frame};
    while (const auto progress = search.next()) {
      publish_pose_initialization_progress(*progress);
    }
    align_result = search.finish();
  });
  replay_logs(align_result.diagnostics.logs);
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
