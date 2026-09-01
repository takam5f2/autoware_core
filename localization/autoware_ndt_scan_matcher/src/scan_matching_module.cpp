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
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <autoware/ndt_scan_matcher/scan_matching_module.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_pcl/transforms.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;

ScanMatchingModule::ScanMatchingModule(Params param, const std::atomic<bool> & activated)
: param_(std::move(param)),
  activated_(activated),
  initial_pose_buffer_(
    param_.validation.initial_pose_timeout_sec, param_.validation.initial_pose_distance_tolerance_m)
{
  if (param_.ndt_regularization_enable) {
    // The regularization buffer accepts anything: the pose it interpolates is an input to the
    // solver, not a measurement to validate.
    constexpr double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<PoseInterpolationBuffer>(value_as_unlimited, value_as_unlimited);
  }
}

DiagnosticsReport ScanMatchingModule::push_initial_pose(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose)
{
  DiagnosticsReport report;
  report.add_key_value({"topic_time_stamp", to_nanoseconds(pose->header.stamp)});

  if (!report.check("is_activated", activated_, DiagnosticLevel::WARN, "Node is not activated.")) {
    return report;
  }

  std::stringstream message;
  message << "Received initial pose message with frame_id " << pose->header.frame_id
          << ", but expected " << param_.frame.map_frame
          << ". Please check the frame_id in the input topic and ensure it is correct.";
  if (!report.check(
        "is_expected_frame_id", pose->header.frame_id == param_.frame.map_frame,
        DiagnosticLevel::ERROR, message.str())) {
    return report;
  }

  initial_pose_buffer_.push_back(pose);
  latest_ekf_position_.with([&](auto & position) { position = pose->pose.pose.position; });
  return report;
}

DiagnosticsReport ScanMatchingModule::push_regularization_pose(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose)
{
  DiagnosticsReport report;
  report.add_key_value({"topic_time_stamp", to_nanoseconds(pose->header.stamp)});
  regularization_pose_buffer_->push_back(pose);
  return report;
}

void ScanMatchingModule::clear_initial_pose_buffer()
{
  initial_pose_buffer_.clear();
}

std::optional<geometry_msgs::msg::Point> ScanMatchingModule::latest_ekf_position()
{
  return latest_ekf_position_.with([](const auto & position) { return position; });
}

ScanMatchingModule::Result ScanMatchingModule::scan_match(
  const ScanInput & input, NdtType & ndt, MapUpdateModule & map_update)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  Result result;
  DiagnosticsReport & report = result.diagnostics;

  const std::optional<CloudPtr> scan_in_baselink_frame = prepare_scan(input, report);
  if (!scan_in_baselink_frame) {
    return result;
  }
  // Handed back even when the match goes no further: the align service reads it, and it is kept
  // from here on -- after the distance gate, before the activation gate -- as it always was.
  result.scan_in_baselink_frame = *scan_in_baselink_frame;

  // The caller holds the NDT's lock around this whole call, so what follows runs as one critical
  // section without taking one of its own.
  const std::optional<Alignment> alignment =
    align_and_judge(input, ndt, map_update, *scan_in_baselink_frame, report);
  if (!alignment) {
    return result;
  }

  result.output =
    build_output(input, ndt, *alignment, *scan_in_baselink_frame, exe_start_time, report);
  result.converged = alignment->is_converged;
  return result;
}

std::optional<ScanMatchingModule::CloudPtr> ScanMatchingModule::prepare_scan(
  const ScanInput & input, DiagnosticsReport & report) const
{
  const auto & sensor_points_msg_in_sensor_frame = input.scan;

  // check topic_time_stamp
  const builtin_interfaces::msg::Time sensor_ros_time =
    sensor_points_msg_in_sensor_frame->header.stamp;
  report.add_key_value({"topic_time_stamp", to_nanoseconds(sensor_ros_time)});

  // check sensor_points_size
  const size_t sensor_points_size = sensor_points_msg_in_sensor_frame->width;
  report.add_key_value({"sensor_points_size", static_cast<int64_t>(sensor_points_size)});
  if (sensor_points_size == 0) {
    std::stringstream message;
    message << "Sensor points is empty.";
    report.update_level_and_message(DiagnosticLevel::WARN, message.str());
    return std::nullopt;
  }

  // check sensor_points_delay_time_sec
  const double sensor_points_delay_time_sec =
    to_seconds(to_nanoseconds(input.now) - to_nanoseconds(sensor_ros_time));
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
  if (!input.base_from_sensor.transform) {
    std::stringstream message;
    message << input.base_from_sensor.error << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    report.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    report.logs.push_back({LogSite::ScanTransformFailed, message.str()});
    report.add_key_value({"is_succeed_transform_sensor_points", false});
    return std::nullopt;
  }
  if (sensor_frame == param_.frame.base_frame) {
    // The transform is the identity and the clouds would be equal; aliasing avoids a copy of the
    // whole scan, as the node's own transform did.
    sensor_points_in_baselink_frame = sensor_points_in_sensor_frame;
  } else {
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_sensor_frame, *sensor_points_in_baselink_frame,
      pose_to_matrix4f(
        autoware_utils_geometry::transform2pose(*input.base_from_sensor.transform).pose));
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
    return std::nullopt;
  }

  // The caller holds the NDT's lock around this whole call, so the body below runs as one
  return sensor_points_in_baselink_frame;
}

std::optional<ScanMatchingModule::Alignment> ScanMatchingModule::align_and_judge(
  const ScanInput & input, NdtType & ndt, MapUpdateModule & map_update,
  const CloudPtr & sensor_points_in_baselink_frame, DiagnosticsReport & report)
{
  auto * const ndt_ptr = &ndt;
  const builtin_interfaces::msg::Time sensor_ros_time = input.scan->header.stamp;

  if (!report.check("is_activated", activated_, DiagnosticLevel::WARN, "Node is not activated.")) {
    return std::nullopt;
  }

  // calculate initial pose
  const std::optional<PoseInterpolationBuffer::InterpolateResult> interpolation_result_opt =
    initial_pose_buffer_.interpolate(sensor_ros_time, report.logs);

  if (!report.check(
        "is_succeed_interpolate_initial_pose", interpolation_result_opt != std::nullopt,
        DiagnosticLevel::WARN,
        "Couldn't interpolate pose. Please verify that "
        "(1) the initial pose topic (primarily come from the EKF) is being published, and "
        "(2) the timestamps of the sensor PCD messages and pose messages are synchronized "
        "correctly.")) {
    return std::nullopt;
  }

  initial_pose_buffer_.pop_old(sensor_ros_time);
  const PoseInterpolationBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();

  // if regularization is enabled and available, set pose to NDT for regularization
  if (param_.ndt_regularization_enable) {
    add_regularization_pose(sensor_ros_time, *ndt_ptr);
  }

  // Warn if the lidar has gone out of the map range
  if (map_update.out_of_map_range(interpolation_result.interpolated_pose.pose.pose.position)) {
    std::stringstream msg;

    msg << "Lidar has gone out of the map range";
    report.update_level_and_message(DiagnosticLevel::WARN, msg.str());
    report.logs.push_back({LogSite::ScanOutOfMapRange, msg.str()});
  }

  if (!report.check(
        "is_set_map_points", ndt_ptr->hasTarget(), DiagnosticLevel::WARN,
        "Map points is not set.")) {
    return std::nullopt;
  }

  // perform ndt scan matching
  const Eigen::Matrix4f initial_pose_matrix =
    pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  ndt_ptr->align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame);
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
  const bool is_local_optimal_solution_oscillation = (oscillation_num > oscillation_num_threshold);
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
    message << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
    report.update_level_and_message(DiagnosticLevel::ERROR, message.str());
    return std::nullopt;
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
  const std::vector<float> & nvtl_array = ndt_result.nearest_voxel_transformation_likelihood_array;
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
  bool is_converged = (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;

  Alignment alignment;
  alignment.ndt_result = ndt_result;
  alignment.result_pose = result_pose_msg;
  alignment.transformation_array = transformation_msg_array;
  alignment.interpolation = interpolation_result;
  alignment.initial_pose_matrix = initial_pose_matrix;
  alignment.is_converged = is_converged;
  return alignment;
}

ScanMatchingModule::ScanMatchingOutput ScanMatchingModule::build_output(
  const ScanInput & input, NdtType & ndt, const Alignment & alignment,
  const CloudPtr & sensor_points_in_baselink_frame,
  const std::chrono::system_clock::time_point & exe_start_time, DiagnosticsReport & report)
{
  auto * const ndt_ptr = &ndt;
  const auto & ndt_result = alignment.ndt_result;
  const auto & result_pose_msg = alignment.result_pose;
  const auto & transformation_msg_array = alignment.transformation_array;
  const auto & interpolation_result = alignment.interpolation;
  const auto & initial_pose_matrix = alignment.initial_pose_matrix;
  const bool is_converged = alignment.is_converged;
  const builtin_interfaces::msg::Time sensor_ros_time = input.scan->header.stamp;
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
      ndt_result, initial_pose_matrix, sensor_ros_time, *ndt_ptr, sensor_points_in_baselink_frame);
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

  output.ndt_marker =
    make_ndt_marker(sensor_ros_time, transformation_msg_array, ndt_ptr->getMaximumIterations() + 2);
  fill_initial_to_result(
    output, result_pose_msg, interpolation_result.interpolated_pose, interpolation_result.old_pose,
    interpolation_result.new_pose);

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
    new pcl::PointCloud<PointSource>);
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
  output.points_aligned =
    make_point_cloud(sensor_ros_time, param_.frame.map_frame, sensor_points_in_map_ptr);

  // check each of point score
  const float lower_nvs = 1.0f;
  const float upper_nvs = 3.5f;
  if (param_.score_estimation.publish_voxel_score_points) {
    output.voxel_score_points = make_point_cloud(
      sensor_ros_time, param_.frame.map_frame,
      colorize_point_score(sensor_points_in_map_ptr, lower_nvs, upper_nvs, *ndt_ptr));
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
    no_ground.transform_probability =
      static_cast<float>(ndt_ptr->calculateTransformationProbability(*no_ground_points_in_map_ptr));
    no_ground.nearest_voxel_transformation_likelihood = static_cast<float>(
      ndt_ptr->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map_ptr));
    output.no_ground = std::move(no_ground);
  }

  return output;
}

sensor_msgs::msg::PointCloud2 ScanMatchingModule::make_point_cloud(
  const builtin_interfaces::msg::Time & sensor_time, const std::string & frame_id,
  const CloudPtr & points_in_map) const
{
  sensor_msgs::msg::PointCloud2 points_msg_in_map;
  pcl::toROSMsg(*points_in_map, points_msg_in_map);
  points_msg_in_map.header.stamp = sensor_time;
  points_msg_in_map.header.frame_id = frame_id;
  return points_msg_in_map;
}

sensor_msgs::msg::PointCloud2 ScanMatchingModule::make_point_cloud(
  const builtin_interfaces::msg::Time & sensor_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & points_in_map) const
{
  sensor_msgs::msg::PointCloud2 points_msg_in_map;
  pcl::toROSMsg(*points_in_map, points_msg_in_map);
  points_msg_in_map.header.stamp = sensor_time;
  points_msg_in_map.header.frame_id = frame_id;
  return points_msg_in_map;
}

geometry_msgs::msg::TransformStamped ScanMatchingModule::make_tf(
  const builtin_interfaces::msg::Time & sensor_time,
  const geometry_msgs::msg::Pose & result_pose_msg) const
{
  return autoware_utils_geometry::pose2transform(
    make_pose(sensor_time, result_pose_msg), param_.frame.ndt_base_frame);
}

geometry_msgs::msg::PoseStamped ScanMatchingModule::make_pose(
  const builtin_interfaces::msg::Time & sensor_time,
  const geometry_msgs::msg::Pose & result_pose_msg) const
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;
  return result_pose_stamped_msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped ScanMatchingModule::make_pose_with_covariance(
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

visualization_msgs::msg::MarkerArray ScanMatchingModule::make_ndt_marker(
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

void ScanMatchingModule::fill_initial_to_result(
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

ScanMatchingModule::CovarianceEstimate ScanMatchingModule::estimate_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const builtin_interfaces::msg::Time & sensor_time, NdtType & ndt_ref,
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ScanMatchingModule::colorize_point_score(
  const CloudPtr & sensor_points_in_map_ptr, const float & lower_nvs, const float & upper_nvs,
  NdtType & ndt_ref) const
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

void ScanMatchingModule::add_regularization_pose(
  const builtin_interfaces::msg::Time & sensor_time, NdtType & ndt_ref)
{
  ndt_ref.unsetRegularizationPose();
  std::vector<LogRequest> discarded_logs;
  const std::optional<PoseInterpolationBuffer::InterpolateResult> interpolation_result_opt =
    regularization_pose_buffer_->interpolate(sensor_time, discarded_logs);
  if (!interpolation_result_opt) {
    return;
  }
  regularization_pose_buffer_->pop_old(sensor_time);
  const PoseInterpolationBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();
  const Eigen::Matrix4f pose = pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  ndt_ref.setRegularizationPose(pose);
}

}  // namespace autoware::ndt_scan_matcher
