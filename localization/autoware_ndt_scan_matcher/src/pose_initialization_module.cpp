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

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/pose_initialization_module.hpp>
#include <autoware_utils_pcl/transforms.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
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
namespace
{

using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;
using autoware::localization_util::TreeStructuredParzenEstimator;

// The text `autoware::localization_util::output_pose_with_cov_to_log` builds, without its
// RCLCPP_DEBUG_STREAM. Reproduced rather than called so that the module can record the line for
// the node to emit, instead of holding a logger.
std::string pose_with_cov_log_line(
  const std::string & prefix, const geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov)
{
  // `make_eigen_covariance` is file-local to util_func.cpp; this is the same construction.
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance{
    pose_with_cov.pose.covariance.data(), 6, 6};
  const geometry_msgs::msg::Pose pose = pose_with_cov.pose.pose;
  geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);
  rpy.x = rpy.x * 180.0 / M_PI;
  rpy.y = rpy.y * 180.0 / M_PI;
  rpy.z = rpy.z * 180.0 / M_PI;

  std::stringstream ss;
  ss << std::fixed << prefix << "," << pose.position.x << "," << pose.position.y << ","
     << pose.position.z << "," << pose.orientation.x << "," << pose.orientation.y << ","
     << pose.orientation.z << "," << pose.orientation.w << "," << rpy.x << "," << rpy.y << ","
     << rpy.z << "," << covariance(0, 0) << "," << covariance(1, 1) << "," << covariance(2, 2)
     << "," << covariance(3, 3) << "," << covariance(4, 4) << "," << covariance(5, 5);
  return ss.str();
}

}  // namespace

PoseInitializationModule::PoseInitializationModule(Params params, ProgressCallback on_progress)
: param_(std::move(params)), on_progress_(std::move(on_progress))
{
}

PoseInitializationModule::Result PoseInitializationModule::estimate(
  NdtType & ndt, const CloudPtr & sensor_points_in_baselink_frame,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_in_map_frame)
{
  Result result;
  DiagnosticsReport & diagnostics = result.diagnostics;

  if (!diagnostics.check(
        "is_set_map_points", ndt.hasTarget(), DiagnosticLevel::WARN,
        "No InputTarget. Please check the map file and the map_loader service",
        LogSite::AlignNoInputTarget)) {
    return result;
  }

  if (!diagnostics.check(
        "is_set_sensor_points", sensor_points_in_baselink_frame != nullptr, DiagnosticLevel::WARN,
        "No InputSource. Please check the input lidar topic", LogSite::AlignNoInputSource)) {
    return result;
  }

  diagnostics.logs.push_back(
    {LogSite::AlignPoseInput,
     pose_with_cov_log_line("align_pose_input", initial_pose_in_map_frame)});

  const auto base_rpy = autoware::localization_util::get_rpy(initial_pose_in_map_frame);
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_in_map_frame.pose.covariance.data(), 6, 6};
  const double stddev_x = std::sqrt(covariance(0, 0));
  const double stddev_y = std::sqrt(covariance(1, 1));
  const double stddev_z = std::sqrt(covariance(2, 2));
  const double stddev_roll = std::sqrt(covariance(3, 3));
  const double stddev_pitch = std::sqrt(covariance(4, 4));

  // Since only yaw is uniformly sampled, we define the mean and standard deviation for the others.
  const std::vector<double> sample_mean{
    initial_pose_in_map_frame.pose.pose.position.x,  // trans_x
    initial_pose_in_map_frame.pose.pose.position.y,  // trans_y
    initial_pose_in_map_frame.pose.pose.position.z,  // trans_z
    base_rpy.x,                                      // angle_x
    base_rpy.y                                       // angle_y
  };
  const std::vector<double> sample_stddev{stddev_x, stddev_y, stddev_z, stddev_roll, stddev_pitch};

  // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
  TreeStructuredParzenEstimator tpe(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, param_.n_startup_trials, sample_mean,
    sample_stddev);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  for (int64_t i = 0; i < param_.particles_num; i++) {
    const TreeStructuredParzenEstimator::Input input = tpe.get_next_input();

    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = input[0];
    initial_pose.position.y = input[1];
    initial_pose.position.z = input[2];
    geometry_msgs::msg::Vector3 init_rpy;
    init_rpy.x = input[3];
    init_rpy.y = input[4];
    init_rpy.z = input[5];
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(init_rpy.x, init_rpy.y, init_rpy.z);
    initial_pose.orientation = tf2::toMsg(tf_quaternion);

    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
    ndt.align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame);
    const pclomp::NdtResult ndt_result = ndt.getResult();

    Particle particle(
      initial_pose, matrix4f_to_pose(ndt_result.pose),
      ndt_result.nearest_voxel_transformation_likelihood, ndt_result.iteration_num);
    particle_array.push_back(particle);

    auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
    Progress progress{i, particle, sensor_msgs::msg::PointCloud2()};
    pcl::toROSMsg(*sensor_points_in_map_ptr, progress.sensor_points_in_map);
    progress.sensor_points_in_map.header.stamp = initial_pose_in_map_frame.header.stamp;
    progress.sensor_points_in_map.header.frame_id = param_.map_frame;
    on_progress_(progress);

    const geometry_msgs::msg::Pose pose = matrix4f_to_pose(ndt_result.pose);
    const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);

    TreeStructuredParzenEstimator::Input result(6);
    result[0] = pose.position.x;
    result[1] = pose.position.y;
    result[2] = pose.position.z;
    result[3] = rpy.x;
    result[4] = rpy.y;
    result[5] = rpy.z;
    tpe.add_trial(TreeStructuredParzenEstimator::Trial{result, ndt_result.transform_probability});
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  Estimate estimate;
  estimate.pose_with_covariance.header.stamp = initial_pose_in_map_frame.header.stamp;
  estimate.pose_with_covariance.header.frame_id = param_.map_frame;
  estimate.pose_with_covariance.pose.pose = best_particle_ptr->result_pose;
  estimate.score = best_particle_ptr->score;
  estimate.reliable =
    (param_.converged_param_nearest_voxel_transformation_likelihood < estimate.score);

  diagnostics.logs.push_back(
    {LogSite::AlignPoseOutput,
     pose_with_cov_log_line("align_pose_output", estimate.pose_with_covariance)});
  diagnostics.add_key_value({"best_particle_score", estimate.score});

  if (!estimate.reliable) {
    std::stringstream message;
    message << "Initial Pose Estimation is Unstable. Score is " << estimate.score;
    diagnostics.logs.push_back({LogSite::AlignUnstableScore, message.str()});
  }

  result.estimate = std::move(estimate);
  return result;
}

}  // namespace autoware::ndt_scan_matcher
