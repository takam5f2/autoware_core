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
#include <autoware/ndt_scan_matcher/pose_initialization_search.hpp>
#include <autoware/ndt_scan_matcher/pose_interpolation_buffer.hpp>  // to_nanoseconds
#include <autoware_utils_pcl/transforms.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <memory>
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

using PointSource = pcl::PointXYZ;
using PointTarget = pcl::PointXYZ;
using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointSource>>;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

// The text `autoware::localization_util::output_pose_with_cov_to_log` builds, without its
// RCLCPP_DEBUG_STREAM. Reproduced rather than called so that the search can record the line for
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

TreeStructuredParzenEstimator make_pose_sampler(
  const PoseInitializationParams & param,
  const PoseWithCovarianceStamped & initial_pose_in_map_frame)
{
  const auto base_rpy = autoware::localization_util::get_rpy(initial_pose_in_map_frame);
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_in_map_frame.pose.covariance.data(), 6, 6};

  // Since only yaw is uniformly sampled, we define the mean and standard deviation for the others.
  const std::vector<double> sample_mean{
    initial_pose_in_map_frame.pose.pose.position.x,  // trans_x
    initial_pose_in_map_frame.pose.pose.position.y,  // trans_y
    initial_pose_in_map_frame.pose.pose.position.z,  // trans_z
    base_rpy.x,                                      // angle_x
    base_rpy.y                                       // angle_y
  };
  const std::vector<double> sample_stddev{
    std::sqrt(covariance(0, 0)), std::sqrt(covariance(1, 1)), std::sqrt(covariance(2, 2)),
    std::sqrt(covariance(3, 3)), std::sqrt(covariance(4, 4))};

  // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
  //
  // Seeded from the request's stamp, so that two requests explore differently -- a retry of a
  // failed initialization is worth making -- while replaying one request reproduces its search
  // exactly. A per-node counter would give the first property and not the second, and the shared
  // `static` this replaced gave neither reliably: which inputs a search proposed depended on how
  // many searches had run anywhere in the process before it.
  return TreeStructuredParzenEstimator{
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, param.n_startup_trials, sample_mean,
    sample_stddev,
    static_cast<std::uint64_t>(to_nanoseconds(initial_pose_in_map_frame.header.stamp))};
}

geometry_msgs::msg::Pose to_pose(const TreeStructuredParzenEstimator::Input & input)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = input[0];
  pose.position.y = input[1];
  pose.position.z = input[2];
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(input[3], input[4], input[5]);
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

}  // namespace

PoseInitializationResult search_initial_pose(
  const PoseInitializationParams & param, NdtType & ndt, const CloudPtr & scan_in_baselink_frame,
  const PoseWithCovarianceStamped & initial_pose_in_map_frame,
  const builtin_interfaces::msg::Time & now)
{
  PoseInitializationResult result;
  DiagnosticsReport & diagnostics = result.diagnostics;

  if (!diagnostics.check(
        "is_set_map_points", ndt.hasTarget(), DiagnosticLevel::WARN,
        "No InputTarget. Please check the map file and the map_loader service",
        LogSite::AlignNoInputTarget)) {
    return result;
  }

  if (!diagnostics.check(
        "is_set_sensor_points", scan_in_baselink_frame != nullptr, DiagnosticLevel::WARN,
        "No InputSource. Please check the input lidar topic", LogSite::AlignNoInputSource)) {
    return result;
  }

  diagnostics.logs.push_back(
    {LogSite::AlignPoseInput,
     pose_with_cov_log_line("align_pose_input", initial_pose_in_map_frame)});

  TreeStructuredParzenEstimator pose_sampler = make_pose_sampler(param, initial_pose_in_map_frame);

  std::vector<Particle> particles;
  // The matrix `align` produced for each particle, in the same order. Kept so that the winner's
  // cloud is transformed by that matrix rather than by a pose round-trip through
  // matrix4f_to_pose, which would round it through float.
  std::vector<Eigen::Matrix4f> result_matrices;
  // Reused across particles, as it was when this was one loop in the node.
  const auto output_cloud = pcl::make_shared<pcl::PointCloud<PointSource>>();

  for (int64_t i = 0; i < param.particles_num; i++) {
    const geometry_msgs::msg::Pose initial_pose = to_pose(pose_sampler.get_next_input());

    ndt.align(*output_cloud, pose_to_matrix4f(initial_pose), scan_in_baselink_frame);
    const pclomp::NdtResult ndt_result = ndt.getResult();

    const geometry_msgs::msg::Pose result_pose = matrix4f_to_pose(ndt_result.pose);
    particles.emplace_back(
      initial_pose, result_pose, ndt_result.nearest_voxel_transformation_likelihood,
      ndt_result.iteration_num);
    result_matrices.push_back(ndt_result.pose);

    const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(result_pose);
    TreeStructuredParzenEstimator::Input trial(6);
    trial[0] = result_pose.position.x;
    trial[1] = result_pose.position.y;
    trial[2] = result_pose.position.z;
    trial[3] = rpy.x;
    trial[4] = rpy.y;
    trial[5] = rpy.z;
    pose_sampler.add_trial(
      TreeStructuredParzenEstimator::Trial{trial, ndt_result.transform_probability});
  }

  if (particles.empty()) {
    return result;
  }

  visualization_msgs::msg::MarkerArray markers;
  for (size_t i = 0; i < particles.size(); i++) {
    push_debug_markers(markers, now, param.map_frame, particles[i], i);
  }
  result.search_markers = std::move(markers);

  const auto best_particle_ptr = std::max_element(
    std::begin(particles), std::end(particles),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });
  const auto best_index =
    static_cast<size_t>(std::distance(std::begin(particles), best_particle_ptr));

  const auto scan_in_map = pcl::make_shared<pcl::PointCloud<PointSource>>();
  autoware_utils_pcl::transform_pointcloud(
    *scan_in_baselink_frame, *scan_in_map, result_matrices[best_index]);
  sensor_msgs::msg::PointCloud2 best_points_aligned;
  pcl::toROSMsg(*scan_in_map, best_points_aligned);
  best_points_aligned.header.stamp = initial_pose_in_map_frame.header.stamp;
  best_points_aligned.header.frame_id = param.map_frame;
  result.best_points_aligned = std::move(best_points_aligned);

  PoseInitializationEstimate estimate;
  estimate.pose_with_covariance.header.stamp = initial_pose_in_map_frame.header.stamp;
  estimate.pose_with_covariance.header.frame_id = param.map_frame;
  estimate.pose_with_covariance.pose.pose = best_particle_ptr->result_pose;
  estimate.score = best_particle_ptr->score;
  estimate.reliable =
    (param.converged_param_nearest_voxel_transformation_likelihood < estimate.score);

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
