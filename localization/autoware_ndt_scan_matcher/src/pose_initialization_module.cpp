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

PoseInitializationModule::PoseInitializationModule(Params params) : param_(std::move(params))
{
}

PoseInitializationModule::Search PoseInitializationModule::begin(
  NdtType & ndt, const CloudPtr & sensor_points_in_baselink_frame,
  const PoseWithCovarianceStamped & initial_pose_in_map_frame) const
{
  return Search{param_, ndt, sensor_points_in_baselink_frame, initial_pose_in_map_frame};
}

PoseInitializationModule::Search::Search(
  Params param, NdtType & ndt, CloudPtr sensor_points_in_baselink_frame,
  PoseWithCovarianceStamped initial_pose_in_map_frame)
: param_(std::move(param)),
  ndt_(ndt),
  sensor_points_in_baselink_frame_(std::move(sensor_points_in_baselink_frame)),
  initial_pose_in_map_frame_(std::move(initial_pose_in_map_frame)),
  output_cloud_(pcl::make_shared<pcl::PointCloud<PointSource>>())
{
  if (!diagnostics_.check(
        "is_set_map_points", ndt_.hasTarget(), DiagnosticLevel::WARN,
        "No InputTarget. Please check the map file and the map_loader service",
        LogSite::AlignNoInputTarget)) {
    return;
  }

  if (!diagnostics_.check(
        "is_set_sensor_points", sensor_points_in_baselink_frame_ != nullptr, DiagnosticLevel::WARN,
        "No InputSource. Please check the input lidar topic", LogSite::AlignNoInputSource)) {
    return;
  }

  diagnostics_.logs.push_back(
    {LogSite::AlignPoseInput,
     pose_with_cov_log_line("align_pose_input", initial_pose_in_map_frame_)});

  const auto base_rpy = autoware::localization_util::get_rpy(initial_pose_in_map_frame_);
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_in_map_frame_.pose.covariance.data(), 6, 6};

  // Since only yaw is uniformly sampled, we define the mean and standard deviation for the others.
  const std::vector<double> sample_mean{
    initial_pose_in_map_frame_.pose.pose.position.x,  // trans_x
    initial_pose_in_map_frame_.pose.pose.position.y,  // trans_y
    initial_pose_in_map_frame_.pose.pose.position.z,  // trans_z
    base_rpy.x,                                       // angle_x
    base_rpy.y                                        // angle_y
  };
  const std::vector<double> sample_stddev{
    std::sqrt(covariance(0, 0)), std::sqrt(covariance(1, 1)), std::sqrt(covariance(2, 2)),
    std::sqrt(covariance(3, 3)), std::sqrt(covariance(4, 4))};

  // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
  tpe_ = std::make_unique<TreeStructuredParzenEstimator>(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, param_.n_startup_trials, sample_mean,
    sample_stddev);
}

PoseInitializationModule::Search::~Search() = default;

std::optional<PoseInitializationModule::Progress> PoseInitializationModule::Search::next()
{
  if (!tpe_ || index_ >= param_.particles_num) {
    return std::nullopt;
  }

  const TreeStructuredParzenEstimator::Input input = tpe_->get_next_input();

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
  ndt_.align(*output_cloud_, initial_pose_matrix, sensor_points_in_baselink_frame_);
  const pclomp::NdtResult ndt_result = ndt_.getResult();

  const Particle particle(
    initial_pose, matrix4f_to_pose(ndt_result.pose),
    ndt_result.nearest_voxel_transformation_likelihood, ndt_result.iteration_num);
  particles_.push_back(particle);

  const geometry_msgs::msg::Pose pose = matrix4f_to_pose(ndt_result.pose);
  const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);

  TreeStructuredParzenEstimator::Input trial(6);
  trial[0] = pose.position.x;
  trial[1] = pose.position.y;
  trial[2] = pose.position.z;
  trial[3] = rpy.x;
  trial[4] = rpy.y;
  trial[5] = rpy.z;
  tpe_->add_trial(TreeStructuredParzenEstimator::Trial{trial, ndt_result.transform_probability});

  auto sensor_points_in_map_ptr = pcl::make_shared<pcl::PointCloud<PointSource>>();
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_in_baselink_frame_, *sensor_points_in_map_ptr, ndt_result.pose);

  Progress progress{index_, particle, sensor_msgs::msg::PointCloud2()};
  pcl::toROSMsg(*sensor_points_in_map_ptr, progress.sensor_points_in_map);
  progress.sensor_points_in_map.header.stamp = initial_pose_in_map_frame_.header.stamp;
  progress.sensor_points_in_map.header.frame_id = param_.map_frame;

  index_++;
  return progress;
}

PoseInitializationModule::Result PoseInitializationModule::Search::finish()
{
  Result result;
  result.diagnostics = std::move(diagnostics_);
  if (particles_.empty()) {
    return result;
  }

  const auto best_particle_ptr = std::max_element(
    std::begin(particles_), std::end(particles_),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  Estimate estimate;
  estimate.pose_with_covariance.header.stamp = initial_pose_in_map_frame_.header.stamp;
  estimate.pose_with_covariance.header.frame_id = param_.map_frame;
  estimate.pose_with_covariance.pose.pose = best_particle_ptr->result_pose;
  estimate.score = best_particle_ptr->score;
  estimate.reliable =
    (param_.converged_param_nearest_voxel_transformation_likelihood < estimate.score);

  result.diagnostics.logs.push_back(
    {LogSite::AlignPoseOutput,
     pose_with_cov_log_line("align_pose_output", estimate.pose_with_covariance)});
  result.diagnostics.add_key_value({"best_particle_score", estimate.score});

  if (!estimate.reliable) {
    std::stringstream message;
    message << "Initial Pose Estimation is Unstable. Score is " << estimate.score;
    result.diagnostics.logs.push_back({LogSite::AlignUnstableScore, message.str()});
  }

  result.estimate = std::move(estimate);
  return result;
}

}  // namespace autoware::ndt_scan_matcher
