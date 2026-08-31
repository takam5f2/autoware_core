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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCHING_MODULE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCHING_MODULE_HPP_

#include "diagnostics_report.hpp"
#include "guarded.hpp"
#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "pose_interpolation_buffer.hpp"

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

// Which of the two NDT scores decides convergence. Declared here rather than in
// hyper_parameters.hpp so that naming it costs no dependency on rclcpp.
enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

enum class CovarianceEstimationType {
  FIXED_VALUE = 0,
  LAPLACE_APPROXIMATION = 1,
  MULTI_NDT = 2,
  MULTI_NDT_SCORE = 3,
};

// One scan, matched against the loaded map: validate it, transform it, interpolate where the
// vehicle was when it was taken, align, judge the result, and assemble what to publish.
//
// No ROS beyond message types. The four things that needed the node -- the clock, the TF lookup,
// whether anyone subscribes to the score cloud, and what the loaded map covers -- arrive as inputs
// or arguments.
class ScanMatchingModule
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointSource>>;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  // Mirrors the shape of the corresponding HyperParameters fields, which is what lets the code
  // that moved here keep reading `param_.frame.map_frame` and the rest unchanged. HyperParameters
  // fills one of these; it cannot hold it, because these fields serve the node too.
  struct Params
  {
    struct Frame
    {
      std::string base_frame;
      std::string ndt_base_frame;
      std::string map_frame;
    } frame{};

    struct SensorPoints
    {
      double timeout_sec{};
      double required_distance{};
    } sensor_points{};

    bool ndt_regularization_enable{};

    struct Validation
    {
      double initial_pose_timeout_sec{};
      double initial_pose_distance_tolerance_m{};
      double initial_to_result_distance_tolerance_m{};
      double critical_upper_bound_exe_time_ms{};
    } validation{};

    struct ScoreEstimation
    {
      ConvergedParamType converged_param_type{};
      double converged_param_transform_probability{};
      double converged_param_nearest_voxel_transformation_likelihood{};
      struct NoGroundPoints
      {
        bool enable{};
        double z_margin_for_ground_removal{};
      } no_ground_points{};
    } score_estimation{};

    struct Covariance
    {
      std::array<double, 36> output_pose_covariance{};

      struct CovarianceEstimation
      {
        CovarianceEstimationType covariance_estimation_type{};
        std::vector<double> initial_pose_offset_model_x;
        std::vector<double> initial_pose_offset_model_y;
        double temperature{};
        double scale_factor{};
      } covariance_estimation{};
    } covariance{};
  };

  // The node's TF lookup, already done. `transform` is empty when it failed, and `error` then
  // carries the exception text, which the diagnostics message prefixes -- a bool could not say
  // that.
  struct TransformLookup
  {
    std::optional<geometry_msgs::msg::TransformStamped> transform;
    std::string error;
  };

  struct ScanInput
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr scan;  // in the sensor frame
    // `now()` at the callback's start, for the delay check. This module reads no clock.
    builtin_interfaces::msg::Time now;
    bool is_activated{};
    // sensor frame -> base frame, looked up by the node.
    TransformLookup base_from_sensor;
    // Whether anyone subscribes to `voxel_score_points`. Colouring every point by its score is
    // expensive, so it is only done when someone is listening.
    bool voxel_score_points_wanted{};
  };

  // Everything one scan match produces for the outside world, assembled while the NDT lock is
  // held and published by the caller once it is released. Message types and PODs only.
  //
  // What is conditional is an `optional`, never a flag: the caller's publish is allowed
  // `if (opt)` and nothing else. A `bool` at the publish site is how the asymmetry
  // `NonConvergedScanSuppressesPoseButStillBroadcastsTf` pins -- tf unconditional, pose only when
  // converged -- gets broken.
  struct ScanMatchingOutput
  {
    builtin_interfaces::msg::Time stamp;

    // Went out ahead of everything below, from inside the covariance estimate.
    std::optional<geometry_msgs::msg::PoseArray> multi_ndt_pose;
    std::optional<geometry_msgs::msg::PoseArray> multi_initial_pose;

    PoseWithCovarianceStamped initial_pose_with_covariance;
    float exe_time_ms{};
    float transform_probability{};
    float nearest_voxel_transformation_likelihood{};
    int32_t iteration_num{};

    geometry_msgs::msg::TransformStamped tf;  // unconditional

    // Engaged together, only when the match converged.
    std::optional<geometry_msgs::msg::PoseStamped> ndt_pose;
    std::optional<PoseWithCovarianceStamped> ndt_pose_with_covariance;

    visualization_msgs::msg::MarkerArray ndt_marker;

    geometry_msgs::msg::PoseStamped initial_to_result_relative_pose;
    float initial_to_result_distance{};
    float initial_to_result_distance_old{};
    float initial_to_result_distance_new{};

    sensor_msgs::msg::PointCloud2 points_aligned;
    std::optional<sensor_msgs::msg::PointCloud2> voxel_score_points;

    // The three go together or not at all, following no_ground_points.enable.
    struct NoGroundScore
    {
      sensor_msgs::msg::PointCloud2 points;
      float transform_probability{};
      float nearest_voxel_transformation_likelihood{};
    };
    std::optional<NoGroundScore> no_ground;
  };

  struct Result
  {
    // Keys one through eighteen, the level, the message and the log records. The caller appends
    // `skipping_publish_num` and forwards the whole thing.
    DiagnosticsReport diagnostics;
    // Empty unless the match reached the point of producing something to publish.
    std::optional<ScanMatchingOutput> output;
    // The scan in base frame, once it has passed the distance gate. The caller keeps it: the
    // align service reads it too. Null when the scan never got that far.
    CloudPtr scan_in_baselink_frame;
    // The value `callback_sensor_points` returned, used only to drive the skip counter. Not for
    // deciding what to publish -- that is what the optionals above are for.
    bool succeeded{};
  };

  explicit ScanMatchingModule(Params param);

  // `ndt` is mutated (regularization pose, then align). The caller owns it and is expected to
  // hold its lock across this call. `map_update` is asked what the loaded map covers, which is
  // only answerable once the initial pose has been interpolated.
  [[nodiscard]] Result scan_match(
    const ScanInput & input, NdtType & ndt, MapUpdateModule & map_update);

  // Subscribed poses, and the state kept between callbacks.
  void push_initial_pose(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose, bool is_activated,
    DiagnosticsReport & report);
  void push_regularization_pose(const PoseWithCovarianceStamped::ConstSharedPtr & pose);
  void clear_initial_pose_buffer();
  // Where the map update timer measures from. Not const: reading the guarded value takes its lock.
  [[nodiscard]] std::optional<geometry_msgs::msg::Point> latest_ekf_position();

private:
  struct CovarianceEstimate
  {
    Eigen::Matrix2d covariance{Eigen::Matrix2d::Identity()};
    std::optional<geometry_msgs::msg::PoseArray> multi_ndt_pose;
    std::optional<geometry_msgs::msg::PoseArray> multi_initial_pose;
  };

  [[nodiscard]] CovarianceEstimate estimate_covariance(
    const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
    const builtin_interfaces::msg::Time & sensor_time, NdtType & ndt_ref,
    const CloudPtr & sensor_points_in_baselink_frame);

  void add_regularization_pose(
    const builtin_interfaces::msg::Time & sensor_time, NdtType & ndt_ref);

  [[nodiscard]] pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorize_point_score(
    const CloudPtr & sensor_points_in_map_ptr, const float & lower_nvs, const float & upper_nvs,
    NdtType & ndt_ref) const;

  [[nodiscard]] geometry_msgs::msg::TransformStamped make_tf(
    const builtin_interfaces::msg::Time & sensor_time,
    const geometry_msgs::msg::Pose & result_pose_msg) const;
  [[nodiscard]] geometry_msgs::msg::PoseStamped make_pose(
    const builtin_interfaces::msg::Time & sensor_time,
    const geometry_msgs::msg::Pose & result_pose_msg) const;
  [[nodiscard]] PoseWithCovarianceStamped make_pose_with_covariance(
    const builtin_interfaces::msg::Time & sensor_time,
    const geometry_msgs::msg::Pose & result_pose_msg,
    const std::array<double, 36> & ndt_covariance) const;
  [[nodiscard]] visualization_msgs::msg::MarkerArray make_ndt_marker(
    const builtin_interfaces::msg::Time & sensor_time,
    const std::vector<geometry_msgs::msg::Pose> & pose_array, int marker_slot_num) const;
  void fill_initial_to_result(
    ScanMatchingOutput & output, const geometry_msgs::msg::Pose & result_pose_msg,
    const PoseWithCovarianceStamped & initial_pose_cov_msg,
    const PoseWithCovarianceStamped & initial_pose_old_msg,
    const PoseWithCovarianceStamped & initial_pose_new_msg) const;

  // Two overloads rather than a template, so that pcl_conversions stays out of this header --
  // it is not on `test_core_is_ros_free`'s include path. The voxel score cloud is PointXYZRGB;
  // everything else is PointXYZ.
  [[nodiscard]] sensor_msgs::msg::PointCloud2 make_point_cloud(
    const builtin_interfaces::msg::Time & sensor_time, const std::string & frame_id,
    const CloudPtr & points_in_map) const;
  [[nodiscard]] sensor_msgs::msg::PointCloud2 make_point_cloud(
    const builtin_interfaces::msg::Time & sensor_time, const std::string & frame_id,
    const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> & points_in_map) const;

  Params param_;

  PoseInterpolationBuffer initial_pose_buffer_;
  // Only created when param_.ndt_regularization_enable is set.
  std::unique_ptr<PoseInterpolationBuffer> regularization_pose_buffer_;

  // Where the map update timer measures from, written by push_initial_pose() on the initial pose
  // callback group and read by the timer on its own.
  Guarded<std::optional<geometry_msgs::msg::Point>> latest_ekf_position_{std::nullopt};
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__SCAN_MATCHING_MODULE_HPP_
