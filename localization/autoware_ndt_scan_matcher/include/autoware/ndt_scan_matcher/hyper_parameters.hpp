// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use node file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_

#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "ndt_scan_matcher.hpp"
#include "scan_matching_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{

// ConvergedParamType and CovarianceEstimationType are declared by ScanMatchingModule, which owns
// the parameters that use them and cannot depend on rclcpp.
// Rejects a parameter whose integer names none of an enum's values. `static_cast` accepts any
// integer, so without this a mistyped config reaches the code that uses it: once per scan for
// `converged_param_type`, which aligned first and only then found it could not judge the result,
// and silently for the covariance type, which fell through to a fixed value with nothing said.
inline void require_enum_value(
  const std::string & name, const int64_t value, const std::initializer_list<int64_t> valid,
  const std::string & expected)
{
  if (std::find(valid.begin(), valid.end(), value) != valid.end()) {
    return;
  }
  std::stringstream message;
  message << "Invalid " << name << ": " << value << ". Expected " << expected << ".";
  throw std::runtime_error(message.str());
}

struct HyperParameters
{
  struct Frame
  {
    std::string base_frame{};
    std::string ndt_base_frame{};
    std::string map_frame{};
  } frame{};

  struct SensorPoints
  {
    double timeout_sec{};
    double required_distance{};
  } sensor_points{};

  pclomp::NdtParams ndt{};
  bool ndt_regularization_enable{};

  struct InitialPoseEstimation
  {
    int64_t particles_num{};
    int64_t n_startup_trials{};
  } initial_pose_estimation{};

  struct Validation
  {
    double initial_pose_timeout_sec{};
    double initial_pose_distance_tolerance_m{};
    double initial_to_result_distance_tolerance_m{};
    double critical_upper_bound_exe_time_ms{};
    int64_t skipping_publish_num{};
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
      std::vector<double> initial_pose_offset_model_x{};
      std::vector<double> initial_pose_offset_model_y{};
      double temperature{};
      double scale_factor{};
    } covariance_estimation{};
  } covariance{};

  // Defined by MapUpdateModule itself: that module has no rclcpp dependency, and declaring its
  // parameter type here would have forced one on it.
  MapUpdateModule::Params dynamic_map_loading{};

public:
  explicit HyperParameters(rclcpp::Node * node)
  {
    frame.base_frame = node->declare_parameter<std::string>("frame.base_frame");
    frame.ndt_base_frame = node->declare_parameter<std::string>("frame.ndt_base_frame");
    frame.map_frame = node->declare_parameter<std::string>("frame.map_frame");

    sensor_points.timeout_sec = node->declare_parameter<double>("sensor_points.timeout_sec");
    sensor_points.required_distance =
      node->declare_parameter<double>("sensor_points.required_distance");

    ndt.trans_epsilon = node->declare_parameter<double>("ndt.trans_epsilon");
    ndt.step_size = node->declare_parameter<double>("ndt.step_size");
    ndt.resolution = node->declare_parameter<float>("ndt.resolution");
    ndt.max_iterations = static_cast<int>(node->declare_parameter<int64_t>("ndt.max_iterations"));
    ndt.num_threads = static_cast<int>(node->declare_parameter<int64_t>("ndt.num_threads"));
    ndt.num_threads = std::max(ndt.num_threads, 1);
    ndt_regularization_enable = node->declare_parameter<bool>("ndt.regularization.enable");
    ndt.regularization_scale_factor =
      static_cast<float>(node->declare_parameter<float>("ndt.regularization.scale_factor"));

    initial_pose_estimation.particles_num =
      node->declare_parameter<int64_t>("initial_pose_estimation.particles_num");
    initial_pose_estimation.n_startup_trials =
      node->declare_parameter<int64_t>("initial_pose_estimation.n_startup_trials");

    validation.initial_pose_timeout_sec =
      node->declare_parameter<double>("validation.initial_pose_timeout_sec");
    validation.initial_pose_distance_tolerance_m =
      node->declare_parameter<double>("validation.initial_pose_distance_tolerance_m");
    validation.initial_to_result_distance_tolerance_m =
      node->declare_parameter<double>("validation.initial_to_result_distance_tolerance_m");
    validation.critical_upper_bound_exe_time_ms =
      node->declare_parameter<double>("validation.critical_upper_bound_exe_time_ms");
    validation.skipping_publish_num =
      node->declare_parameter<int64_t>("validation.skipping_publish_num");

    const int64_t converged_param_type_tmp =
      node->declare_parameter<int64_t>("score_estimation.converged_param_type");
    require_enum_value(
      "score_estimation.converged_param_type", converged_param_type_tmp, {0, 1},
      "0 (transform probability) or 1 (nearest voxel transformation likelihood)");
    score_estimation.converged_param_type =
      static_cast<ConvergedParamType>(converged_param_type_tmp);
    score_estimation.converged_param_transform_probability =
      node->declare_parameter<double>("score_estimation.converged_param_transform_probability");
    score_estimation.converged_param_nearest_voxel_transformation_likelihood =
      node->declare_parameter<double>(
        "score_estimation.converged_param_nearest_voxel_transformation_likelihood");
    score_estimation.no_ground_points.enable =
      node->declare_parameter<bool>("score_estimation.no_ground_points.enable");
    score_estimation.no_ground_points.z_margin_for_ground_removal = node->declare_parameter<double>(
      "score_estimation.no_ground_points.z_margin_for_ground_removal");

    std::vector<double> output_pose_covariance =
      node->declare_parameter<std::vector<double>>("covariance.output_pose_covariance");
    for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
      covariance.output_pose_covariance[i] = output_pose_covariance[i];
    }
    const int64_t covariance_estimation_type_tmp = node->declare_parameter<int64_t>(
      "covariance.covariance_estimation.covariance_estimation_type");
    require_enum_value(
      "covariance.covariance_estimation.covariance_estimation_type", covariance_estimation_type_tmp,
      {0, 1, 2, 3},
      "0 (fixed value), 1 (Laplace approximation), 2 (multi NDT) or 3 (multi NDT score)");
    covariance.covariance_estimation.covariance_estimation_type =
      static_cast<CovarianceEstimationType>(covariance_estimation_type_tmp);
    covariance.covariance_estimation.initial_pose_offset_model_x =
      node->declare_parameter<std::vector<double>>(
        "covariance.covariance_estimation.initial_pose_offset_model_x");
    covariance.covariance_estimation.initial_pose_offset_model_y =
      node->declare_parameter<std::vector<double>>(
        "covariance.covariance_estimation.initial_pose_offset_model_y");
    if (
      covariance.covariance_estimation.initial_pose_offset_model_x.size() !=
      covariance.covariance_estimation.initial_pose_offset_model_y.size()) {
      std::stringstream message;
      message << "Invalid initial pose offset model parameters."
              << "Please make sure that the number of elements in "
              << "initial_pose_offset_model_x and initial_pose_offset_model_y are the same.";
      throw std::runtime_error(message.str());
    }
    covariance.covariance_estimation.temperature =
      node->declare_parameter<double>("covariance.covariance_estimation.temperature");
    covariance.covariance_estimation.scale_factor =
      node->declare_parameter<double>("covariance.covariance_estimation.scale_factor");

    dynamic_map_loading.update_distance =
      node->declare_parameter<double>("dynamic_map_loading.update_distance");
    dynamic_map_loading.map_radius =
      node->declare_parameter<double>("dynamic_map_loading.map_radius");
    dynamic_map_loading.lidar_radius =
      node->declare_parameter<double>("dynamic_map_loading.lidar_radius");
    dynamic_map_loading.publish_loaded_map =
      node->declare_parameter<bool>("dynamic_map_loading.publish_loaded_map");
  }

  // The declared parameters as the core wants them.
  //
  // This adapter is the only thing in the package that may see both sides: NdtScanMatcher::Params
  // cannot take a HyperParameters, because naming this type would put rclcpp back into the core.
  // The dependency runs one way -- the adapter knows the core, the core knows nothing of it.
  //
  // The copying below is field by field rather than by holding the core's structs, because most of
  // these values serve the node as well as the core.
  [[nodiscard]] NdtScanMatcher::Params to_core_params() const
  {
    NdtScanMatcher::Params param;
    param.ndt = ndt;
    param.map_update = dynamic_map_loading;
    param.skipping_publish_num = validation.skipping_publish_num;
    param.pose_initialization = PoseInitializationParams{
      initial_pose_estimation.particles_num, initial_pose_estimation.n_startup_trials,
      frame.map_frame, score_estimation.converged_param_nearest_voxel_transformation_likelihood};

    ScanMatchingModule::Params & scan_matching = param.scan_matching;
    scan_matching.frame.base_frame = frame.base_frame;
    scan_matching.frame.ndt_base_frame = frame.ndt_base_frame;
    scan_matching.frame.map_frame = frame.map_frame;
    scan_matching.sensor_points.timeout_sec = sensor_points.timeout_sec;
    scan_matching.sensor_points.required_distance = sensor_points.required_distance;
    scan_matching.ndt_regularization_enable = ndt_regularization_enable;
    scan_matching.validation = {
      validation.initial_pose_timeout_sec, validation.initial_pose_distance_tolerance_m,
      validation.initial_to_result_distance_tolerance_m,
      validation.critical_upper_bound_exe_time_ms};
    scan_matching.score_estimation.converged_param_type = score_estimation.converged_param_type;
    scan_matching.score_estimation.converged_param_transform_probability =
      score_estimation.converged_param_transform_probability;
    scan_matching.score_estimation.converged_param_nearest_voxel_transformation_likelihood =
      score_estimation.converged_param_nearest_voxel_transformation_likelihood;
    scan_matching.score_estimation.no_ground_points = {
      score_estimation.no_ground_points.enable,
      score_estimation.no_ground_points.z_margin_for_ground_removal};
    scan_matching.covariance.output_pose_covariance = covariance.output_pose_covariance;
    scan_matching.covariance.covariance_estimation = {
      covariance.covariance_estimation.covariance_estimation_type,
      covariance.covariance_estimation.initial_pose_offset_model_x,
      covariance.covariance_estimation.initial_pose_offset_model_y,
      covariance.covariance_estimation.temperature, covariance.covariance_estimation.scale_factor};
    return param;
  }
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_
