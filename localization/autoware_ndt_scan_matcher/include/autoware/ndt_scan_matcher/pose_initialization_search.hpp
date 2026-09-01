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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__POSE_INITIALIZATION_SEARCH_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__POSE_INITIALIZATION_SEARCH_HPP_

#include "diagnostics_report.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "particle.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <optional>
#include <string>

namespace autoware::ndt_scan_matcher
{

struct PoseInitializationParams
{
  int64_t particles_num{};
  int64_t n_startup_trials{};
  std::string map_frame;
  // A best score at or below this is reported as not reliable. The service answers with it; it
  // does not make the call fail.
  double converged_param_nearest_voxel_transformation_likelihood{};
};

struct PoseInitializationEstimate
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
  double score{};
  bool reliable{};
};

// What the search produced, returned rather than written through an out-parameter, matching
// MapUpdateModule::UpdateResult: the caller applies the reports of the calls it makes in the order
// it makes them, which is what keeps the joined diagnostics message in order.
struct PoseInitializationResult
{
  // Empty when a precondition failed; `diagnostics` says which.
  std::optional<PoseInitializationEstimate> estimate;
  DiagnosticsReport diagnostics;

  // What the caller publishes. Empty together with `estimate`, so the caller's publish is allowed
  // `if (opt)` and nothing else.
  //
  // One cloud, not one per particle: the scan aligned by the winning pose. The search used to
  // publish every particle's cloud as it went, which put a full lidar scan on `points_aligned`
  // once per particle -- a hundred on the shipped configuration, against a queue of ten -- and
  // forced the caller to step the search so it could publish between particles. The markers below
  // carry the same exploration, arrow by arrow, at a fraction of the size.
  std::optional<sensor_msgs::msg::PointCloud2> best_points_aligned;
  // Every particle's initial and result pose, coloured by score, iteration count and index.
  std::optional<visualization_msgs::msg::MarkerArray> search_markers;
};

// The `ndt_align` service's search: sample initial poses with a tree-structured Parzen estimator,
// align from each, and keep the best.
//
// A free function rather than a module: nothing here outlives a call, so there is no state for an
// object to hold -- unlike MapUpdateModule, which owns the loaded map, or PoseInterpolationBuffer,
// which owns the buffer.
//
// No ROS beyond message types, and nothing injected. `now` stamps the debug markers, which is the
// only thing here that wanted a clock. `ndt` is aligned against and mutated; the caller owns it
// and is expected to hold its lock across the call.
[[nodiscard]] PoseInitializationResult search_initial_pose(
  const PoseInitializationParams & param,
  pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> & ndt,
  const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> & scan_in_baselink_frame,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_in_map_frame,
  const builtin_interfaces::msg::Time & now);

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__POSE_INITIALIZATION_SEARCH_HPP_
