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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__POSE_INITIALIZATION_MODULE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__POSE_INITIALIZATION_MODULE_HPP_

#include "diagnostics_report.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "particle.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

namespace autoware::ndt_scan_matcher
{

// The `ndt_align` service's search: sample initial poses with a tree-structured Parzen estimator,
// align from each, and keep the best.
//
// No ROS beyond message types. The two things that needed the node -- the clock for the debug
// marker stamps, and the publishers the search feeds while it runs -- are handled by giving the
// caller each particle as it is produced.
class PoseInitializationModule
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointSource>>;

  struct Params
  {
    int64_t particles_num{};
    int64_t n_startup_trials{};
    std::string map_frame;
    // A best score at or below this is reported as not reliable. The service answers with it; it
    // does not make the call fail.
    double converged_param_nearest_voxel_transformation_likelihood{};
  };

  // One particle's outcome, handed to the caller as it is produced rather than collected and
  // returned at the end. The search publishes its progress while it runs -- deliberately, "to see
  // the progress and to avoid dropping data" -- and
  // `SuccessfulAlignEmitsTheseKeysAndOneCloudPerParticle` pins one cloud per particle, so
  // collecting would change what goes out.
  struct Progress
  {
    int64_t index{};
    Particle particle;
    // The scan transformed by this particle's result pose, stamped and framed ready to publish.
    sensor_msgs::msg::PointCloud2 sensor_points_in_map;
  };
  using ProgressCallback = std::function<void(const Progress &)>;

  struct Result
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
    double score{};
    bool reliable{};
  };

  PoseInitializationModule(Params params, ProgressCallback on_progress);

  // Returns nullopt when there is nothing to align: no map in `ndt`, or no scan received yet.
  // Both are reported through `diagnostics`, which the caller keeps threading through the rest of
  // the service call.
  //
  // `ndt` is mutated by the search (it aligns from each sampled pose). The caller holds it, and is
  // expected to hold its lock across this call.
  [[nodiscard]] std::optional<Result> estimate(
    NdtType & ndt, const CloudPtr & sensor_points_in_baselink_frame,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_in_map_frame,
    DiagnosticsReport & diagnostics);

private:
  Params param_;
  ProgressCallback on_progress_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__POSE_INITIALIZATION_MODULE_HPP_
