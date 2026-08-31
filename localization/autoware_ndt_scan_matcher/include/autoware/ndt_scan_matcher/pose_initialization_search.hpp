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

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::localization_util
{
// Which optimizer drives the search is an implementation detail; naming it here would put
// localization_util on the include path of everything that includes this header.
class TreeStructuredParzenEstimator;
}  // namespace autoware::localization_util

namespace autoware::ndt_scan_matcher
{

// The `ndt_align` service's search: sample initial poses with a tree-structured Parzen estimator,
// align from each, and keep the best.
//
// One object per request rather than a long-lived module, because nothing here outlives a request
// -- unlike MapUpdateModule, which owns the loaded map, or PoseInterpolationBuffer, which owns the
// buffer. Making it a module would have meant a class whose only job was to hold `Params` and hand
// out searches.
//
// No ROS beyond message types, and nothing injected. The two things that needed the node -- the
// clock for the debug marker stamps, and the publishers fed while the search runs -- stay with the
// caller, which keeps them by stepping the search itself:
//
//   PoseInitializationSearch search{params, ndt, scan, initial_pose};
//   while (const auto progress = search.next()) {
//     publish(*progress);
//   }
//   const auto result = search.finish();
//
// Collecting the particles and returning them at the end would be simpler and is wrong: the search
// publishes as it goes "to see the progress and to avoid dropping data", and `points_aligned` has
// a depth of ten. The characterization suite does not protect this -- it counts the clouds, so
// collect-then-publish passes it -- the original comment does.
//
// Holds the NDT and the scan by reference. Must not outlive them, and the caller is expected to
// hold the NDT's lock for the whole search, as before.
class PoseInitializationSearch
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointSource>>;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  struct Params
  {
    int64_t particles_num{};
    int64_t n_startup_trials{};
    std::string map_frame;
    // A best score at or below this is reported as not reliable. The service answers with it; it
    // does not make the call fail.
    double converged_param_nearest_voxel_transformation_likelihood{};
  };

  // One particle's outcome, handed back as it is produced.
  struct Progress
  {
    int64_t index{};
    Particle particle;
    // The scan transformed by this particle's result pose, stamped and framed ready to publish.
    sensor_msgs::msg::PointCloud2 sensor_points_in_map;
  };

  struct Estimate
  {
    PoseWithCovarianceStamped pose_with_covariance;
    double score{};
    bool reliable{};
  };

  // What the search produced, returned rather than written through an out-parameter, matching
  // MapUpdateModule::UpdateResult: the caller applies the reports of the calls it makes in the
  // order it makes them, which is what keeps the joined diagnostics message in order.
  struct Result
  {
    // Empty when a precondition failed; `diagnostics` says which.
    std::optional<Estimate> estimate;
    DiagnosticsReport diagnostics;
  };

  // Evaluates the preconditions -- a map in `ndt`, a scan received -- recording both on the report
  // either way. A failure leaves `next()` empty from the start, so the caller needs no branch for
  // it before looping.
  PoseInitializationSearch(
    Params param, NdtType & ndt, CloudPtr sensor_points_in_baselink_frame,
    PoseWithCovarianceStamped initial_pose_in_map_frame);

  PoseInitializationSearch(const PoseInitializationSearch &) = delete;
  PoseInitializationSearch & operator=(const PoseInitializationSearch &) = delete;
  PoseInitializationSearch(PoseInitializationSearch &&) = delete;
  PoseInitializationSearch & operator=(PoseInitializationSearch &&) = delete;
  // Out of line: the optimizer is only complete in the .cpp.
  ~PoseInitializationSearch();

  // Aligns from the next sampled pose. Empty once every particle has been tried.
  [[nodiscard]] std::optional<Progress> next();

  // The best particle, and the diagnostics for the whole search. Call once `next()` has run out;
  // called earlier it reports as though the search had not run.
  [[nodiscard]] Result finish();

private:
  Params param_;
  NdtType & ndt_;
  CloudPtr sensor_points_in_baselink_frame_;
  PoseWithCovarianceStamped initial_pose_in_map_frame_;

  DiagnosticsReport diagnostics_;
  // Null when a precondition failed, which is also what stops `next()` before it starts.
  std::unique_ptr<autoware::localization_util::TreeStructuredParzenEstimator> tpe_;
  std::vector<Particle> particles_;
  int64_t index_{0};
  // Reused across steps, as the single-call version reused it across iterations.
  CloudPtr output_cloud_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__POSE_INITIALIZATION_SEARCH_HPP_
