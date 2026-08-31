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
// No ROS beyond message types, and nothing injected. The two things that needed the node -- the
// clock for the debug marker stamps, and the publishers the search feeds as it runs -- are the
// caller's, and the caller keeps them by driving the search a particle at a time.
class PoseInitializationModule
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

  // What one search produced, returned rather than written through an out-parameter, matching
  // MapUpdateModule::UpdateResult: the caller applies the reports of the calls it makes in the
  // order it makes them, which is what keeps the joined diagnostics message in order.
  struct Result
  {
    // Empty when a precondition failed; `diagnostics` says which.
    std::optional<Estimate> estimate;
    DiagnosticsReport diagnostics;
  };

  // One search, stepped by the caller.
  //
  // The search publishes as it goes -- one `points_aligned` per particle, markers in batches of
  // twenty -- and that is deliberate: "to see the progress and to avoid dropping data". Collecting
  // the particles and returning them at the end would keep the counts that
  // `SuccessfulAlignEmitsTheseKeysAndOneCloudPerParticle` checks, but would hand a hundred clouds
  // to a depth-ten publisher at once, which is the dropping that comment is about. Stepping keeps
  // the timing without the module having to be told how to publish.
  //
  // Holds the NDT and the scan by reference. Must not outlive them, and the caller is expected to
  // hold the NDT's lock for the whole search, as before.
  class Search
  {
  public:
    Search(const Search &) = delete;
    Search & operator=(const Search &) = delete;
    Search(Search &&) = delete;
    Search & operator=(Search &&) = delete;
    // Out of line: the optimizer is only complete in the .cpp.
    ~Search();

    // Aligns from the next sampled pose. Empty once every particle has been tried -- immediately
    // so, if a precondition failed.
    [[nodiscard]] std::optional<Progress> next();

    // The best particle, and the diagnostics for the whole search. Call once `next()` has run out;
    // called earlier it reports as though the search had not run.
    [[nodiscard]] Result finish();

  private:
    friend class PoseInitializationModule;
    Search(
      Params param, NdtType & ndt, CloudPtr sensor_points_in_baselink_frame,
      PoseWithCovarianceStamped initial_pose_in_map_frame);

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

  explicit PoseInitializationModule(Params params);

  // Evaluates the preconditions -- a map in `ndt`, a scan received -- and returns the search to
  // step. Both are recorded on the search's report either way.
  [[nodiscard]] Search begin(
    NdtType & ndt, const CloudPtr & sensor_points_in_baselink_frame,
    const PoseWithCovarianceStamped & initial_pose_in_map_frame) const;

private:
  Params param_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__POSE_INITIALIZATION_MODULE_HPP_
