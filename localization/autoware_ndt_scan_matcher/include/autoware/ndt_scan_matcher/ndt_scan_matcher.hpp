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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_

#include "diagnostics_report.hpp"
#include "guarded.hpp"
#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"
#include "pose_initialization_search.hpp"
#include "scan_matching_module.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

namespace autoware::ndt_scan_matcher
{

// Everything the node is a node *for*: the NDT map, the scan it last kept, and the three things
// done with them -- match a scan, keep the map loaded around the vehicle, search for an initial
// pose. No ROS beyond message types.
//
// One method per event the node receives, and each takes what the node had to look up and returns
// what the node has to emit. Nothing is injected but the pcd loader, which is a service call.
//
// This owns the NDT and its lock, so no caller ever holds that lock: the results below are
// assembled under it and published after it is released.
class NdtScanMatcher
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using NdtPtrType = std::shared_ptr<NdtType>;
  using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointSource>>;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  // Defined once, by the module that reads them; named here so callers need only this header.
  using ScanMatchingOutput = ScanMatchingModule::ScanMatchingOutput;
  using TransformLookup = ScanMatchingModule::TransformLookup;

  struct Params
  {
    pclomp::NdtParams ndt{};
    ScanMatchingModule::Params scan_matching;
    MapUpdateModule::Params map_update;
    PoseInitializationParams pose_initialization;
    // How many consecutive scans may produce nothing to publish before that is a WARN.
    int64_t skipping_publish_num{};
  };

  NdtScanMatcher(Params param, MapUpdateModule::PcdLoaderFunction pcd_loader);

  // --- One method per event the node receives ---

  // `points_raw`.
  struct ScanResult
  {
    // Every key for this scan, including the skip count, which is why the caller adds none.
    DiagnosticsReport diagnostics;
    // Empty unless the match reached the point of producing something to publish. The caller's
    // publish is allowed `if (opt)` and nothing else: a bool at the publish site is how the
    // asymmetry between the unconditional tf and the converged-only pose gets broken.
    std::optional<ScanMatchingOutput> output;
  };
  // The scan, and the two things about it the node had to look up: `now` for the delay check,
  // because this class reads no clock, and the sensor-to-base transform, because it holds no TF
  // buffer.
  [[nodiscard]] ScanResult match_scan(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan,
    const builtin_interfaces::msg::Time & now, const TransformLookup & base_from_sensor);

  // The 1 Hz map update. Reports `is_activated` and `is_set_last_update_position`, and touches
  // nothing else until the trigger service has activated the matcher and an initial pose has
  // arrived. Past that it reports how far the vehicle has travelled since the map was last loaded,
  // raises an error while it has outrun that map, and reloads once it has moved far enough to be
  // worth it.
  struct MapUpdateResult
  {
    DiagnosticsReport diagnostics;
    std::optional<sensor_msgs::msg::PointCloud2> loaded_pcd_map;
  };
  [[nodiscard]] MapUpdateResult update_map_periodically();

  // `ndt_align_srv`.
  struct AlignInput
  {
    // Already in the map frame: the TF is the node's to look up.
    PoseWithCovarianceStamped initial_pose_in_map_frame;
    // Stamps the search's debug markers. This class reads no clock.
    builtin_interfaces::msg::Time now;
  };
  struct AlignResult
  {
    // Empty when the map or the scan was missing, or the search found nothing.
    std::optional<PoseInitializationEstimate> estimate;
    // The map update's report followed by the search's, in that order, which is what keeps the
    // joined message reading as the sequence it is.
    DiagnosticsReport diagnostics;

    std::optional<sensor_msgs::msg::PointCloud2> best_points_aligned;
    std::optional<visualization_msgs::msg::MarkerArray> search_markers;
    std::optional<sensor_msgs::msg::PointCloud2> loaded_pcd_map;
  };
  [[nodiscard]] AlignResult align(const AlignInput & input);

  // `ekf_pose_with_covariance`, `regularization_pose_with_covariance`, `trigger_node_srv`.
  [[nodiscard]] DiagnosticsReport push_initial_pose(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose);
  [[nodiscard]] DiagnosticsReport push_regularization_pose(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose);
  // Whether the matcher should be producing output at all. Activating discards the buffered
  // initial poses: the ones from before the gap are not to be interpolated across it.
  void set_activated(bool activated);

private:
  // Loads the map around `position` and installs it. The NDT lock is held for the whole load only
  // while the installed map does not cover `position`; otherwise just for the swap, so that the
  // load overlaps with whatever is aligning.
  [[nodiscard]] std::optional<sensor_msgs::msg::PointCloud2> install_map_update(
    const geometry_msgs::msg::Point & position, DiagnosticsReport & report);

  Params param_;

  // Set by the trigger service, read from three of the node's callback groups -- hence atomic
  // rather than Guarded, which would serialise readers for nothing. ScanMatchingModule holds a
  // reference to it; declared before it for that reason.
  std::atomic<bool> activated_{false};

  // The installed map. MapUpdateModule keeps its own and hands back a replacement, so this is
  // written only by the swap in install_map_update() and never touched from inside a load.
  Guarded<NdtPtrType> ndt_ptr_;

  // Written by match_scan, read by align. Guarded rather than bare because this class cannot see
  // how its caller schedules those two: in this node they are one MutuallyExclusive callback
  // group and the lock is never contended, but that is the node's arrangement to make, not an
  // assumption this class is entitled to. Kept a leaf -- never held together with another lock.
  Guarded<CloudPtr> scan_in_baselink_frame_;

  // Consecutive scans that produced nothing to publish, reset by a match or by deactivation.
  int64_t skipping_publish_num_{0};

  MapUpdateModule map_update_;
  ScanMatchingModule scan_matching_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
