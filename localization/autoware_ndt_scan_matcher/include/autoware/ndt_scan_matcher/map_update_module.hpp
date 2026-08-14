// Copyright 2022 Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_

#include "guarded.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::ndt_scan_matcher
{

class MapUpdateModule
{
public:
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using NdtPtrType = std::shared_ptr<NdtType>;

  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

  // Injected by the ROS node: given a differential map request, returns the response,
  // or nullptr if the map could not be fetched (e.g. the service is unavailable).
  using PcdLoaderFunction = std::function<GetDifferentialPointCloudMap::Response::SharedPtr(
    const GetDifferentialPointCloudMap::Request::SharedPtr &)>;

  // Owned by this module rather than by HyperParameters so that declaring it costs no dependency
  // on rclcpp. HyperParameters embeds this type and fills it in from the node's parameters.
  struct Params
  {
    // How far the vehicle has to move before the periodic update runs again [m].
    double update_distance{};
    // Radius of the map requested from the pcd loader [m].
    double map_radius{};
    // Radius of the input lidar range [m]. Once map_radius - lidar_radius is exceeded the loaded
    // map no longer covers the sensor and has to be rebuilt.
    double lidar_radius{};
    // Whether to produce the merged loaded map for the debug publish.
    bool publish_loaded_map{};
  };

  // Severity of a diagnostics update. Mirrors diagnostic_msgs::msg::DiagnosticStatus levels so
  // this module needs no ROS diagnostics dependency.
  enum class DiagnosticLevel : int8_t { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };

  // A single diagnostic key/value. The value keeps its type so the ROS node can format it exactly
  // the way DiagnosticsInterface would (e.g. bool as "True"/"False").
  struct DiagnosticKeyValue
  {
    std::string key;
    std::variant<bool, int64_t, double, std::string> value;
  };

  // Diagnostics accumulated while updating the map: key/values plus an overall status (level +
  // message). Returned to the ROS node, which forwards it to a DiagnosticsInterface. Plain data,
  // kept ROS-free on purpose.
  struct DiagnosticsReport
  {
    DiagnosticLevel level{DiagnosticLevel::OK};
    std::string message;
    std::vector<DiagnosticKeyValue> key_values;

    void add_key_value(DiagnosticKeyValue key_value) { key_values.push_back(std::move(key_value)); }

    // Accumulates like DiagnosticsInterface: raises the level and appends the message.
    void update_level_and_message(DiagnosticLevel new_level, const std::string & new_message)
    {
      if (static_cast<int8_t>(new_level) > static_cast<int8_t>(DiagnosticLevel::OK)) {
        if (!message.empty()) {
          message += "; ";
        }
        message += new_message;
      }
      if (static_cast<int8_t>(new_level) > static_cast<int8_t>(level)) {
        level = new_level;
      }
    }
  };

  // Result of update(): the map to install, plus what to report for this call.
  struct MapUpdate
  {
    // The NDT the caller should install in place of the one it currently uses, or null when the
    // map did not change. Returned by value rather than written through a shared pointer so that
    // this module never has to reach into state the ROS node owns.
    NdtPtrType ndt;
    DiagnosticsReport diagnostics;
    // The merged loaded point cloud map for debugging. Set only when publish_loaded_map is enabled
    // and the map changed; the ROS node publishes it as-is.
    std::optional<sensor_msgs::msg::PointCloud2> loaded_pcd_map;
  };

private:
  // Outcome of a single differential map load. "Failed" (the pcd_loader could not be reached) and
  // "NoChange" (the loader had nothing to add or remove) must stay distinguishable: the former
  // leaves the NDT without the map for this position and has to be retried, while the latter means
  // the NDT is already up to date and is the normal steady state.
  enum class LoadResult : uint8_t { Updated, NoChange, Failed };

  // The point clouds of the map cells the NDT currently holds, keyed by cell id, together with the
  // stamp of the response they came from. Only populated when param_.publish_loaded_map is enabled.
  // Keeping the cells separately (rather than one pre-merged cloud) is what allows a removed cell
  // to be dropped again, so that the debug publish keeps matching the map the NDT actually holds.
  struct LoadedPcdMap
  {
    std::map<std::string, sensor_msgs::msg::PointCloud2> cells;
    builtin_interfaces::msg::Time stamp;
  };

  // The map being built, kept between calls so that each differential load starts from the
  // generation the caller is currently using. Guarded as a unit because the ROS node drives
  // update() from two callback groups that can run on different threads, and the underlying
  // MultiVoxelGridCovariance is not safe against concurrent loads.
  struct BuilderState
  {
    NdtPtrType ndt;
    LoadedPcdMap loaded_pcd_map;
  };

public:
  MapUpdateModule(Params param, pclomp::NdtParams ndt_params, PcdLoaderFunction pcd_loader);

  // Loads the map around `position` and returns the NDT the caller should install. When to call
  // this is the caller's decision; see needs_update() and out_of_map_range().
  [[nodiscard]] MapUpdate update(const geometry_msgs::msg::Point & position);

  // True once the vehicle has moved far enough for a periodic update to be worth running.
  [[nodiscard]] bool needs_update(const geometry_msgs::msg::Point & position);

  // True while the lidar range around `position` is not covered by the loaded map. The next
  // update() then rebuilds from scratch instead of loading differentially, and until it succeeds
  // the caller should treat the map it has installed as unusable at `position`.
  [[nodiscard]] bool out_of_map_range(const geometry_msgs::msg::Point & position);

  // Distance moved since the position at which the map was last loaded successfully, or nullopt
  // if no load has succeeded yet. Exposed so that the caller can tell "no map loaded yet" apart
  // from "the map has fallen behind", which read the same through out_of_map_range().
  [[nodiscard]] std::optional<double> distance_from_last_update(
    const geometry_msgs::msg::Point & position);

private:
  // Rebuilds the map from scratch, adopting the result only once the load has succeeded so that a
  // failing pcd_loader leaves the previously loaded map intact.
  // Precondition: builder_state_'s lock must be held; the caller passes in its guarded value.
  LoadResult rebuild_map(
    BuilderState & builder_state, const geometry_msgs::msg::Point & position,
    DiagnosticsReport & diagnostics);

  // Loads the differential map around `position` into `ndt`, mirroring the added and removed cells
  // into `loaded_pcd_map` when param_.publish_loaded_map is enabled. Leaves both untouched unless
  // it returns LoadResult::Updated.
  LoadResult load_differential_map(
    const geometry_msgs::msg::Point & position, NdtType & ndt, LoadedPcdMap & loaded_pcd_map,
    DiagnosticsReport & diagnostics);

  // The only place that reaches outside this module. Returns the loader's response, or nullptr if
  // it could not be obtained.
  [[nodiscard]] GetDifferentialPointCloudMap::Response::SharedPtr fetch_differential_map(
    const geometry_msgs::msg::Point & position, const std::vector<std::string> & cached_ids,
    DiagnosticsReport & diagnostics);

  // Applies an already fetched response to `ndt` and to the debug map. Performs no I/O, so every
  // call out of this module stays in fetch_differential_map() above.
  void apply_differential_map(
    const GetDifferentialPointCloudMap::Response & response, NdtType & ndt,
    LoadedPcdMap & loaded_pcd_map);

  // Merges the per-cell clouds into the single cloud the ROS node publishes. Best effort: a cell
  // whose field layout does not match the others is skipped and reported through `diagnostics`.
  [[nodiscard]] static sensor_msgs::msg::PointCloud2 merge_loaded_pcd_map(
    const LoadedPcdMap & loaded_pcd_map, DiagnosticsReport & diagnostics);

  PcdLoaderFunction pcd_loader_;

  // To prevent deadlocks, acquire builder_state_ before last_update_position_. This module never
  // touches the NDT the caller has installed, so the caller may hold its own lock across update().
  Guarded<BuilderState> builder_state_;
  Guarded<std::optional<geometry_msgs::msg::Point>> last_update_position_{std::nullopt};

  Params param_;
  pclomp::NdtParams ndt_params_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
