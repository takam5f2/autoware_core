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

#include <autoware/ndt_scan_matcher/map_update_module.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{

MapUpdateModule::MapUpdateModule(
  HyperParameters::DynamicMapLoading param, pclomp::NdtParams ndt_params,
  PcdLoaderFunction pcd_loader)
: pcd_loader_(std::move(pcd_loader)), param_(param), ndt_params_(ndt_params)
{
  builder_state_.with([&](auto & builder_state) {
    builder_state.ndt = std::make_shared<NdtType>();
    builder_state.ndt->setParams(ndt_params_);
  });
}

MapUpdateModule::MapUpdate MapUpdateModule::update(const geometry_msgs::msg::Point & position)
{
  MapUpdate result;
  DiagnosticsReport & diagnostics = result.diagnostics;

  builder_state_.with([&](auto & builder_state) {
    // A rebuild is needed exactly while the loaded map does not cover the lidar range, which is
    // what out_of_map_range() reports.
    const bool rebuild = out_of_map_range(position);
    diagnostics.add_key_value({"is_need_rebuild", rebuild});

    const LoadResult load_result =
      rebuild ? rebuild_map(builder_state, position, diagnostics)
              : load_differential_map(
                  position, *builder_state.ndt, builder_state.loaded_pcd_map, diagnostics);

    // check is_updated_map
    diagnostics.add_key_value({"is_updated_map", load_result == LoadResult::Updated});

    if (load_result == LoadResult::Failed) {
      // Deliberately leave last_update_position_ untouched. It is the reference for both
      // needs_update() and out_of_map_range(), so overwriting it here would make the module claim
      // coverage it never obtained: the caller would stop being told that the map has fallen
      // behind, and the periodic update would go quiet until the vehicle has moved another
      // param_.update_distance.
      return;
    }

    // Updated and NoChange both mean the map is now correct for this position.
    last_update_position_.with([&](auto & pos) { pos = position; });

    if (load_result == LoadResult::NoChange) {
      return;
    }

    // Hand out a copy. builder_state.ndt stays here as the master for the next differential load,
    // so that its cached cell ids never fall a generation behind what the caller is using.
    result.ndt = std::make_shared<NdtType>(*builder_state.ndt);

    if (param_.publish_loaded_map) {
      result.loaded_pcd_map = merge_loaded_pcd_map(builder_state.loaded_pcd_map, diagnostics);
    }
  });

  return result;
}

bool MapUpdateModule::needs_update(const geometry_msgs::msg::Point & position)
{
  const auto moved_distance = distance_from_last_update(position);
  return !moved_distance || *moved_distance > param_.update_distance;
}

bool MapUpdateModule::out_of_map_range(const geometry_msgs::msg::Point & position)
{
  const auto moved_distance = distance_from_last_update(position);
  return !moved_distance || *moved_distance + param_.lidar_radius > param_.map_radius;
}

std::optional<double> MapUpdateModule::distance_from_last_update(
  const geometry_msgs::msg::Point & position)
{
  const auto last_update_position =
    last_update_position_.with([](const auto & pos) { return pos; });

  if (last_update_position == std::nullopt) {
    return std::nullopt;
  }

  return std::hypot(position.x - last_update_position->x, position.y - last_update_position->y);
}

MapUpdateModule::LoadResult MapUpdateModule::rebuild_map(
  BuilderState & builder_state, const geometry_msgs::msg::Point & position,
  DiagnosticsReport & diagnostics)
{
  // Load into a fresh NDT and adopt it only once the load has succeeded. Clearing the existing one
  // up front would mean giving up the map we still have before knowing whether a new one can be
  // obtained, so a transient pcd_loader failure would leave us with no map at all.
  auto rebuilt_ndt = std::make_shared<NdtType>();
  rebuilt_ndt->setParams(ndt_params_);
  LoadedPcdMap rebuilt_pcd_map;

  const LoadResult load_result =
    load_differential_map(position, *rebuilt_ndt, rebuilt_pcd_map, diagnostics);

  if (load_result != LoadResult::Updated) {
    diagnostics.update_level_and_message(
      DiagnosticLevel::ERROR,
      "Map load failed. If this happens with initial position estimation, make sure that"
      "(1) the initial position matches the pcd map and (2) the map_loader is working "
      "properly.");
    return load_result;
  }

  builder_state.ndt = std::move(rebuilt_ndt);
  builder_state.loaded_pcd_map = std::move(rebuilt_pcd_map);

  return LoadResult::Updated;
}

MapUpdateModule::LoadResult MapUpdateModule::load_differential_map(
  const geometry_msgs::msg::Point & position, NdtType & ndt, LoadedPcdMap & loaded_pcd_map,
  DiagnosticsReport & diagnostics)
{
  diagnostics.add_key_value(
    {"maps_size_before", static_cast<int64_t>(ndt.getCurrentMapIDs().size())});

  const auto response = fetch_differential_map(position, ndt.getCurrentMapIDs(), diagnostics);
  if (!response) {
    return LoadResult::Failed;
  }

  if (response->new_pointcloud_with_ids.empty() && response->ids_to_remove.empty()) {
    return LoadResult::NoChange;
  }

  const auto exe_start_time = std::chrono::system_clock::now();

  apply_differential_map(*response, ndt, loaded_pcd_map);

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<double>(duration_micro_sec) / 1000.0;
  diagnostics.add_key_value({"map_update_execution_time", exe_time});
  diagnostics.add_key_value(
    {"maps_size_after", static_cast<int64_t>(ndt.getCurrentMapIDs().size())});

  return LoadResult::Updated;
}

MapUpdateModule::GetDifferentialPointCloudMap::Response::SharedPtr
MapUpdateModule::fetch_differential_map(
  const geometry_msgs::msg::Point & position, const std::vector<std::string> & cached_ids,
  DiagnosticsReport & diagnostics)
{
  auto request = std::make_shared<GetDifferentialPointCloudMap::Request>();

  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(param_.map_radius);
  request->cached_ids = cached_ids;

  // The ROS node performs the actual service call.
  const auto response = pcd_loader_(request);

  // check is_succeed_call_pcd_loader
  const bool is_succeed_call_pcd_loader = (response != nullptr);
  diagnostics.add_key_value({"is_succeed_call_pcd_loader", is_succeed_call_pcd_loader});
  if (!is_succeed_call_pcd_loader) {
    diagnostics.update_level_and_message(
      DiagnosticLevel::WARN, "pcd_loader service is not working.");
    return nullptr;
  }

  diagnostics.add_key_value(
    {"maps_to_add_size", static_cast<int64_t>(response->new_pointcloud_with_ids.size())});
  diagnostics.add_key_value(
    {"maps_to_remove_size", static_cast<int64_t>(response->ids_to_remove.size())});

  return response;
}

void MapUpdateModule::apply_differential_map(
  const GetDifferentialPointCloudMap::Response & response, NdtType & ndt,
  LoadedPcdMap & loaded_pcd_map)
{
  // Add pcd
  for (const auto & map : response.new_pointcloud_with_ids) {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointTarget>>();

    pcl::fromROSMsg(map.pointcloud, *cloud);
    ndt.addTarget(cloud, map.cell_id);

    // Mirror the cell for the debug publish. It is kept under its cell id so that it can be
    // dropped again below when the map loader asks for the cell to be removed.
    if (param_.publish_loaded_map) {
      loaded_pcd_map.cells[map.cell_id] = map.pointcloud;
    }
  }

  // Remove pcd
  for (const std::string & map_id_to_remove : response.ids_to_remove) {
    ndt.removeTarget(map_id_to_remove);

    if (param_.publish_loaded_map) {
      loaded_pcd_map.cells.erase(map_id_to_remove);
    }
  }

  // Has to stay in the same function as the add and remove above: until the kdtree is rebuilt the
  // NDT is inconsistent, because grid_list_ still holds the slots of the removed cells and both
  // the voxel centroids and the leaf pointers are stale.
  ndt.createVoxelKdtree();

  if (param_.publish_loaded_map) {
    loaded_pcd_map.stamp = response.header.stamp;
  }
}

sensor_msgs::msg::PointCloud2 MapUpdateModule::merge_loaded_pcd_map(
  const LoadedPcdMap & loaded_pcd_map, DiagnosticsReport & diagnostics)
{
  sensor_msgs::msg::PointCloud2 merged;
  int64_t skipped_cells = 0;

  for (const auto & cell : loaded_pcd_map.cells) {
    // Merge the PointCloud2 messages directly to avoid a PCL round-trip. This only fails when the
    // cells disagree on their field layout, which the map loader should never produce.
    if (!pcl::concatenatePointCloud(merged, cell.second, merged)) {
      ++skipped_cells;
    }
  }

  diagnostics.add_key_value({"loaded_pcd_map_skipped_cells", skipped_cells});
  if (skipped_cells > 0) {
    diagnostics.update_level_and_message(
      DiagnosticLevel::WARN, "Some map cells could not be merged for the loaded map publish.");
  }

  merged.header.stamp = loaded_pcd_map.stamp;
  merged.header.frame_id = "map";

  return merged;
}

}  // namespace autoware::ndt_scan_matcher
