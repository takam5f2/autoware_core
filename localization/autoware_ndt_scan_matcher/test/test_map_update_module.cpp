// Copyright 2025 Autoware Foundation
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

#include "test_util.hpp"

#include <autoware/ndt_scan_matcher/map_update_module.hpp>

#include <autoware_map_msgs/msg/point_cloud_map_cell_with_id.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

// The map-update logic driven directly: the pcd load is injected as a plain std::function, so the
// whole flow runs in-process with a fake loader and no ROS service or executor.

namespace autoware::ndt_scan_matcher
{
namespace
{
using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

// The stamp the fake loader puts on its responses, which is what a merged debug map carries.
constexpr int32_t response_stamp_sec = 42;

geometry_msgs::msg::Point make_point(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

GetDifferentialPointCloudMap::Response::SharedPtr make_response_with_one_cell()
{
  auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();
  autoware_map_msgs::msg::PointCloudMapCellWithID cell;
  cell.cell_id = "0";
  pcl::toROSMsg(make_sample_half_cubic_pcd(), cell.pointcloud);
  response->new_pointcloud_with_ids.push_back(cell);
  response->header.frame_id = "map";
  response->header.stamp.sec = response_stamp_sec;
  return response;
}

// A fake loader that always returns a single map cell built from the sample cubic pcd.
MapUpdateModule::PcdLoaderFunction make_loader_returning_cell()
{
  return [](const GetDifferentialPointCloudMap::Request::SharedPtr & /*request*/) {
    return make_response_with_one_cell();
  };
}

// A fake loader that returns `num_cells` map cells, each the sample cubic pcd shifted along x so
// the cells do not overlap. Exercises the multi-cell merge in merge_loaded_pcd_map().
MapUpdateModule::PcdLoaderFunction make_loader_returning_cells(const std::size_t num_cells)
{
  return [num_cells](const GetDifferentialPointCloudMap::Request::SharedPtr & /*request*/)
           -> GetDifferentialPointCloudMap::Response::SharedPtr {
    auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();
    for (std::size_t i = 0; i < num_cells; ++i) {
      autoware_map_msgs::msg::PointCloudMapCellWithID cell;
      cell.cell_id = std::to_string(i);
      auto cloud = make_sample_half_cubic_pcd();
      for (auto & point : cloud.points) {
        point.x += static_cast<float>(i) * 20.0F;
      }
      pcl::toROSMsg(cloud, cell.pointcloud);
      response->new_pointcloud_with_ids.push_back(cell);
    }
    response->header.frame_id = "map";
    response->header.stamp.sec = response_stamp_sec;
    return response;
  };
}

// A fake loader that fails, as if the pcd_loader service were unavailable.
MapUpdateModule::PcdLoaderFunction make_failing_loader()
{
  return [](const GetDifferentialPointCloudMap::Request::SharedPtr & /*request*/)
           -> GetDifferentialPointCloudMap::Response::SharedPtr { return nullptr; };
}

// A fake loader that works until `working` is cleared, and fails afterwards.
MapUpdateModule::PcdLoaderFunction make_switchable_loader(const std::shared_ptr<bool> & working)
{
  return [working](const GetDifferentialPointCloudMap::Request::SharedPtr & /*request*/)
           -> GetDifferentialPointCloudMap::Response::SharedPtr {
    if (!*working) {
      return nullptr;
    }
    return make_response_with_one_cell();
  };
}

std::optional<int64_t> int_value(const DiagnosticsReport & report, const std::string & key)
{
  for (const auto & key_value : report.key_values) {
    if (key_value.key == key) {
      return std::get<int64_t>(key_value.value);
    }
  }
  return std::nullopt;
}

}  // namespace

class MapUpdateModuleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ndt_params_.trans_epsilon = 0.01;
    ndt_params_.step_size = 0.1;
    ndt_params_.resolution = 2.0F;
    ndt_params_.max_iterations = 30;
    ndt_params_.num_threads = 1;
    ndt_params_.regularization_scale_factor = 0.0F;

    param_.update_distance = 5.0;
    param_.map_radius = 30.0;
    param_.lidar_radius = 5.0;
    param_.publish_loaded_map = false;
  }

  pclomp::NdtParams ndt_params_{};
  MapUpdateModule::Params param_{};
};

// The first update has no previous position, so it loads from scratch and hands back an NDT that
// holds the map. Nothing the caller owns is touched: the map arrives as a return value.
TEST_F(MapUpdateModuleTest, FirstUpdateLoadsAMapAndHandsItBack)  // NOLINT
{
  MapUpdateModule module(param_, ndt_params_, make_loader_returning_cell());

  const auto result = module.update(make_point(0.0, 0.0));

  ASSERT_NE(result.ndt, nullptr);
  EXPECT_TRUE(result.ndt->hasTarget());
}

// out_of_map_range is true before any update, false at the last update position, and true again
// once the query moves beyond (map_radius - lidar_radius) from it.
TEST_F(MapUpdateModuleTest, OutOfMapRangeTracksLastUpdate)  // NOLINT
{
  MapUpdateModule module(param_, ndt_params_, make_loader_returning_cell());

  EXPECT_TRUE(module.out_of_map_range(make_point(0.0, 0.0)));  // nothing loaded yet

  ASSERT_NE(module.update(make_point(0.0, 0.0)).ndt, nullptr);

  EXPECT_FALSE(module.out_of_map_range(make_point(0.0, 0.0)));   // at the last update position
  EXPECT_TRUE(module.out_of_map_range(make_point(100.0, 0.0)));  // 100 + 5 > 30
}

// needs_update answers the periodic caller's question: it is true before anything is loaded, and
// then only once the vehicle has travelled update_distance.
TEST_F(MapUpdateModuleTest, NeedsUpdateWaitsForTheVehicleToMove)  // NOLINT
{
  MapUpdateModule module(param_, ndt_params_, make_loader_returning_cell());

  EXPECT_TRUE(module.needs_update(make_point(0.0, 0.0)));  // nothing loaded yet

  ASSERT_NE(module.update(make_point(0.0, 0.0)).ndt, nullptr);

  EXPECT_FALSE(module.needs_update(make_point(1.0, 0.0)));  // 1 m < update_distance (5 m)
  EXPECT_TRUE(module.needs_update(make_point(6.0, 0.0)));   // 6 m > 5 m
}

// A failing loader yields no map and an ERROR diagnostic mentioning the loader failure.
TEST_F(MapUpdateModuleTest, LoaderFailureReportsErrorAndHandsBackNothing)  // NOLINT
{
  MapUpdateModule module(param_, ndt_params_, make_failing_loader());

  const auto result = module.update(make_point(0.0, 0.0));

  EXPECT_EQ(result.ndt, nullptr);
  EXPECT_EQ(result.diagnostics.level, DiagnosticLevel::ERROR);
  EXPECT_NE(
    result.diagnostics.message.find("pcd_loader service is not working."), std::string::npos);
}

// A rebuild that fails leaves the map already loaded intact.
//
// The module loads into a fresh NDT and adopts it only once the load has succeeded, so a transient
// loader failure cannot leave it with nothing. `maps_size_before` on the next successful load is
// the witness: it counts the cells the cached map still holds, and would read zero had the failed
// rebuild cleared it.
//
// The other half is that the failure leaves last_update_position_ alone. Without that, the query
// below would be measured from (100, 0) and would itself be treated as out of range.
TEST_F(MapUpdateModuleTest, AFailedRebuildKeepsTheMapAlreadyLoaded)  // NOLINT
{
  auto working = std::make_shared<bool>(true);
  MapUpdateModule module(param_, ndt_params_, make_switchable_loader(working));

  ASSERT_NE(module.update(make_point(0.0, 0.0)).ndt, nullptr);

  // Far enough out of range to force a rebuild rather than a differential load, and the loader is
  // now unreachable.
  *working = false;
  ASSERT_TRUE(module.out_of_map_range(make_point(100.0, 0.0)));
  const auto failed = module.update(make_point(100.0, 0.0));
  EXPECT_EQ(failed.ndt, nullptr);
  EXPECT_EQ(failed.diagnostics.level, DiagnosticLevel::ERROR);

  // The failure did not move the reference position, so this is still inside the loaded map.
  EXPECT_FALSE(module.out_of_map_range(make_point(0.0, 0.0)));

  *working = true;
  const auto recovered = module.update(make_point(0.0, 0.0));
  EXPECT_EQ(int_value(recovered.diagnostics, "maps_size_before"), 1)
    << "the failed rebuild discarded the cached map";
}

// With publish_loaded_map enabled, a successful update returns the merged debug cloud in the "map"
// frame, stamped from the loader's response rather than from any clock this module does not have.
TEST_F(MapUpdateModuleTest, PublishesStampedLoadedMapWhenEnabled)  // NOLINT
{
  param_.publish_loaded_map = true;
  MapUpdateModule module(param_, ndt_params_, make_loader_returning_cell());

  const auto result = module.update(make_point(0.0, 0.0));

  ASSERT_NE(result.ndt, nullptr);
  ASSERT_TRUE(result.loaded_pcd_map.has_value());
  EXPECT_EQ(result.loaded_pcd_map->header.frame_id, "map");
  EXPECT_EQ(result.loaded_pcd_map->header.stamp.sec, response_stamp_sec);
  EXPECT_GT(
    static_cast<std::size_t>(result.loaded_pcd_map->width) * result.loaded_pcd_map->height, 0U);
  EXPECT_EQ(int_value(result.diagnostics, "loaded_pcd_map_skipped_cells"), 0);
}

// Every loaded cell ends up in the one merged debug cloud, and none is skipped.
TEST_F(MapUpdateModuleTest, MergesAllLoadedCellsIntoOneCloud)  // NOLINT
{
  constexpr std::size_t num_cells = 3;
  param_.publish_loaded_map = true;
  MapUpdateModule module(param_, ndt_params_, make_loader_returning_cells(num_cells));

  const auto result = module.update(make_point(0.0, 0.0));

  ASSERT_NE(result.ndt, nullptr);
  ASSERT_TRUE(result.loaded_pcd_map.has_value());

  // The per-cell totals to compare against: every cell holds one sample cubic pcd.
  sensor_msgs::msg::PointCloud2 one_cell;
  pcl::toROSMsg(make_sample_half_cubic_pcd(), one_cell);

  EXPECT_EQ(result.loaded_pcd_map->height, 1U);  // merging makes the cloud unorganized
  EXPECT_EQ(
    static_cast<std::size_t>(result.loaded_pcd_map->width),
    num_cells * static_cast<std::size_t>(one_cell.width) * one_cell.height);
  EXPECT_EQ(result.loaded_pcd_map->data.size(), num_cells * one_cell.data.size());
  EXPECT_EQ(int_value(result.diagnostics, "loaded_pcd_map_skipped_cells"), 0);
  EXPECT_EQ(result.diagnostics.level, DiagnosticLevel::OK);
}

// Without publish_loaded_map, no debug cloud is produced even on a successful update.
TEST_F(MapUpdateModuleTest, DoesNotProduceLoadedMapWhenDisabled)  // NOLINT
{
  param_.publish_loaded_map = false;
  MapUpdateModule module(param_, ndt_params_, make_loader_returning_cell());

  const auto result = module.update(make_point(0.0, 0.0));

  ASSERT_NE(result.ndt, nullptr);
  EXPECT_FALSE(result.loaded_pcd_map.has_value());
}

// DiagnosticsReport accumulates like DiagnosticsInterface: the level only rises and non-OK
// messages are joined with "; ". This is pure logic and needs no module instance.
TEST(MapUpdateModuleDiagnosticsReport, RaisesLevelAndJoinsMessages)  // NOLINT
{
  DiagnosticsReport report;
  EXPECT_EQ(report.level, DiagnosticLevel::OK);
  EXPECT_TRUE(report.message.empty());

  // OK-level updates neither raise the level nor append a message.
  report.update_level_and_message(DiagnosticLevel::OK, "ignored");
  EXPECT_EQ(report.level, DiagnosticLevel::OK);
  EXPECT_TRUE(report.message.empty());

  report.update_level_and_message(DiagnosticLevel::WARN, "first");
  report.update_level_and_message(DiagnosticLevel::ERROR, "second");
  // A later, lower level must not lower the accumulated level.
  report.update_level_and_message(DiagnosticLevel::WARN, "third");

  EXPECT_EQ(report.level, DiagnosticLevel::ERROR);
  EXPECT_EQ(report.message, "first; second; third");
}

}  // namespace autoware::ndt_scan_matcher
