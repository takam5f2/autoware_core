// Copyright 2026 Autoware Foundation
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

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

using GetDifferentialPointCloudMap = MapUpdateModule::GetDifferentialPointCloudMap;
using NdtType = MapUpdateModule::NdtType;
using NdtPtrType = MapUpdateModule::NdtPtrType;

// The fake map is a grid of cells, each holding the same small cube of points.
constexpr double cell_spacing = 100.0;
constexpr int cell_count_per_axis = 7;  // cells at 0, 100, ... 600 on both axes
constexpr size_t points_per_cell = 1000;

geometry_msgs::msg::Point make_point(const double x, const double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

// A 4 m cube of points, dense enough for every NDT voxel to reach min_points_per_voxel.
sensor_msgs::msg::PointCloud2 make_cell_cloud(const double center_x, const double center_y)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      for (int k = 0; k < 10; ++k) {
        cloud.push_back(
          pcl::PointXYZ(
            static_cast<float>(center_x + i * 0.4), static_cast<float>(center_y + j * 0.4),
            static_cast<float>(k * 0.4)));
      }
    }
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  return msg;
}

// Stands in for pointcloud_map_loader. Serves the differential map honestly, and can be told to
// pretend the service is unreachable.
class FakePcdLoader
{
public:
  bool available{true};
  int call_count{0};

  GetDifferentialPointCloudMap::Response::SharedPtr operator()(
    const GetDifferentialPointCloudMap::Request::SharedPtr & request)
  {
    ++call_count;
    if (!available) {
      return nullptr;
    }

    auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();
    response->header.frame_id = "map";

    const std::set<std::string> wanted = cells_within(*request);
    const std::set<std::string> cached(request->cached_ids.begin(), request->cached_ids.end());

    for (const auto & id : wanted) {
      if (cached.count(id) == 0) {
        autoware_map_msgs::msg::PointCloudMapCellWithID cell;
        cell.cell_id = id;
        cell.pointcloud = make_cell_cloud(center_of(id).first, center_of(id).second);
        response->new_pointcloud_with_ids.push_back(cell);
      }
    }
    for (const auto & id : cached) {
      if (wanted.count(id) == 0) {
        response->ids_to_remove.push_back(id);
      }
    }
    return response;
  }

  // The cell ids the loader would serve for a request centred at (x, y).
  static std::set<std::string> cells_within(const GetDifferentialPointCloudMap::Request & request)
  {
    std::set<std::string> ids;
    for (int i = 0; i < cell_count_per_axis; ++i) {
      for (int j = 0; j < cell_count_per_axis; ++j) {
        const double x = i * cell_spacing;
        const double y = j * cell_spacing;
        if (
          std::hypot(x - request.area.center_x, y - request.area.center_y) <= request.area.radius) {
          ids.insert(std::to_string(i) + "_" + std::to_string(j));
        }
      }
    }
    return ids;
  }

private:
  static std::pair<double, double> center_of(const std::string & id)
  {
    const size_t separator = id.find('_');
    return {
      std::stod(id.substr(0, separator)) * cell_spacing,
      std::stod(id.substr(separator + 1)) * cell_spacing};
  }
};

class MapUpdateModuleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    param_.update_distance = 20.0;
    param_.map_radius = 150.0;
    param_.lidar_radius = 100.0;
    param_.publish_loaded_map = false;
  }

  // Built lazily so that each test can adjust param_ in its own body first.
  MapUpdateModule & module()
  {
    if (!module_) {
      module_ = std::make_unique<MapUpdateModule>(
        ndt_ptr_, param_, [this](const GetDifferentialPointCloudMap::Request::SharedPtr & request) {
          return loader_(request);
        });
    }
    return *module_;
  }

  [[nodiscard]] std::vector<std::string> loaded_cell_ids()
  {
    return ndt_ptr_.with([](const auto & ndt_ptr) { return ndt_ptr->getCurrentMapIDs(); });
  }

  HyperParameters::DynamicMapLoading param_{};
  FakePcdLoader loader_;
  Guarded<NdtPtrType> ndt_ptr_{std::make_shared<NdtType>()};
  std::unique_ptr<MapUpdateModule> module_;
};

// The very first update has no reference position, so it must load the map.
TEST_F(MapUpdateModuleTest, first_update_loads_the_map)
{
  const auto result = module().callback_timer(true, make_point(300.0, 300.0));

  EXPECT_TRUE(result.map_updated);
  EXPECT_EQ(result.diagnostics.level, MapUpdateModule::DiagnosticLevel::OK);
  EXPECT_FALSE(loaded_cell_ids().empty());
}

TEST_F(MapUpdateModuleTest, update_is_skipped_until_the_vehicle_has_moved_far_enough)
{
  ASSERT_TRUE(module().callback_timer(true, make_point(300.0, 300.0)).map_updated);
  const int calls_after_first_update = loader_.call_count;

  // Below param_.update_distance, so the loader must not even be asked.
  const auto result = module().callback_timer(true, make_point(310.0, 300.0));

  EXPECT_FALSE(result.map_updated);
  EXPECT_EQ(loader_.call_count, calls_after_first_update);
}

// A failing pcd_loader must not cost us the map we already have. Before, the rebuild path reset
// ndt_ptr_ before attempting the load, so a transient failure left NDT with no map at all.
TEST_F(MapUpdateModuleTest, rebuild_failure_keeps_the_previous_map)
{
  ASSERT_TRUE(module().callback_timer(true, make_point(300.0, 300.0)).map_updated);
  const std::vector<std::string> cells_before = loaded_cell_ids();
  ASSERT_FALSE(cells_before.empty());

  // Move beyond map_radius - lidar_radius so that a full rebuild is required, and fail the load.
  loader_.available = false;
  const auto result = module().callback_timer(true, make_point(300.0, 360.0));

  EXPECT_FALSE(result.map_updated);
  EXPECT_EQ(result.diagnostics.level, MapUpdateModule::DiagnosticLevel::ERROR);
  EXPECT_EQ(loaded_cell_ids(), cells_before);
}

// The reference position must not advance on a failed load, otherwise the measured distance is
// reset to zero and a standing vehicle never retries.
TEST_F(MapUpdateModuleTest, rebuild_failure_is_retried_without_moving)
{
  ASSERT_TRUE(module().callback_timer(true, make_point(300.0, 300.0)).map_updated);

  loader_.available = false;
  const auto far_position = make_point(300.0, 360.0);
  ASSERT_FALSE(module().callback_timer(true, far_position).map_updated);

  // Same position as the failed attempt: the vehicle has not moved at all.
  loader_.available = true;
  const auto result = module().callback_timer(true, far_position);

  EXPECT_TRUE(result.map_updated);
  EXPECT_FALSE(loaded_cell_ids().empty());
}

// "Nothing to load here" is not a failure: the reference position advances so that the next
// callback does not mistake the accumulated distance for the map falling behind.
TEST_F(MapUpdateModuleTest, no_change_advances_the_reference_position)
{
  ASSERT_TRUE(module().callback_timer(true, make_point(300.0, 300.0)).map_updated);

  // Far outside the fake map, so the loader has nothing to offer.
  const auto empty_area_position = make_point(5000.0, 5000.0);
  const auto result = module().callback_timer(true, empty_area_position);

  EXPECT_FALSE(result.map_updated);
  EXPECT_FALSE(module().out_of_map_range(empty_area_position));
}

TEST_F(MapUpdateModuleTest, out_of_map_range_tracks_the_loaded_map)
{
  const auto position = make_point(300.0, 300.0);

  EXPECT_TRUE(module().out_of_map_range(position));

  ASSERT_TRUE(module().callback_timer(true, position).map_updated);
  EXPECT_FALSE(module().out_of_map_range(position));

  // map_radius - lidar_radius is 50 m with these parameters.
  EXPECT_FALSE(module().out_of_map_range(make_point(300.0, 340.0)));
  EXPECT_TRUE(module().out_of_map_range(make_point(300.0, 360.0)));
}

// The debug publish used to be a single cloud that only ever grew, so cells the NDT had dropped
// stayed in it forever. It has to shrink together with the NDT's own cell set.
TEST_F(MapUpdateModuleTest, loaded_map_publish_drops_removed_cells)
{
  param_.publish_loaded_map = true;

  const auto first = module().callback_timer(true, make_point(300.0, 300.0));
  ASSERT_TRUE(first.map_updated);
  ASSERT_TRUE(first.loaded_pcd_map.has_value());
  ASSERT_EQ(first.loaded_pcd_map->width, loaded_cell_ids().size() * points_per_cell);

  // Moving here keeps the update incremental (25 m > update_distance, < 50 m) while pushing two
  // cells out of map_radius, so the loaded map must lose them.
  const auto second = module().callback_timer(true, make_point(300.0, 325.0));

  ASSERT_TRUE(second.map_updated);
  ASSERT_TRUE(second.loaded_pcd_map.has_value());
  EXPECT_LT(second.loaded_pcd_map->width, first.loaded_pcd_map->width);
  EXPECT_EQ(second.loaded_pcd_map->width, loaded_cell_ids().size() * points_per_cell);
}

}  // namespace
}  // namespace autoware::ndt_scan_matcher

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
