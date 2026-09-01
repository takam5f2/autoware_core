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

// NdtScanMatcher driven directly: no node, no executor, no ROS graph.
//
// The characterization suite pins the same behaviour from outside a running node, which is what
// made the extraction safe but is not where these pins belong: a case there costs a node launch, a
// service discovery and several seconds, and it can only observe what a topic carries. These run
// in milliseconds against the returned structs, and reach things a topic cannot show -- that a
// scan rejected at the activation gate is still the one the align service will use, for instance.
//
// This file is where the characterization pins move to as the core becomes reachable without a
// node. It is not yet the whole of them; see the notes on individual cases.

#include "test_util.hpp"

#include <autoware/ndt_scan_matcher/ndt_scan_matcher.hpp>

#include <autoware_map_msgs/msg/point_cloud_map_cell_with_id.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{
using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

// Where the stub loader puts its one map cell. Far from the origin so that a query at (0, 0) is
// unambiguously outside it.
constexpr double map_center_x = 100.0;
constexpr double map_center_y = 100.0;

int8_t level_of(const DiagnosticsReport & report)
{
  return static_cast<int8_t>(report.level);
}

// The keys a report carries, in the order they were recorded.
std::vector<std::string> keys_in_order(const DiagnosticsReport & report)
{
  std::vector<std::string> keys;
  keys.reserve(report.key_values.size());
  for (const auto & key_value : report.key_values) {
    keys.push_back(key_value.key);
  }
  return keys;
}

bool has_key(const DiagnosticsReport & report, const std::string & key)
{
  const auto keys = keys_in_order(report);
  return std::find(keys.begin(), keys.end(), key) != keys.end();
}

bool has_log_site(const DiagnosticsReport & report, const LogSite site)
{
  return std::any_of(report.logs.begin(), report.logs.end(), [site](const LogRequest & log) {
    return log.site == site;
  });
}

builtin_interfaces::msg::Time make_time(const int32_t sec, const uint32_t nanosec = 0)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

// The sample cloud as a message, centred on the vehicle so that its farthest point clears the
// required distance.
sensor_msgs::msg::PointCloud2 make_scan_msg(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(make_sample_half_cubic_pcd(), msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  return msg;
}

// A loader that answers with the sample cloud placed at the map centre, and with nothing when the
// requested area does not reach it -- the same area test the ROS stub service applies, which is
// what makes a request far from the centre a failed load rather than a successful one.
MapUpdateModule::PcdLoaderFunction make_loader_returning_cell()
{
  return [](const GetDifferentialPointCloudMap::Request::SharedPtr & request)
           -> GetDifferentialPointCloudMap::Response::SharedPtr {
    auto response = std::make_shared<GetDifferentialPointCloudMap::Response>();
    response->header.frame_id = "map";
    if (
      request->area.center_x - request->area.radius > map_center_x ||
      request->area.center_x + request->area.radius < map_center_x ||
      request->area.center_y - request->area.radius > map_center_y ||
      request->area.center_y + request->area.radius < map_center_y) {
      return response;
    }

    autoware_map_msgs::msg::PointCloudMapCellWithID cell;
    cell.cell_id = "0";
    pcl::PointCloud<pcl::PointXYZ> cloud = make_sample_half_cubic_pcd();
    for (auto & point : cloud.points) {
      point.x += static_cast<float>(map_center_x);
      point.y += static_cast<float>(map_center_y);
    }
    pcl::toROSMsg(cloud, cell.pointcloud);
    response->new_pointcloud_with_ids.push_back(cell);
    return response;
  };
}

}  // namespace

class NdtScanMatcherCoreTest : public ::testing::Test
{
protected:
  // The shipped configuration, with num_threads pinned for determinism.
  static NdtScanMatcher::Params shipped_params()
  {
    NdtScanMatcher::Params param;

    param.ndt.trans_epsilon = 0.01;
    param.ndt.step_size = 0.1;
    param.ndt.resolution = 2.0F;
    param.ndt.max_iterations = 30;
    param.ndt.num_threads = 1;
    param.ndt.regularization_scale_factor = 0.01F;

    param.skipping_publish_num = 5;

    param.map_update.update_distance = 20.0;
    param.map_update.map_radius = 150.0;
    param.map_update.lidar_radius = 100.0;
    param.map_update.publish_loaded_map = false;

    param.pose_initialization.particles_num = 5;
    param.pose_initialization.n_startup_trials = 3;
    param.pose_initialization.map_frame = "map";
    param.pose_initialization.converged_param_nearest_voxel_transformation_likelihood = 2.3;

    auto & scan_matching = param.scan_matching;
    scan_matching.frame.base_frame = "base_link";
    scan_matching.frame.ndt_base_frame = "ndt_base_link";
    scan_matching.frame.map_frame = "map";
    scan_matching.sensor_points.timeout_sec = 1.0;
    scan_matching.sensor_points.required_distance = 10.0;
    scan_matching.ndt_regularization_enable = false;
    scan_matching.validation.initial_pose_timeout_sec = 1.0;
    scan_matching.validation.initial_pose_distance_tolerance_m = 10.0;
    scan_matching.validation.initial_to_result_distance_tolerance_m = 3.0;
    scan_matching.validation.critical_upper_bound_exe_time_ms = 100.0;
    scan_matching.score_estimation.converged_param_type =
      ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD;
    scan_matching.score_estimation.converged_param_transform_probability = 3.0;
    scan_matching.score_estimation.converged_param_nearest_voxel_transformation_likelihood = 2.3;
    scan_matching.score_estimation.publish_voxel_score_points = false;
    scan_matching.score_estimation.no_ground_points.enable = false;
    scan_matching.score_estimation.no_ground_points.z_margin_for_ground_removal = 0.8;
    scan_matching.covariance.output_pose_covariance = {
      0.0225, 0.0,    0.0,    0.0,      0.0,      0.0,  //
      0.0,    0.0225, 0.0,    0.0,      0.0,      0.0,  //
      0.0,    0.0,    0.0225, 0.0,      0.0,      0.0,  //
      0.0,    0.0,    0.0,    0.000625, 0.0,      0.0,  //
      0.0,    0.0,    0.0,    0.0,      0.000625, 0.0,  //
      0.0,    0.0,    0.0,    0.0,      0.0,      0.000625};
    scan_matching.covariance.covariance_estimation.covariance_estimation_type =
      CovarianceEstimationType::FIXED_VALUE;
    scan_matching.covariance.covariance_estimation.initial_pose_offset_model_x = {0.0,  0.0, 0.5,
                                                                                  -0.5, 1.0, -1.0};
    scan_matching.covariance.covariance_estimation.initial_pose_offset_model_y = {0.5, -0.5, 0.0,
                                                                                  0.0, 0.0,  0.0};
    scan_matching.covariance.covariance_estimation.temperature = 0.05;
    scan_matching.covariance.covariance_estimation.scale_factor = 1.0;
    return param;
  }

  void SetUp() override
  {
    matcher_ = std::make_unique<NdtScanMatcher>(shipped_params(), make_loader_returning_cell());
  }

  // A scan the hot path will accept as far as the activation gate: non-empty, undelayed, and with
  // the sensor frame equal to the base frame so that no transform is applied. Whether the gate
  // lets it through is the matcher's own state, set with set_activated().
  static NdtScanMatcher::ScanInput make_scan_input(const int32_t sec = 10)
  {
    NdtScanMatcher::ScanInput input;
    input.scan =
      std::make_shared<sensor_msgs::msg::PointCloud2>(make_scan_msg(make_time(sec), "base_link"));
    input.now = make_time(sec);
    input.base_from_sensor.transform = geometry_msgs::msg::TransformStamped();
    return input;
  }

  std::unique_ptr<NdtScanMatcher> matcher_;
};

// An empty scan is rejected on its size, before anything that needs a transform or a map.
TEST_F(NdtScanMatcherCoreTest, EmptyScanIsRejectedOnItsSize)  // NOLINT
{
  sensor_msgs::msg::PointCloud2 empty;
  empty.header.stamp = make_time(10);
  empty.header.frame_id = "base_link";

  matcher_->set_activated(true);
  auto input = make_scan_input();
  input.scan = std::make_shared<sensor_msgs::msg::PointCloud2>(empty);

  const auto result = matcher_->match_scan(input);

  EXPECT_FALSE(result.output.has_value());
  EXPECT_EQ(result.diagnostics.message, "Sensor points is empty.");
  EXPECT_EQ(level_of(result.diagnostics), 1);  // WARN
  // The size gate returns before the transform is looked at, so that key is never recorded. The
  // skip count is appended by the matcher itself, whatever the gate.
  EXPECT_EQ(
    keys_in_order(result.diagnostics),
    (std::vector<std::string>{"topic_time_stamp", "sensor_points_size", "skipping_publish_num"}));
}

// A scan whose transform failed to look up is an error, and records the log line the node used to
// emit inline.
TEST_F(NdtScanMatcherCoreTest, ScanWithoutATransformIsAnErrorAndRecordsItsLogSite)  // NOLINT
{
  matcher_->set_activated(true);
  auto input = make_scan_input();
  input.scan =
    std::make_shared<sensor_msgs::msg::PointCloud2>(make_scan_msg(make_time(10), "lidar_top"));
  input.base_from_sensor.transform = std::nullopt;
  input.base_from_sensor.error = "Could not find a connection";

  const auto result = matcher_->match_scan(input);

  EXPECT_FALSE(result.output.has_value());
  EXPECT_EQ(level_of(result.diagnostics), 2);  // ERROR
  EXPECT_EQ(
    result.diagnostics.message,
    "Could not find a connection. Please publish TF lidar_top to base_link");
  EXPECT_TRUE(has_log_site(result.diagnostics, LogSite::ScanTransformFailed));
}

// A scan received while deactivated is still the scan the align service works from.
//
// The pin the characterization suite calls `SensorPointsAreStoredEvenWhileDeactivated`, which can
// only reach it through a later align. Here the two calls are adjacent, and what connects them --
// the matcher keeping the scan across the activation gate -- is the thing under test.
TEST_F(NdtScanMatcherCoreTest, AScanKeptWhileDeactivatedIsTheOneAlignSearchesFrom)  // NOLINT
{
  const auto before = matcher_->align({make_pose(map_center_x, map_center_y), make_time(10)});
  ASSERT_FALSE(before.estimate.has_value());
  ASSERT_FALSE(before.diagnostics.key_values.empty());
  EXPECT_EQ(keys_in_order(before.diagnostics).back(), "is_set_sensor_points")
    << "with no scan yet, the search should stop on the sensor-points check";

  const auto scan = matcher_->match_scan(make_scan_input());
  ASSERT_FALSE(scan.output.has_value()) << "the activation gate should have stopped the match";

  const auto after = matcher_->align({make_pose(map_center_x, map_center_y), make_time(10)});
  EXPECT_TRUE(has_key(after.diagnostics, "is_set_sensor_points"));
  EXPECT_TRUE(after.estimate.has_value())
    << "align found no scan to work from. message: " << after.diagnostics.message;
}

// Aligning where no map can be loaded fails, reporting the map update's findings ahead of the
// search's and joining their messages in that order.
//
// The core half of `AligningOutsideMapRangeFailsWithThreeJoinedMessages`; the node adds the third
// message, so only two are joined here.
TEST_F(NdtScanMatcherCoreTest, AligningOutsideMapRangeJoinsTheMapAndSearchMessages)  // NOLINT
{
  const auto result = matcher_->align({make_pose(-map_center_x, -map_center_y), make_time(10)});

  EXPECT_FALSE(result.estimate.has_value());
  EXPECT_FALSE(result.best_points_aligned.has_value());
  EXPECT_FALSE(result.search_markers.has_value());
  EXPECT_EQ(level_of(result.diagnostics), 2);  // ERROR, raised by the failed update
  EXPECT_EQ(
    result.diagnostics.message,
    "update_ndt failed. If this happens with initial position estimation, make sure that(1) the "
    "initial position matches the pcd map and (2) the map_loader is working properly.; "
    "No InputTarget. Please check the map file and the map_loader service");
  // The map check short-circuits the sensor-points check, as it does through the node.
  EXPECT_FALSE(has_key(result.diagnostics, "is_set_sensor_points"));
}

// The map update timer reports that it has no reference position, and asks the loader for nothing,
// until an initial pose has arrived.
TEST_F(NdtScanMatcherCoreTest, MapUpdateWaitsForAReferencePosition)  // NOLINT
{
  const auto deactivated = matcher_->update_map_periodically();
  EXPECT_FALSE(deactivated.map_updated);
  EXPECT_EQ(level_of(deactivated.diagnostics), 1);  // WARN
  EXPECT_EQ(keys_in_order(deactivated.diagnostics), (std::vector<std::string>{"is_activated"}))
    << "the activation gate should stop the update before it looks for a position";

  matcher_->set_activated(true);
  const auto before = matcher_->update_map_periodically();

  EXPECT_FALSE(before.map_updated);
  EXPECT_EQ(level_of(before.diagnostics), 1);  // WARN
  EXPECT_EQ(
    keys_in_order(before.diagnostics),
    (std::vector<std::string>{"is_activated", "is_set_last_update_position"}));

  auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(
    make_pose(map_center_x, map_center_y));
  pose->header.stamp = make_time(10);
  const auto push_report = matcher_->push_initial_pose(pose);
  ASSERT_EQ(level_of(push_report), 0) << "the pose was rejected: " << push_report.message;

  const auto after = matcher_->update_map_periodically();

  EXPECT_TRUE(after.map_updated);
  // The two gate keys stay ahead of the map module's own findings.
  EXPECT_EQ(keys_in_order(after.diagnostics).front(), "is_activated");
  EXPECT_TRUE(has_key(after.diagnostics, "is_updated_map"));
}

// The skip counter counts scans that produced nothing, warns once it reaches the limit, and is
// reset by deactivation rather than by success alone.
TEST_F(NdtScanMatcherCoreTest, SkipCounterWarnsAtTheLimitAndResetsWhenDeactivated)  // NOLINT
{
  const auto value_of_skip_count = [](const DiagnosticsReport & report) {
    for (const auto & key_value : report.key_values) {
      if (key_value.key == "skipping_publish_num") {
        return std::get<int64_t>(key_value.value);
      }
    }
    return static_cast<int64_t>(-1);
  };

  // Nothing is set up for a match to succeed, so every activated scan below is a skip.
  matcher_->set_activated(true);
  for (int64_t expected = 1; expected <= 4; expected++) {
    const auto result = matcher_->match_scan(make_scan_input());
    EXPECT_EQ(value_of_skip_count(result.diagnostics), expected);
    EXPECT_EQ(level_of(result.diagnostics), 1) << "WARN from the gate, not yet from the counter";
  }

  const auto at_limit = matcher_->match_scan(make_scan_input());
  EXPECT_EQ(value_of_skip_count(at_limit.diagnostics), 5);
  EXPECT_NE(
    at_limit.diagnostics.message.find("skipping_publish_num exceed limit (5 times)."),
    std::string::npos)
    << "message was: " << at_limit.diagnostics.message;

  matcher_->set_activated(false);
  const auto deactivated = matcher_->match_scan(make_scan_input());
  EXPECT_EQ(value_of_skip_count(deactivated.diagnostics), 0);
}

}  // namespace autoware::ndt_scan_matcher
