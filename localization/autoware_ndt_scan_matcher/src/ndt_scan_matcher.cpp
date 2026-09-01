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

#include <autoware/ndt_scan_matcher/ndt_scan_matcher.hpp>

#include <memory>
#include <sstream>
#include <utility>

namespace autoware::ndt_scan_matcher
{
namespace
{

// The NDT the module hands to MapUpdateModule, with its parameters already applied: the module
// copies the map on the way in, so it has to be set up before that.
NdtScanMatcher::NdtPtrType make_ndt(const pclomp::NdtParams & params)
{
  auto ndt = std::make_shared<NdtScanMatcher::NdtType>();
  ndt->setParams(params);
  return ndt;
}

}  // namespace

NdtScanMatcher::NdtScanMatcher(Params param, MapUpdateModule::PcdLoaderFunction pcd_loader)
: param_(std::move(param)),
  ndt_ptr_(make_ndt(param_.ndt)),
  map_update_(ndt_ptr_, param_.map_update, std::move(pcd_loader)),
  scan_matching_(param_.scan_matching, activated_)
{
}

NdtScanMatcher::ScanResult NdtScanMatcher::match_scan(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan,
  const builtin_interfaces::msg::Time & now, const TransformLookup & base_from_sensor)
{
  ScanMatchingModule::Result match;
  // The lock is held across the match; everything the caller publishes is assembled inside it and
  // handed back for publishing outside.
  ndt_ptr_.with([&](const auto & ndt_ptr) {
    match = scan_matching_.scan_match(scan, now, base_from_sensor, *ndt_ptr, map_update_);
  });

  if (match.scan_in_baselink_frame) {
    // Taken after the NDT lock is released, so the two are never held together.
    scan_in_baselink_frame_.with([&](auto & scan) { scan = match.scan_in_baselink_frame; });
  }

  ScanResult result;
  result.output = std::move(match.output);
  result.diagnostics = std::move(match.diagnostics);

  // Not counted while deactivated: nothing was going to be published then anyway, and counting
  // would raise the warning below throughout every startup.
  skipping_publish_num_ = ((match.converged || !activated_) ? 0 : (skipping_publish_num_ + 1));
  result.diagnostics.add_key_value({"skipping_publish_num", skipping_publish_num_});
  if (skipping_publish_num_ >= param_.skipping_publish_num) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit (" << skipping_publish_num_ << " times).";
    result.diagnostics.update_level_and_message(DiagnosticLevel::WARN, message.str());
  }
  return result;
}

MapUpdateModule::UpdateResult NdtScanMatcher::update_map_periodically()
{
  MapUpdateModule::UpdateResult result;

  // check is_activated
  if (!result.diagnostics.check(
        "is_activated", activated_, DiagnosticLevel::WARN, "Node is not activated.")) {
    return result;
  }

  // check is_set_last_update_position
  const auto position = scan_matching_.latest_ekf_position();
  const bool is_set_last_update_position = (position != std::nullopt);
  result.diagnostics.add_key_value({"is_set_last_update_position", is_set_last_update_position});
  if (!is_set_last_update_position) {
    result.diagnostics.update_level_and_message(
      DiagnosticLevel::WARN,
      "Cannot find the reference position for map update."
      "Please check if the EKF odometry is provided to NDT.");
    return result;
  }

  auto update = map_update_.callback_timer(*position);
  result.map_updated = update.map_updated;
  result.loaded_pcd_map = std::move(update.loaded_pcd_map);
  result.diagnostics.append(std::move(update.diagnostics));
  return result;
}

NdtScanMatcher::AlignResult NdtScanMatcher::align(const AlignInput & input)
{
  AlignResult result;

  // Outside the NDT lock: the map update takes that lock itself, and must not be called with it
  // already held.
  auto update = map_update_.update_map(input.initial_pose_in_map_frame.pose.pose.position);
  result.loaded_pcd_map = std::move(update.loaded_pcd_map);
  result.diagnostics.append(std::move(update.diagnostics));

  // Copied before the NDT lock is taken, so the scan's lock stays a leaf.
  const auto scan = scan_in_baselink_frame_.with([](const auto & value) { return value; });

  PoseInitializationResult search;
  // The lock is held across the search: it aligns against the installed NDT.
  ndt_ptr_.with([&](auto & ndt_ptr) {
    search = search_initial_pose(
      param_.pose_initialization, *ndt_ptr, scan, input.initial_pose_in_map_frame, input.now);
  });

  result.diagnostics.append(std::move(search.diagnostics));
  result.estimate = std::move(search.estimate);
  result.best_points_aligned = std::move(search.best_points_aligned);
  result.search_markers = std::move(search.search_markers);
  return result;
}

DiagnosticsReport NdtScanMatcher::push_initial_pose(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose)
{
  return scan_matching_.push_initial_pose(pose);
}

DiagnosticsReport NdtScanMatcher::push_regularization_pose(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose)
{
  return scan_matching_.push_regularization_pose(pose);
}

void NdtScanMatcher::set_activated(const bool activated)
{
  activated_ = activated;
  if (activated) {
    scan_matching_.clear_initial_pose_buffer();
  }
}

}  // namespace autoware::ndt_scan_matcher
