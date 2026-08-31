// Copyright 2023- Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__POSE_INTERPOLATION_BUFFER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__POSE_INTERPOLATION_BUFFER_HPP_

#include "diagnostics_report.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace autoware::ndt_scan_matcher
{

// `builtin_interfaces::msg::Time` as nanoseconds, which is how `rclcpp::Time` holds and compares
// it. Every comparison and difference in this file goes through here so that the arithmetic stays
// integral, as it was.
[[nodiscard]] inline int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

// `rclcpp::Duration::seconds()` converts by *dividing* by 1e9 (libstdc++ ratio conversion), so
// this does too. Multiplying by 1e-9 instead moves the last ulp.
[[nodiscard]] inline double to_seconds(const int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1e9;
}

// A time-ordered buffer of poses that answers "where were we at this stamp?".
//
// Vendored from `autoware::localization_util::SmartPoseBuffer` so that the core layer can hold it:
// that class takes an `rclcpp::Logger` in its constructor and `rclcpp::Time` in its methods, which
// would put rclcpp back into the core. Behaviour is unchanged -- the rejection rules, their order,
// and the fact that the covariance is not interpolated are all as they were. The four log lines it
// emitted are recorded as `LogRequest`s for the node to emit instead of being dropped.
class PoseInterpolationBuffer
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  struct InterpolateResult
  {
    PoseWithCovarianceStamped old_pose;
    PoseWithCovarianceStamped new_pose;
    PoseWithCovarianceStamped interpolated_pose;
  };

  PoseInterpolationBuffer() = delete;
  PoseInterpolationBuffer(double pose_timeout_sec, double pose_distance_tolerance_meters);

  // Rejections are reported through `logs`; there is no diagnostics key here, as before.
  [[nodiscard]] std::optional<InterpolateResult> interpolate(
    const builtin_interfaces::msg::Time & target_time, std::vector<LogRequest> & logs);

  // A stamp older than the newest one already held clears the buffer, so that replaying a rosbag
  // twice does not leave it out of order.
  void push_back(const PoseWithCovarianceStamped::ConstSharedPtr & pose_msg_ptr);

  void pop_old(const builtin_interfaces::msg::Time & target_time);

  void clear();

private:
  [[nodiscard]] bool validate_time_stamp_difference(
    const builtin_interfaces::msg::Time & target_time,
    const builtin_interfaces::msg::Time & reference_time, double time_tolerance_sec,
    std::vector<LogRequest> & logs) const;
  [[nodiscard]] bool validate_position_difference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, double distance_tolerance_m,
    std::vector<LogRequest> & logs) const;

  std::deque<PoseWithCovarianceStamped::ConstSharedPtr> pose_buffer_;
  mutable std::mutex mutex_;  // guards pose_buffer_

  const double pose_timeout_sec_;
  const double pose_distance_tolerance_meters_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__POSE_INTERPOLATION_BUFFER_HPP_
