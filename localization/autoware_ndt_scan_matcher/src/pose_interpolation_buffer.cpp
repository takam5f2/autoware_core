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

#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/pose_interpolation_buffer.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ndt_scan_matcher
{
namespace
{

using autoware::localization_util::calc_diff_for_radian;
using autoware::localization_util::get_rpy;
using autoware::localization_util::norm;

// Vendored from `autoware::localization_util::calc_twist`, which takes the two stamps as
// `rclcpp::Time`. Same arithmetic on the same values.
geometry_msgs::msg::Twist calc_twist(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b)
{
  const double dt_s =
    to_seconds(to_nanoseconds(pose_b.header.stamp) - to_nanoseconds(pose_a.header.stamp));

  if (dt_s == 0) {
    return geometry_msgs::msg::Twist();
  }

  const auto pose_a_rpy = get_rpy(pose_a);
  const auto pose_b_rpy = get_rpy(pose_b);

  geometry_msgs::msg::Vector3 diff_xyz;
  geometry_msgs::msg::Vector3 diff_rpy;

  diff_xyz.x = pose_b.pose.position.x - pose_a.pose.position.x;
  diff_xyz.y = pose_b.pose.position.y - pose_a.pose.position.y;
  diff_xyz.z = pose_b.pose.position.z - pose_a.pose.position.z;
  diff_rpy.x = calc_diff_for_radian(pose_b_rpy.x, pose_a_rpy.x);
  diff_rpy.y = calc_diff_for_radian(pose_b_rpy.y, pose_a_rpy.y);
  diff_rpy.z = calc_diff_for_radian(pose_b_rpy.z, pose_a_rpy.z);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = diff_xyz.x / dt_s;
  twist.linear.y = diff_xyz.y / dt_s;
  twist.linear.z = diff_xyz.z / dt_s;
  twist.angular.x = diff_rpy.x / dt_s;
  twist.angular.y = diff_rpy.y / dt_s;
  twist.angular.z = diff_rpy.z / dt_s;
  return twist;
}

// Vendored from `autoware::localization_util::interpolate_pose`. The "any of the three stamps is
// zero seconds -> default-constructed pose" guard is kept: two characterization cases reach it.
geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b,
  const builtin_interfaces::msg::Time & time_stamp)
{
  geometry_msgs::msg::PoseStamped tmp_pose_a;
  tmp_pose_a.header = pose_a.header;
  tmp_pose_a.pose = pose_a.pose.pose;

  geometry_msgs::msg::PoseStamped tmp_pose_b;
  tmp_pose_b.header = pose_b.header;
  tmp_pose_b.pose = pose_b.pose.pose;

  const int64_t pose_a_ns = to_nanoseconds(tmp_pose_a.header.stamp);
  const int64_t pose_b_ns = to_nanoseconds(tmp_pose_b.header.stamp);
  const int64_t target_ns = to_nanoseconds(time_stamp);

  if (
    (to_seconds(pose_a_ns) == 0.0) || (to_seconds(pose_b_ns) == 0.0) ||
    (to_seconds(target_ns) == 0.0)) {
    return geometry_msgs::msg::PoseStamped();
  }

  const auto twist = calc_twist(tmp_pose_a, tmp_pose_b);
  const double dt = to_seconds(target_ns - pose_a_ns);

  const auto pose_a_rpy = get_rpy(tmp_pose_a);

  geometry_msgs::msg::Vector3 xyz;
  geometry_msgs::msg::Vector3 rpy;
  xyz.x = tmp_pose_a.pose.position.x + twist.linear.x * dt;
  xyz.y = tmp_pose_a.pose.position.y + twist.linear.y * dt;
  xyz.z = tmp_pose_a.pose.position.z + twist.linear.z * dt;
  rpy.x = pose_a_rpy.x + twist.angular.x * dt;
  rpy.y = pose_a_rpy.y + twist.angular.y * dt;
  rpy.z = pose_a_rpy.z + twist.angular.z * dt;

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = tmp_pose_a.header;
  pose.header.stamp = time_stamp;
  pose.pose.position.x = xyz.x;
  pose.pose.position.y = xyz.y;
  pose.pose.position.z = xyz.z;
  pose.pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

// The two validation warnings were RCLCPP_WARN with printf formatting; going through snprintf
// reproduces their text byte for byte, which std::to_string or a stream would not.
template <typename... Args>
std::string format(const char * fmt, Args... args)
{
  std::array<char, 512> buffer{};
  std::snprintf(buffer.data(), buffer.size(), fmt, args...);
  return std::string(buffer.data());
}

}  // namespace

PoseInterpolationBuffer::PoseInterpolationBuffer(
  const double pose_timeout_sec, const double pose_distance_tolerance_meters)
: pose_timeout_sec_(pose_timeout_sec),
  pose_distance_tolerance_meters_(pose_distance_tolerance_meters)
{
}

std::optional<PoseInterpolationBuffer::InterpolateResult> PoseInterpolationBuffer::interpolate(
  const builtin_interfaces::msg::Time & target_time, std::vector<LogRequest> & logs)
{
  InterpolateResult result;
  const int64_t target_ns = to_nanoseconds(target_time);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pose_buffer_.size() < 2) {
      logs.push_back({LogSite::PoseBufferTooFewSamples, "pose_buffer_.size() < 2"});
      return std::nullopt;
    }

    if (target_ns < to_nanoseconds(pose_buffer_.front()->header.stamp)) {
      logs.push_back(
        {LogSite::PoseBufferStampMismatch,
         "Mismatch between pose timestamp and current timestamp"});
      return std::nullopt;
    }

    // A target newer than the newest buffered pose is acceptable here: the sensor stamp often runs
    // ahead of the EKF. Too far ahead is caught by validate_time_stamp_difference below.

    // get the nearest poses
    result.old_pose = *pose_buffer_.front();
    for (const PoseWithCovarianceStamped::ConstSharedPtr & pose_cov_msg_ptr : pose_buffer_) {
      result.new_pose = *pose_cov_msg_ptr;
      if (to_nanoseconds(result.new_pose.header.stamp) > target_ns) {
        break;
      }
      result.old_pose = *pose_cov_msg_ptr;
    }
  }

  // check the time stamp
  const bool is_old_pose_valid = validate_time_stamp_difference(
    result.old_pose.header.stamp, target_time, pose_timeout_sec_, logs);
  const bool is_new_pose_valid = validate_time_stamp_difference(
    result.new_pose.header.stamp, target_time, pose_timeout_sec_, logs);

  // check the position jumping (ex. immediately after the initial pose estimation)
  const bool is_pose_diff_valid = validate_position_difference(
    result.old_pose.pose.pose.position, result.new_pose.pose.pose.position,
    pose_distance_tolerance_meters_, logs);

  // all validations must be true
  if (!(is_old_pose_valid && is_new_pose_valid && is_pose_diff_valid)) {
    return std::nullopt;
  }

  const geometry_msgs::msg::PoseStamped interpolated_pose_msg =
    interpolate_pose(result.old_pose, result.new_pose, target_time);
  result.interpolated_pose.header = interpolated_pose_msg.header;
  result.interpolated_pose.pose.pose = interpolated_pose_msg.pose;
  // This does not interpolate the covariance.
  // interpolated_pose.pose.covariance is always set to old_pose.covariance
  result.interpolated_pose.pose.covariance = result.old_pose.pose.covariance;
  return result;
}

void PoseInterpolationBuffer::push_back(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose_msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pose_buffer_.empty()) {
    // Check for non-chronological timestamp order
    // This situation can arise when replaying a rosbag multiple times
    if (
      to_nanoseconds(pose_msg_ptr->header.stamp) <
      to_nanoseconds(pose_buffer_.back()->header.stamp)) {
      // Clear the buffer if timestamps are reversed to maintain chronological order
      pose_buffer_.clear();
    }
  }
  pose_buffer_.push_back(pose_msg_ptr);
}

void PoseInterpolationBuffer::pop_old(const builtin_interfaces::msg::Time & target_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t target_ns = to_nanoseconds(target_time);
  while (!pose_buffer_.empty()) {
    if (to_nanoseconds(pose_buffer_.front()->header.stamp) >= target_ns) {
      break;
    }
    pose_buffer_.pop_front();
  }
}

void PoseInterpolationBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_buffer_.clear();
}

bool PoseInterpolationBuffer::validate_time_stamp_difference(
  const builtin_interfaces::msg::Time & target_time,
  const builtin_interfaces::msg::Time & reference_time, const double time_tolerance_sec,
  std::vector<LogRequest> & logs) const
{
  const double dt =
    std::abs(to_seconds(to_nanoseconds(target_time) - to_nanoseconds(reference_time)));
  const bool success = dt < time_tolerance_sec;
  if (!success) {
    logs.push_back(
      {LogSite::PoseBufferTimeoutViolation,
       format(
         "Validation error. The reference time is %lf[sec], but the target time is %lf[sec]. The "
         "difference is %lf[sec] (the tolerance is %lf[sec]).",
         to_seconds(to_nanoseconds(reference_time)), to_seconds(to_nanoseconds(target_time)), dt,
         time_tolerance_sec)});
  }
  return success;
}

bool PoseInterpolationBuffer::validate_position_difference(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & reference_point,
  const double distance_tolerance_m, std::vector<LogRequest> & logs) const
{
  const double distance = norm(target_point, reference_point);
  const bool success = distance < distance_tolerance_m;
  if (!success) {
    logs.push_back(
      {LogSite::PoseBufferDistanceViolation,
       format(
         "Validation error. The distance from reference position to target position is %lf[m] (the "
         "tolerance is %lf[m]).",
         distance, distance_tolerance_m)});
  }
  return success;
}

}  // namespace autoware::ndt_scan_matcher
