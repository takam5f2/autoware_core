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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::ndt_scan_matcher
{

// Severity of a diagnostics update. Mirrors diagnostic_msgs::msg::DiagnosticStatus levels so that
// the modules that produce these need no ROS diagnostics dependency.
enum class DiagnosticLevel : int8_t { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };

// A single diagnostic key/value. The value keeps its type so the ROS node can format it exactly
// the way DiagnosticsInterface would (e.g. bool as "True"/"False").
//
// Keep the alternatives matched to the std::to_string() overload the original call site used:
// float and double format identically (both go through "%f"), and so do the integer widths in
// play here, but an unsigned value routed through double would not.
struct DiagnosticKeyValue
{
  std::string key;
  std::variant<bool, int64_t, double, std::string> value;
};

// Identifies the log call site a core module would have logged from. The ROS node switches on it
// and logs from a `case` of its own, which is what keeps each site's severity, and the throttle
// state of the throttled ones, exactly where it was: RCLCPP_*_THROTTLE holds its state per macro
// expansion, so replaying several sites through one shared macro would let an unrelated failure
// consume another's throttle window.
//
// Severity belongs to the site, not to the record -- carrying it in both would give two answers to
// one question. Add a value here only together with the `case` that logs it.
enum class LogSite : uint8_t {
  // autoware::localization_util::SmartPoseBuffer's four, in PoseInterpolationBuffer. None of these
  // were throttled.
  PoseBufferTooFewSamples,      // INFO
  PoseBufferStampMismatch,      // INFO
  PoseBufferTimeoutViolation,   // WARN
  PoseBufferDistanceViolation,  // WARN

  // ScanMatchingModule's hot path. All three were throttled at 1000 ms.
  ScanTransformFailed,      // ERROR
  ScanOutOfMapRange,        // WARN
  ScanScoreBelowThreshold,  // WARN

  // PoseInitializationSearch. The two Align* warnings were throttled at 1000 ms; the other three
  // were not.
  AlignNoInputTarget,  // WARN, throttled
  AlignNoInputSource,  // WARN, throttled
  AlignUnstableScore,  // WARN
  AlignPoseInput,      // DEBUG
  AlignPoseOutput,     // DEBUG
};

// One log line a core module would have emitted. Modules record; the node logs.
struct LogRequest
{
  LogSite site;
  std::string message;
};

// Diagnostics accumulated while a module runs: key/values plus an overall status (level +
// message). Returned to the ROS node, which forwards it to a DiagnosticsInterface. Plain data,
// kept ROS-free on purpose.
//
// One report is threaded through a whole call in order, across module boundaries, because
// update_level_and_message() joins messages with "; " and the joined text is observable.
struct DiagnosticsReport
{
  DiagnosticLevel level{DiagnosticLevel::OK};
  std::string message;
  std::vector<DiagnosticKeyValue> key_values;
  // Independent of the three above: these go to the logger, not into the DiagnosticStatus.
  std::vector<LogRequest> logs;

  void add_key_value(DiagnosticKeyValue key_value) { key_values.push_back(std::move(key_value)); }

  // A yes/no precondition, which is the shape almost every check in this package has: record it
  // as a key either way, and on failure raise the level and append the message. Returns `value`,
  // so a call site reads as the condition it is:
  //
  //   if (!diagnostics.check("is_set_map_points", ndt.hasTarget(), WARN, "No InputTarget...")) {
  //     return result;
  //   }
  bool check(
    std::string key, const bool value, const DiagnosticLevel level_on_failure,
    const std::string & message_on_failure)
  {
    add_key_value({std::move(key), value});
    if (!value) {
      update_level_and_message(level_on_failure, message_on_failure);
    }
    return value;
  }

  // As above, and the failure also goes to the log site it used to be logged from.
  bool check(
    std::string key, const bool value, const DiagnosticLevel level_on_failure,
    const std::string & message_on_failure, const LogSite site)
  {
    if (!check(std::move(key), value, level_on_failure, message_on_failure)) {
      logs.push_back({site, message_on_failure});
      return false;
    }
    return true;
  }

  // Accumulates like DiagnosticsInterface: raises the level and appends the message. An empty
  // message is skipped, as it is there -- without that, `append` below would join a separator
  // onto nothing for a report that was raised without a message.
  void update_level_and_message(DiagnosticLevel new_level, const std::string & new_message)
  {
    if (
      static_cast<int8_t>(new_level) > static_cast<int8_t>(DiagnosticLevel::OK) &&
      !new_message.empty()) {
      if (!message.empty()) {
        message += "; ";
      }
      message += new_message;
    }
    if (static_cast<int8_t>(new_level) > static_cast<int8_t>(level)) {
      level = new_level;
    }
  }

  // Merges a report produced later into this one: keys and logs in call order, level and message
  // accumulated as above.
  //
  // This is what forwarding two reports to one DiagnosticsInterface in turn used to do, and it is
  // what keeps `AligningOutsideMapRangeFailsWithThreeJoinedMessages` joined in the right order now
  // that the map update and the pose search report into a single call's result.
  void append(DiagnosticsReport other)
  {
    for (auto & key_value : other.key_values) {
      key_values.push_back(std::move(key_value));
    }
    for (auto & log : other.logs) {
      logs.push_back(std::move(log));
    }
    update_level_and_message(other.level, other.message);
  }
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_
