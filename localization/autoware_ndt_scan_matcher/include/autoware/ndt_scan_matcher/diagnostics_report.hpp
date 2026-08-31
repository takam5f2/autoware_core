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

  // PoseInitializationModule. The two Align* warnings were throttled at 1000 ms; the other three
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

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__DIAGNOSTICS_REPORT_HPP_
