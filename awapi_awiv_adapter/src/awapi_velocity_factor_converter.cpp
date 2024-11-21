// Copyright 2024 TIER IV, Inc.
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

#include "awapi_awiv_adapter/awapi_velocity_factor_converter.hpp"

#include <memory>
#include <vector>

namespace autoware_api
{
AutowareIvVelocityFactorConverter::AutowareIvVelocityFactorConverter(
  rclcpp::Node & node, const double thresh_dist_to_stop_pose)
: logger_(node.get_logger().get_child("awapi_awiv_velocity_factor_converter")),
  clock_(node.get_clock()),
  thresh_dist_to_stop_pose_(thresh_dist_to_stop_pose)
{
}

tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr
AutowareIvVelocityFactorConverter::updateStopReasonArray(
  const autoware_adapi_v1_msgs::msg::VelocityFactorArray::ConstSharedPtr & msg_ptr)
{
  return convert(msg_ptr);
}

tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr
AutowareIvVelocityFactorConverter::convert(
  const autoware_adapi_v1_msgs::msg::VelocityFactorArray::ConstSharedPtr & msg_ptr)
{
  tier4_planning_msgs::msg::StopReasonArray stop_reason_array_msg;
  // input header
  stop_reason_array_msg.header.frame_id = "map";
  stop_reason_array_msg.header.stamp = clock_->now();

  // input stop reason
  for (const auto & factor : msg_ptr->factors) {
    // stop reason doesn't has corresponding factor.
    if (conversion_map.count(factor.behavior) == 0) {
      continue;
    }

    // append only near velocity factor.
    if (factor.distance > thresh_dist_to_stop_pose_) {
      continue;
    }

    // TODO(satoshi-ota): it's better to check if it is stop factor.
    tier4_planning_msgs::msg::StopReason near_stop_reason;
    near_stop_reason.reason = conversion_map.at(factor.behavior);
    near_stop_reason.stop_factors.push_back(convert(factor));

    stop_reason_array_msg.stop_reasons.push_back(near_stop_reason);
  }

  return std::make_shared<tier4_planning_msgs::msg::StopReasonArray>(stop_reason_array_msg);
}

tier4_planning_msgs::msg::StopFactor AutowareIvVelocityFactorConverter::convert(
  const autoware_adapi_v1_msgs::msg::VelocityFactor & factor)
{
  tier4_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose = factor.pose;
  stop_factor.dist_to_stop_pose = factor.distance;

  return stop_factor;
}

}  // namespace autoware_api
