// Copyright 2026 goober
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

#ifndef PICK_N_PLACE_BOT__MANIPULATOR_NODE_HPP_
#define PICK_N_PLACE_BOT__MANIPULATOR_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "pick_n_place_bot/arm_kinematics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class ManipulatorNode : public rclcpp::Node
{
public:
  ManipulatorNode();
  ~ManipulatorNode();

private:
  void aruco_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pick_n_place(int aruco_id);
  void send_command(const std::string & command);
  bool send_ik_pose(
    double x, double y, double z, int gripper, int wrist_angle = -1,
    int duration_ms = -1);
  double transit_z_for_place(double x, double y, double place_z) const;
  void worker_thread();

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_sub_;

  int serial_fd_{-1};
  std::string serial_port_name_;

  std::mutex state_mutex_;
  std::condition_variable work_cv_;
  std::optional<int> pending_aruco_id_;
  bool is_busy_;
  std::unordered_set<int> handled_aruco_ids_;
  std::atomic<bool> shutdown_requested_;
  std::thread worker_;

  int command_delay_ms_;
  int pose_duration_ms_;
  int pick_gripper_open_;
  int place_gripper_closed_;
  std::string gripper_close_cmd_;
  std::string gripper_open_cmd_;
  std::vector<std::string> pick_commands_;
  arm_kinematics::Params ik_params_;
  double pick_point_x_;
  double pick_point_y_;
  double pick_point_z_;
  double place_line_origin_x_;
  double place_line_origin_y_;
  double place_line_origin_z_;
  double place_line_step_x_;
  double place_line_step_y_;
  double place_line_step_z_;
  double place_clearance_m_;
  double gripper_open_width_m_;
  int post_place_wrist_tilt_ms_;
  int pre_pick_return_duration_ms_;
  int pre_pick_wrist_angle_;
  int post_place_retract_duration_ms_;
  int max_line_slots_;
};

#endif  // PICK_N_PLACE_BOT__MANIPULATOR_NODE_HPP_
