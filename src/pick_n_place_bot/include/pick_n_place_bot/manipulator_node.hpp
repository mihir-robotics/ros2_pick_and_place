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
#include <vector>

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
  void execute_commands(const std::vector<std::string> & commands);
  bool wait_for_place2_aruco();
  void worker_thread();

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_sub_;

  int serial_fd_{-1};
  std::string serial_port_name_;

  std::mutex state_mutex_;
  std::condition_variable work_cv_;
  std::optional<int> pending_aruco_id_;
  bool is_busy_;
  bool waiting_for_place2_aruco_;
  bool place2_aruco_seen_;
  std::atomic<bool> shutdown_requested_;
  std::thread worker_;

  int command_delay_ms_;
  std::vector<std::string> pick_commands_;
  std::vector<std::string> place_commands_1_;
  std::vector<std::string> place_commands_2_;
};

#endif  // PICK_N_PLACE_BOT__MANIPULATOR_NODE_HPP_
