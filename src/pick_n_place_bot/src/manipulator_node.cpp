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

#include "pick_n_place_bot/manipulator_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <sstream>

ManipulatorNode::ManipulatorNode()
: Node("manipulator_node"),
  is_busy_(false),
  shutdown_requested_(false)
{
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("command_delay_ms", 750);
  this->declare_parameter<std::vector<std::string>>(
    "pick_commands",
    std::vector<std::string>{"pose-40,50,0,0,180;300", "gripper-100"});

  this->declare_parameter("link_base_m", 0.06);
  this->declare_parameter("link_shoulder_m", 0.065);
  this->declare_parameter("link_wrist_m", 0.065);
  this->declare_parameter("ee_angle_offset_rad", 0.09);
  this->declare_parameter("prefer_elbow_up", true);
  this->declare_parameter("pose_duration_ms", 300);
  this->declare_parameter("pick_gripper_open", 180);
  this->declare_parameter("place_gripper_closed", 100);
  this->declare_parameter<std::string>("gripper_close_cmd", "gripper-100");
  this->declare_parameter<std::string>("gripper_open_cmd", "gripper-180");
  this->declare_parameter("pick_point_x", 0.058863);
  this->declare_parameter("pick_point_y", -0.070150);
  this->declare_parameter("pick_point_z", 0.068012);
  this->declare_parameter("place_line_origin_x", 0.064013);
  this->declare_parameter("place_line_origin_y", 0.064013);
  this->declare_parameter("place_line_origin_z", 0.075962);
  this->declare_parameter("place_line_step_x", 0.031850);
  this->declare_parameter("place_line_step_y", -0.032803);
  this->declare_parameter("place_line_step_z", 0.0);
  this->declare_parameter("place_clearance_m", 0.06);
  this->declare_parameter("gripper_open_width_m", 0.085);
  this->declare_parameter("post_place_wrist_tilt_ms", 400);
  this->declare_parameter("pre_pick_return_duration_ms", 400);
  this->declare_parameter("pre_pick_wrist_angle", 0);
  this->declare_parameter("post_place_retract_duration_ms", 400);
  this->declare_parameter("max_line_slots", 3);

  serial_port_name_ = this->get_parameter("serial_port").as_string();
  command_delay_ms_ = this->get_parameter("command_delay_ms").as_int();
  pick_commands_ = this->get_parameter("pick_commands").as_string_array();
  pose_duration_ms_ = this->get_parameter("pose_duration_ms").as_int();
  pick_gripper_open_ = this->get_parameter("pick_gripper_open").as_int();
  place_gripper_closed_ = this->get_parameter("place_gripper_closed").as_int();
  gripper_close_cmd_ = this->get_parameter("gripper_close_cmd").as_string();
  gripper_open_cmd_ = this->get_parameter("gripper_open_cmd").as_string();
  pick_point_x_ = this->get_parameter("pick_point_x").as_double();
  pick_point_y_ = this->get_parameter("pick_point_y").as_double();
  pick_point_z_ = this->get_parameter("pick_point_z").as_double();
  place_line_origin_x_ = this->get_parameter("place_line_origin_x").as_double();
  place_line_origin_y_ = this->get_parameter("place_line_origin_y").as_double();
  place_line_origin_z_ = this->get_parameter("place_line_origin_z").as_double();
  place_line_step_x_ = this->get_parameter("place_line_step_x").as_double();
  place_line_step_y_ = this->get_parameter("place_line_step_y").as_double();
  place_line_step_z_ = this->get_parameter("place_line_step_z").as_double();
  place_clearance_m_ = this->get_parameter("place_clearance_m").as_double();
  gripper_open_width_m_ = this->get_parameter("gripper_open_width_m").as_double();
  post_place_wrist_tilt_ms_ = this->get_parameter("post_place_wrist_tilt_ms").as_int();
  pre_pick_return_duration_ms_ = this->get_parameter("pre_pick_return_duration_ms").as_int();
  pre_pick_wrist_angle_ = this->get_parameter("pre_pick_wrist_angle").as_int();
  post_place_retract_duration_ms_ =
    this->get_parameter("post_place_retract_duration_ms").as_int();
  max_line_slots_ = this->get_parameter("max_line_slots").as_int();

  ik_params_.link_base = this->get_parameter("link_base_m").as_double();
  ik_params_.link1 = this->get_parameter("link_shoulder_m").as_double();
  ik_params_.link2 = this->get_parameter("link_wrist_m").as_double();
  ik_params_.ee_offset_rad = this->get_parameter("ee_angle_offset_rad").as_double();
  ik_params_.prefer_elbow_up = this->get_parameter("prefer_elbow_up").as_bool();

  serial_fd_ = ::open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to open serial port %s. Running in simulation mode.",
      serial_port_name_.c_str());
  } else {
    struct termios tty{};
    tcgetattr(serial_fd_, &tty);
    cfsetspeed(&tty, B9600);
    cfmakeraw(&tty);
    tcsetattr(serial_fd_, TCSANOW, &tty);
    RCLCPP_INFO(this->get_logger(), "Serial port: %s @ 9600 baud",
      serial_port_name_.c_str());
  }

  aruco_sub_ =
    this->create_subscription<std_msgs::msg::Int32>(
    "detector/aruco_id", rclcpp::SensorDataQoS().best_effort(),
    std::bind(&ManipulatorNode::aruco_callback, this,
      std::placeholders::_1));

  if (!pick_commands_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Moving to pre-pick pose on startup");
    send_command(pick_commands_.front());
  }

  worker_ = std::thread(std::bind(&ManipulatorNode::worker_thread, this));

  const double step_xy = std::hypot(place_line_step_x_, place_line_step_y_);
  if (step_xy + 1e-6 < gripper_open_width_m_) {
    RCLCPP_WARN(this->get_logger(),
      "Place line step %.1f cm is less than open gripper width %.1f cm — "
      "raise step or reduce slots to avoid knocking objects (arm reach limit)",
      step_xy * 100.0, gripper_open_width_m_ * 100.0);
  }

  RCLCPP_INFO(this->get_logger(), "Manipulator node started");
  RCLCPP_INFO(this->get_logger(), "Command delay: %d ms", command_delay_ms_);
}

ManipulatorNode::~ManipulatorNode()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    shutdown_requested_ = true;
  }
  work_cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void ManipulatorNode::aruco_callback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  const int aruco_id = msg->data;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (handled_aruco_ids_.count(aruco_id) != 0) {
      return;
    }

    if (static_cast<int>(handled_aruco_ids_.size()) >= max_line_slots_) {
      RCLCPP_INFO(this->get_logger(),
        "Aruco marker %d ignored — line full (%d slots)", aruco_id, max_line_slots_);
      return;
    }

    if (is_busy_) {
      RCLCPP_WARN(this->get_logger(),
        "Aruco marker %d detected but manipulator is busy", aruco_id);
      return;
    }

    is_busy_ = true;
    pending_aruco_id_ = aruco_id;
    RCLCPP_INFO(this->get_logger(),
      "Aruco marker %d detected — starting pick and place (line slot %zu)",
      aruco_id, handled_aruco_ids_.size());
  }

  work_cv_.notify_one();
}

void ManipulatorNode::send_command(const std::string & command)
{
  if (serial_fd_ >= 0) {
    std::string full_command = command + "\n";
    ssize_t bytes_sent =
      ::write(serial_fd_, full_command.c_str(), full_command.size());

    if (bytes_sent > 0) {
      RCLCPP_INFO(this->get_logger(), "Sent command: %s (%zd bytes)",
        command.c_str(), bytes_sent);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s",
        command.c_str());
      return;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "[SIM] Command: %s", command.c_str());
  }

  if (command_delay_ms_ > 0) {
    std::this_thread::sleep_for(
      std::chrono::milliseconds(command_delay_ms_));
  }
}

bool ManipulatorNode::send_ik_pose(
  double x, double y, double z, int gripper, int wrist_angle, int duration_ms)
{
  auto joints = arm_kinematics::solve_ik(x, y, z, ik_params_);
  if (!joints) {
    RCLCPP_WARN(this->get_logger(), "IK failed for (%.3f, %.3f, %.3f)", x, y, z);
    return false;
  }

  const int wrist = wrist_angle >= 0 ? wrist_angle : joints->wrist;
  const int dur = duration_ms >= 0 ? duration_ms : pose_duration_ms_;

  std::ostringstream cmd;
  cmd << "pose-" << joints->base_x << ',' << joints->base_y << ','
      << joints->shoulder << ',' << wrist << ',' << gripper
      << ';' << dur;

  send_command(cmd.str());
  return true;
}

double ManipulatorNode::transit_z_for_place(
  double x, double y, double place_z) const
{
  const double desired = place_z + place_clearance_m_;
  if (arm_kinematics::solve_ik(x, y, desired, ik_params_)) {
    return desired;
  }

  double lo = place_z;
  double hi = desired;
  for (int i = 0; i < 24; ++i) {
    const double mid = (lo + hi) * 0.5;
    if (arm_kinematics::solve_ik(x, y, mid, ik_params_)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  if (!arm_kinematics::solve_ik(x, y, lo, ik_params_)) {
    return place_z;
  }
  return lo;
}

void ManipulatorNode::pick_n_place(int aruco_id)
{
  RCLCPP_INFO(this->get_logger(),
    "Starting pick and place for Aruco marker: %d", aruco_id);

  if (pick_commands_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "pick_commands is empty — aborting cycle");
    return;
  }

  const size_t slot = handled_aruco_ids_.size();
  if (slot >= static_cast<size_t>(max_line_slots_)) {
    RCLCPP_ERROR(this->get_logger(),
      "No line slot %zu configured (max_line_slots=%d)", slot, max_line_slots_);
    return;
  }

  const double slot_n = static_cast<double>(slot);
  const double place_x = place_line_origin_x_ + slot_n * place_line_step_x_;
  const double place_y = place_line_origin_y_ + slot_n * place_line_step_y_;
  const double place_z = place_line_origin_z_;
  const double transit_z = transit_z_for_place(place_x, place_y, place_z);

  RCLCPP_INFO(this->get_logger(),
    "Line slot %zu at (%.3f, %.3f, %.3f)", slot, place_x, place_y, place_z);
  if (transit_z < place_z + place_clearance_m_ - 1e-4) {
    RCLCPP_WARN(this->get_logger(),
      "Transit height capped to %.3f m (requested %.3f m) for reach at slot %zu",
      transit_z, place_z + place_clearance_m_, slot);
  }

  RCLCPP_INFO(this->get_logger(), "Pre-pick pose (wrist up, waiting for grasp)");
  send_command(pick_commands_.front());

  RCLCPP_INFO(this->get_logger(), "Pick (level gripper) at (%.3f, %.3f, %.3f)",
    pick_point_x_, pick_point_y_, pick_point_z_);
  if (!send_ik_pose(pick_point_x_, pick_point_y_, pick_point_z_, pick_gripper_open_)) {
    return;
  }
  send_command(gripper_close_cmd_);

  RCLCPP_INFO(this->get_logger(), "Transit above place at z=%.3f", transit_z);
  if (!arm_kinematics::solve_ik(place_x, place_y, transit_z, ik_params_)) {
    RCLCPP_ERROR(this->get_logger(),
      "No reachable transit height for slot %zu at (%.3f, %.3f, %.3f)",
      slot, place_x, place_y, place_z);
    return;
  }
  if (!send_ik_pose(place_x, place_y, transit_z, place_gripper_closed_)) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Place");
  if (!send_ik_pose(place_x, place_y, place_z, place_gripper_closed_)) {
    return;
  }
  send_command(gripper_open_cmd_);

  RCLCPP_INFO(this->get_logger(), "Tilt wrist up slowly before leaving place");
  {
    std::ostringstream wrist_cmd;
    wrist_cmd << "wrist-" << pre_pick_wrist_angle_ << '-' << post_place_wrist_tilt_ms_;
    send_command(wrist_cmd.str());
  }

  RCLCPP_INFO(this->get_logger(), "Retract up from placement zone");
  if (!send_ik_pose(
      place_x, place_y, transit_z, pick_gripper_open_, pre_pick_wrist_angle_,
      post_place_retract_duration_ms_))
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Returning to pre-pick pose (wrist up, slow)");
  {
    const std::string & pre_pick = pick_commands_.front();
    const auto semi = pre_pick.rfind(';');
    if (semi != std::string::npos) {
      send_command(pre_pick.substr(0, semi + 1) + std::to_string(pre_pick_return_duration_ms_));
    } else {
      send_command(pre_pick);
    }
  }

  handled_aruco_ids_.insert(aruco_id);
  RCLCPP_INFO(this->get_logger(), "Pick and place complete for Aruco %d", aruco_id);
}

void ManipulatorNode::worker_thread()
{
  while (true) {
    int aruco_id = 0;

    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      work_cv_.wait(lock, [this] {
          return pending_aruco_id_.has_value() || shutdown_requested_;
        });

      if (shutdown_requested_ && !pending_aruco_id_.has_value()) {
        break;
      }

      aruco_id = *pending_aruco_id_;
      pending_aruco_id_.reset();
    }

    try {
      pick_n_place(aruco_id);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Pick and place failed: %s", e.what());
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_busy_ = false;
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManipulatorNode>());
  rclcpp::shutdown();
  return 0;
}
