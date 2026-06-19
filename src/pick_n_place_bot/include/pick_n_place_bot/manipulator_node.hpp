#ifndef MANIPULATOR_NODE_HPP
#define MANIPULATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <atomic>

class ManipulatorNode : public rclcpp::Node {
public:
  ManipulatorNode();
  ~ManipulatorNode();

private:
  void aruco_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void pick_n_place(int aruco_id);
  void send_command(const std::string& command);
  void execute_commands(const std::vector<std::string>& commands);
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

#endif // MANIPULATOR_NODE_HPP
