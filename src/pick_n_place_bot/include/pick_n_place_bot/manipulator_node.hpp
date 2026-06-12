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
  /**
   * @brief ROS callback for newly detected ArUco marker IDs.
   *
   * Keeps work lightweight: only validates input and queues a job for the
   * worker thread. Must not run pick-and-place here (would block the executor).
   */
  void aruco_callback(const std_msgs::msg::Int32::SharedPtr msg);

  /**
   * @brief Perform pick and place operation on the worker thread.
   * @param aruco_id The detected Aruco marker ID
   */
  void pick_n_place(int aruco_id);

  /**
   * @brief Send command to Arduino via serial port
   * @param command String command to send
   */
  void send_command(const std::string& command);

  /**
   * @brief Background worker that runs queued pick-and-place cycles.
   *
   * Long-running serial commands and delays run here so the ROS executor
   * stays responsive to new /detector/aruco_id messages.
   */
  void worker_thread();

  // Subscriber (std_msgs/Int32 matches detector_node publisher)
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aruco_sub_;

  // Serial communication
  int serial_fd_{-1};
  std::string serial_port_name_;

  // State management — mutex protects is_busy_, pending_aruco_id_, and shutdown
  std::mutex state_mutex_;
  std::condition_variable work_cv_;
  std::optional<int> pending_aruco_id_;
  bool is_busy_;  // True while a pick-and-place cycle is queued or running
  std::atomic<bool> shutdown_requested_;
  std::thread worker_;

  // Pick and place parameters
  int command_delay_ms_;
  std::vector<std::string> pick_commands_;
  std::vector<std::string> place_commands_;
};

#endif // MANIPULATOR_NODE_HPP
