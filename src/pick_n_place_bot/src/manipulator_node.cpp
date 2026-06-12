#include "pick_n_place_bot/manipulator_node.hpp"
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

ManipulatorNode::ManipulatorNode()
    : Node("manipulator_node"),
      is_busy_(false),
      shutdown_requested_(false) {
  // Declare parameters
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("command_delay_ms", 1000);
  this->declare_parameter<std::vector<std::string>>(
      "pick_commands",
      std::vector<std::string>{"home", "gripper-100", "base_y-100"});
  this->declare_parameter<std::vector<std::string>>(
      "place_commands",
      std::vector<std::string>{"base_x-130", "base_y-95", "base_y-90",
                                 "gripper-180", "base_y-140","base_x-40",
                                 "base_y-110,shoulder-180","home"});

  // Get parameters
  serial_port_name_ = this->get_parameter("serial_port").as_string();
  command_delay_ms_ = this->get_parameter("command_delay_ms").as_int();
  pick_commands_ = this->get_parameter("pick_commands").as_string_array();
  place_commands_ = this->get_parameter("place_commands").as_string_array();

  // Initialize serial port
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

  // Subscribe to ArUco IDs published by detector_node (std_msgs/Int32)
  aruco_sub_ =
      this->create_subscription<std_msgs::msg::Int32>(
          "detector/aruco_id", rclcpp::SensorDataQoS().best_effort(),
          std::bind(&ManipulatorNode::aruco_callback, this,
                    std::placeholders::_1));

  // Run pick-and-place on a dedicated thread so callbacks return immediately
  worker_ = std::thread(std::bind(&ManipulatorNode::worker_thread, this));

  RCLCPP_INFO(this->get_logger(), "Manipulator node started");
  RCLCPP_INFO(this->get_logger(), "Command delay: %d ms", command_delay_ms_);
}

ManipulatorNode::~ManipulatorNode() {
  // Signal worker to exit and wait for any in-flight cycle to finish
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
    const std_msgs::msg::Int32::SharedPtr msg) {
  const int aruco_id = msg->data;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Reject overlapping requests; is_busy_ is cleared by the worker after each cycle
    if (is_busy_) {
      RCLCPP_WARN(this->get_logger(),
                  "Aruco marker %d detected but manipulator is busy", aruco_id);
      return;
    }

    is_busy_ = true;
    pending_aruco_id_ = aruco_id;
  }

  // Wake worker without holding the mutex (avoids deadlock with pick_n_place)
  work_cv_.notify_one();
}

void ManipulatorNode::pick_n_place(int aruco_id) {
  RCLCPP_INFO(this->get_logger(),
              "Starting pick and place for Aruco marker: %d", aruco_id);

  // Execute pick commands
  RCLCPP_INFO(this->get_logger(), "Executing pick commands");
  for (const auto& cmd : pick_commands_) {
    send_command(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(command_delay_ms_));
  }

  // Execute place commands
  RCLCPP_INFO(this->get_logger(), "Executing place commands");
  for (const auto& cmd : place_commands_) {
    send_command(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(command_delay_ms_));
  }

  RCLCPP_INFO(this->get_logger(), "Pick and place cycle complete");
}

void ManipulatorNode::send_command(const std::string& command) {
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
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "[SIM] Command: %s", command.c_str());
  }
}

void ManipulatorNode::worker_thread() {
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
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Pick and place failed: %s", e.what());
    }

    // Always release busy flag so the next ArUco ID can be accepted
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_busy_ = false;
    }
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManipulatorNode>());
  rclcpp::shutdown();
  return 0;
}
