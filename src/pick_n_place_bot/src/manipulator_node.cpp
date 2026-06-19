#include "pick_n_place_bot/manipulator_node.hpp"
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

ManipulatorNode::ManipulatorNode()
    : Node("manipulator_node"),
      is_busy_(false),
      waiting_for_place2_aruco_(false),
      place2_aruco_seen_(false),
      shutdown_requested_(false) {
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("command_delay_ms", 750);
  this->declare_parameter<std::vector<std::string>>(
      "pick_commands",
      std::vector<std::string>{"pose-40,50,0,0,180;300", "wrist-45-500", "gripper-100"});
  this->declare_parameter<std::vector<std::string>>(
      "place_commands_1",
      std::vector<std::string>{"pose-135,55,0,55,100;300", "gripper-180",
                               "wrist-0-500", "home-500"});
  this->declare_parameter<std::vector<std::string>>(
      "place_commands_2",
      std::vector<std::string>{"pose-110,45,10,60,100;300", "gripper-180",
                               "wrist-0-500", "home-500"});

  serial_port_name_ = this->get_parameter("serial_port").as_string();
  command_delay_ms_ = this->get_parameter("command_delay_ms").as_int();
  pick_commands_ = this->get_parameter("pick_commands").as_string_array();
  place_commands_1_ = this->get_parameter("place_commands_1").as_string_array();
  place_commands_2_ = this->get_parameter("place_commands_2").as_string_array();

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

  RCLCPP_INFO(this->get_logger(), "Manipulator node started");
  RCLCPP_INFO(this->get_logger(), "Command delay: %d ms", command_delay_ms_);
}

ManipulatorNode::~ManipulatorNode() {
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

    if (waiting_for_place2_aruco_) {
      place2_aruco_seen_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Aruco marker %d detected — running pick and place_commands_2",
                  aruco_id);
    } else if (is_busy_) {
      RCLCPP_WARN(this->get_logger(),
                  "Aruco marker %d detected but manipulator is busy", aruco_id);
      return;
    } else {
      is_busy_ = true;
      pending_aruco_id_ = aruco_id;
      RCLCPP_INFO(this->get_logger(),
                  "Aruco marker %d detected — starting pick and place_commands_1",
                  aruco_id);
    }
  }

  work_cv_.notify_one();
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

void ManipulatorNode::execute_commands(
    const std::vector<std::string>& commands) {
  for (const auto& cmd : commands) {
    send_command(cmd);
  }
}

bool ManipulatorNode::wait_for_place2_aruco() {
  RCLCPP_INFO(this->get_logger(),
              "At pre-pick pose — waiting for Aruco marker to run place_commands_2");

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    waiting_for_place2_aruco_ = true;
    place2_aruco_seen_ = false;
  }

  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    work_cv_.wait(lock, [this] {
      return place2_aruco_seen_ || shutdown_requested_;
    });
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    waiting_for_place2_aruco_ = false;
  }

  return place2_aruco_seen_;
}

void ManipulatorNode::pick_n_place(int aruco_id) {
  RCLCPP_INFO(this->get_logger(),
              "Starting pick and place for Aruco marker: %d", aruco_id);

  if (pick_commands_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "pick_commands is empty — aborting cycle");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Executing pick commands");
  execute_commands(pick_commands_);

  RCLCPP_INFO(this->get_logger(), "Executing place_commands_1");
  execute_commands(place_commands_1_);

  RCLCPP_INFO(this->get_logger(), "Returning to pre-pick pose");
  send_command(pick_commands_.front());

  if (!wait_for_place2_aruco()) {
    RCLCPP_INFO(this->get_logger(),
                "Cycle aborted while waiting for second Aruco marker");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Executing pick commands");
  execute_commands(pick_commands_);

  RCLCPP_INFO(this->get_logger(), "Executing place_commands_2");
  execute_commands(place_commands_2_);

  RCLCPP_INFO(this->get_logger(), "Pick and place cycle complete");
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
