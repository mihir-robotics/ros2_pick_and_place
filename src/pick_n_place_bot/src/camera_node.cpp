#include "pick_n_place_bot/camera_node.hpp"
#include "cv_bridge/cv_bridge.hpp"

CameraNode::CameraNode()
    : Node("camera_node") {
  this->declare_parameter<std::string>( "camera_device", "/dev/video0");
  this->declare_parameter<int>("camera_device_id", 0);
  this->declare_parameter<int>("frame_width", 640);
  this->declare_parameter<int>("frame_height", 480);
  this->declare_parameter<int>("frame_rate", 30);

  camera_device_ = this->get_parameter("camera_device").as_string();
  camera_device_id_ = this->get_parameter("camera_device_id").as_int();
  frame_width_ = this->get_parameter("frame_width").as_int();
  frame_height_ = this->get_parameter("frame_height").as_int();
  frame_rate_ = this->get_parameter("frame_rate").as_int();

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "camera/image", rclcpp::SensorDataQoS().best_effort());

  if (!camera_device_.empty()) {
    camera_.open(camera_device_, cv::CAP_V4L2);
  } else {
    camera_.open(camera_device_id_, cv::CAP_V4L2);
  }

  if (!camera_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to open camera (device=%s, id=%d)",
                 camera_device_.c_str(), camera_device_id_);
    throw std::runtime_error("Camera open failed");
  }

  if (!configure_camera()) {
    throw std::runtime_error("Camera configuration failed");
  }

  if (camera_device_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Camera opened: /dev/video%d (%dx%d @ %d fps)",
                camera_device_id_, frame_width_, frame_height_, frame_rate_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Camera opened: %s (%dx%d @ %d fps)",
                camera_device_.c_str(), frame_width_, frame_height_, frame_rate_);
  }

  const int timer_period_ms = std::max(1, 1000 / frame_rate_);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms),
      std::bind(&CameraNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
              "Camera node started, publishing to /camera/image");
}

CameraNode::~CameraNode() {
  if (camera_.isOpened()) {
    camera_.release();
  }
}

bool CameraNode::configure_camera() {
  // MJPEG reduces USB bandwidth; set before resolution on most webcams
  camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  camera_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
  camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
  camera_.set(cv::CAP_PROP_FPS, frame_rate_);

  // Discard stale buffered frames after (re)configuration
  for (int i = 0; i < 5; ++i) {
    camera_.grab();
  }

  cv::Mat test_frame;
  if (!camera_.grab() || !camera_.retrieve(test_frame) || test_frame.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Camera opened but failed to read an initial frame");
    return false;
  }

  consecutive_failures_ = 0;
  return true;
}

void CameraNode::reopen_camera() {
  RCLCPP_WARN(this->get_logger(), "Reopening camera after read failures");
  camera_.release();

  if (!camera_device_.empty()) {
    camera_.open(camera_device_, cv::CAP_V4L2);
  } else {
    camera_.open(camera_device_id_, cv::CAP_V4L2);
  }

  if (!camera_.isOpened() || !configure_camera()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reopen camera");
    consecutive_failures_ = 0;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Camera reopened successfully");
}

void CameraNode::timer_callback() {
  if (!camera_.grab()) {
    ++consecutive_failures_;
    RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera");
    if (consecutive_failures_ >= 3) {
      reopen_camera();
    }
    return;
  }

  cv::Mat frame;
  if (!camera_.retrieve(frame) || frame.empty()) {
    ++consecutive_failures_;
    RCLCPP_WARN(this->get_logger(), "Failed to retrieve frame from camera");
    if (consecutive_failures_ >= 3) {
      reopen_camera();
    }
    return;
  }

  consecutive_failures_ = 0;

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                 .toImageMsg();
  msg->header.stamp = this->now();
  msg->header.frame_id = "camera";

  image_pub_->publish(*msg);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
