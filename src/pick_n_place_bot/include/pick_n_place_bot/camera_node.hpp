#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

class CameraNode : public rclcpp::Node {
public:
  CameraNode();
  ~CameraNode();

private:
  void timer_callback();
  bool configure_camera();
  void reopen_camera();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::VideoCapture camera_;
  std::string camera_device_;
  int camera_device_id_;
  int frame_width_;
  int frame_height_;
  int frame_rate_;
  int consecutive_failures_{0};
};

#endif // CAMERA_NODE_HPP
