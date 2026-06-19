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

#ifndef PICK_N_PLACE_BOT__CAMERA_NODE_HPP_
#define PICK_N_PLACE_BOT__CAMERA_NODE_HPP_

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node
{
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

#endif  // PICK_N_PLACE_BOT__CAMERA_NODE_HPP_
