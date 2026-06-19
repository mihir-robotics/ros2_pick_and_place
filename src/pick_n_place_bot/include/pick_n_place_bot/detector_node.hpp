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

#ifndef PICK_N_PLACE_BOT__DETECTOR_NODE_HPP_
#define PICK_N_PLACE_BOT__DETECTOR_NODE_HPP_

#include <memory>

#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode();
  ~DetectorNode();

private:
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> image_sub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> aruco_pub_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  int aruco_dictionary_id_;
  double marker_size_;
  double min_marker_perimeter_rate_;
  int detected_marker_id_;
  bool marker_detected_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif  // PICK_N_PLACE_BOT__DETECTOR_NODE_HPP_
