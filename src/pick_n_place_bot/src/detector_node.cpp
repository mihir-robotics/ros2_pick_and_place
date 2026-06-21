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

#include "pick_n_place_bot/detector_node.hpp"

#include <opencv2/aruco.hpp>

#include "cv_bridge/cv_bridge.hpp"

namespace
{

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_id_to_type(int id)
{
  switch (id) {
    case 0:
      return cv::aruco::DICT_4X4_50;
    case 1:
      return cv::aruco::DICT_5X5_100;
    case 2:
      return cv::aruco::DICT_6X6_250;
    case 3:
      return cv::aruco::DICT_7X7_1000;
    default:
      return cv::aruco::DICT_4X4_50;
  }
}

}  // namespace

DetectorNode::DetectorNode()
: Node("detector_node"), detected_marker_id_(-1), marker_detected_(false)
{
  // Declare parameters
  this->declare_parameter<int>("aruco_dictionary_id", 0);
  this->declare_parameter<double>("marker_size", 0.05);
  this->declare_parameter<double>("min_marker_perimeter_rate", 0.05);

  // Get parameters
  aruco_dictionary_id_ = this->get_parameter("aruco_dictionary_id").as_int();
  marker_size_ = this->get_parameter("marker_size").as_double();
  min_marker_perimeter_rate_ =
    this->get_parameter("min_marker_perimeter_rate").as_double();

  const auto dict_type = dictionary_id_to_type(aruco_dictionary_id_);
  if (aruco_dictionary_id_ < 0 || aruco_dictionary_id_ > 3) {
    RCLCPP_WARN(this->get_logger(),
      "Unknown aruco_dictionary_id %d, using DICT_4X4_50",
      aruco_dictionary_id_);
  }

  dictionary_ = cv::aruco::getPredefinedDictionary(dict_type);
  detector_params_ = cv::makePtr<cv::aruco::DetectorParameters>();
  detector_params_->minMarkerPerimeterRate = min_marker_perimeter_rate_;

  // Subscribe to camera/image — must match camera_node publisher topic name
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image", rclcpp::SensorDataQoS().best_effort(),
    std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));

  // Publish detected IDs as std_msgs/Int32 (matches manipulator_node subscriber)
  aruco_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "detector/aruco_id", rclcpp::SensorDataQoS().best_effort());

  RCLCPP_INFO(this->get_logger(),
    "Detector node initialized. Listening on camera/image");
}

DetectorNode::~DetectorNode() {}

void DetectorNode::image_callback(
  const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Convert ROS Image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;

    // Detect ArUco markers (OpenCV 4.5.x compatible API)
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids,
      detector_params_);

    if (!marker_ids.empty()) {
      detected_marker_id_ = marker_ids[0];
      marker_detected_ = true;

      // Publish marker ID on each frame while visible (manipulator ignores if busy)
      auto msg_id = std_msgs::msg::Int32();
      msg_id.data = detected_marker_id_;
      aruco_pub_->publish(msg_id);

      RCLCPP_DEBUG(this->get_logger(), "ArUco marker detected with ID: %d",
        detected_marker_id_);
    } else {
      marker_detected_ = false;
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectorNode>());
  rclcpp::shutdown();
  return 0;
}
