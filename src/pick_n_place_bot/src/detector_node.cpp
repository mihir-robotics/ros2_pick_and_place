#include "pick_n_place_bot/detector_node.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/aruco.hpp>
#include <sstream>

DetectorNode::DetectorNode()
    : Node("detector_node"), detected_marker_id_(-1), marker_detected_(false) {

  // Declare parameters
  this->declare_parameter<int>("aruco_dictionary_id", 0);
  this->declare_parameter<double>("marker_size", 0.05);
  this->declare_parameter<double>("min_marker_perimeter_rate", 0.80);

  // Get parameters
  aruco_dictionary_id_ = this->get_parameter("aruco_dictionary_id").as_int();
  marker_size_ = this->get_parameter("marker_size").as_double();
  min_marker_perimeter_rate_ =
      this->get_parameter("min_marker_perimeter_rate").as_double();

  // Initialize ArUco detector (OpenCV 4.5.x compatible API)
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  detector_params_ = cv::makePtr<cv::aruco::DetectorParameters>();

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
    const sensor_msgs::msg::Image::SharedPtr msg) {
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

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectorNode>());
  rclcpp::shutdown();
  return 0;
}
