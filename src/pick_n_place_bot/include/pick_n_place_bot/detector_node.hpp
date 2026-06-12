#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp> 
#include <memory>

class DetectorNode : public rclcpp::Node {
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

#endif // DETECTOR_NODE_HPP
