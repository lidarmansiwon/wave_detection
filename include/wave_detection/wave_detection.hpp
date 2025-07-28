#ifndef WAVE__DETECTION__WAVE__DETECTION_HPP_
#define WAVE__DETECTION__WAVE__DETECTION_HPP_

#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


class WaveDetection : public rclcpp::Node
{
public:
  explicit WaveDetection(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~WaveDetection();

private:
  void process(); // 주기적 처리 함수
  double get_sysSec_from_ros_time(const rclcpp::Time& ros_time);

  // Callback 함수 정의
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // 객체 생성
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
//   rclcpp::Publisher<mk3_msgs::msg::NavigationType>::SharedPtr navigation_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  bool start_time_set_ = false;
  double start_time_sec_ = 0.0;
  // 수신한 메시지 저장
  sensor_msgs::msg::Image::SharedPtr image_data_{nullptr};
};
#endif  // ODOM_NAVI_NODE__ODOM_NAVI_NODE_HPP_