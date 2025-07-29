#ifndef WAVE_DETECTION_HPP
#define WAVE_DETECTION_HPP

#include <iostream>
#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h" // cv_bridge 헤더 추가
#include "opencv2/opencv.hpp"   // OpenCV 헤더 추가
#include "opencv2/highgui/highgui.hpp" // imshow 등을 위한 highgui

// 시각화를 위한 추가 메시지 타입 (선택 사항)
#include "sensor_msgs/msg/image.hpp" // 처리된 이미지 퍼블리시
// #include "visualization_msgs/msg/marker_array.hpp" // 마커 퍼블리시 (선택 사항)

using std::placeholders::_1;
using namespace std::chrono_literals;

class WaveDetection : public rclcpp::Node
{
public:
  explicit WaveDetection(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~WaveDetection();

private:
  void process(); // 주기적 처리 함수
  double get_sysSec_from_ros_time(const rclcpp::Time& ros_time);

  // 트랙바 콜백 함수 (static으로 선언하고, 사용자 데이터를 통해 인스턴스에 접근)
  static void on_trackbar_crest(int val, void* userdata);
  static void on_trackbar_trough(int val, void* userdata);
  static void on_trackbar_roi(int val, void* userdata);

  // Canny 임계값 트랙바 콜백 함수 추가
  static void on_trackbar_canny_low(int val, void* userdata);
  static void on_trackbar_canny_high(int val, void* userdata);

  // Callback 함수 정의
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // 객체 생성
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
//   rclcpp::Publisher<mk3_msgs::msg::NavigationType>::SharedPtr navigation_publisher_;

  sensor_msgs::msg::Image::SharedPtr image_data_; // Raw image data from topic
  cv::Mat current_cv_image_; // OpenCV Mat to store the current image

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // 이미지 퍼블리셔 추가
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr wave_crest_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr wave_trough_image_publisher_;

  // // 마커 퍼블리셔 추가 (선택 사항)
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  // 파도 분석 관련 파라미터
  int wave_roi_start_row_percentage_; // 파도 ROI 시작 행 (전체 이미지 높이에 대한 백분율)
  int crest_threshold_; // 파고 추출을 위한 밝기 임계값
  int trough_threshold_; // 파저 추출을 위한 밝기 임계값

  // Canny 임계값 멤버 변수 추가
  int canny_low_threshold_;
  int canny_high_threshold_;

  std::string window_name_ = "Wave Detection Result"; // 창 이름 멤버 변수

  rclcpp::Time start_time_;
  bool start_time_set_ = false;
  double start_time_sec_ = 0.0;
  // 수신한 메시지 저장
  // sensor_msgs::msg::Image::SharedPtr image_data_{nullptr};
};
#endif  // ODOM_NAVI_NODE__ODOM_NAVI_NODE_HPP_