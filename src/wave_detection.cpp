#include <iostream>
#include <memory>
// C++ Standard lib 
#include <chrono>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "wave_detection/wave_detection.hpp"

#include "wave_detection/wave_detection.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

WaveDetection::WaveDetection(const rclcpp::NodeOptions & node_options)
: Node("wave_detection", node_options)

{
  this->declare_parameter<std::string>("image_topic", "/ouster/signal_image");
  this->declare_parameter<int>("wave_roi_start_row_percentage", 60); // 예시: 이미지 높이의 60%부터 파도 ROI 시작
  this->declare_parameter<int>("crest_threshold", 200); // 파고 임계값 (0-255)
  this->declare_parameter<int>("trough_threshold", 50);  // 파저 임계값 (0-255)

  std::string image_topic  = this->get_parameter("image_topic").as_string();
  wave_roi_start_row_percentage_ = this->get_parameter("wave_roi_start_row_percentage").as_int();
  crest_threshold_ = this->get_parameter("crest_threshold").as_int();
  trough_threshold_ = this->get_parameter("trough_threshold").as_int();

  RCLCPP_INFO(this->get_logger(), "Run wave_detection");
  RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", image_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Wave ROI start percentage: %d%%", wave_roi_start_row_percentage_);
  RCLCPP_INFO(this->get_logger(), "Crest threshold: %d", crest_threshold_);
  RCLCPP_INFO(this->get_logger(), "Trough threshold: %d", trough_threshold_);


  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(), std::bind(&WaveDetection::image_callback, this, _1));
  
  // 처리된 이미지를 퍼블리시할 퍼블리셔 생성
  processed_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/wave_detection/processed_image", 10);
  wave_crest_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/wave_detection/wave_crest_image", 10);
  wave_trough_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/wave_detection/wave_trough_image", 10);

  // // 마커 퍼블리셔 (선택 사항)
  // marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/wave_detection/markers", 10);


  // 50ms --> 20Hz 
  timer_ = this->create_wall_timer(50ms, std::bind(&WaveDetection::process, this));
}

WaveDetection::~WaveDetection()
{
  // OpenCV 창이 열려 있다면 닫기
  cv::destroyAllWindows();
}

// void WaveDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){image_data_ = msg;}

void WaveDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // ROS Image 메시지를 OpenCV Mat으로 변환
    // Ouster signal_image는 일반적으로 mono8 (단일 채널 8비트) 형식입니다.
    current_cv_image_ = cv_bridge::toCvCopy(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  image_data_ = msg; // 원본 메시지도 저장 (나중에 헤더 정보 등을 사용하기 위함)
}

double WaveDetection::get_sysSec_from_ros_time(const rclcpp::Time& ros_time)
{
  // ROS Time → system_clock 시간으로 변환
  std::chrono::nanoseconds ns_since_epoch(static_cast<int64_t>(ros_time.nanoseconds()));
  auto time_point = std::chrono::time_point<std::chrono::system_clock>(ns_since_epoch);

  // system_clock 기준으로 struct tm 얻기 (localtime)
  std::time_t t_c = std::chrono::system_clock::to_time_t(time_point);
  std::tm local_tm = *std::localtime(&t_c);

  // 마이크로초 추출
  auto duration_today = time_point - std::chrono::system_clock::from_time_t(t_c);
  auto microsec = std::chrono::duration_cast<std::chrono::microseconds>(duration_today).count();

  // 초 단위 시스템 시간 계산
  double sysSec = local_tm.tm_hour * 3600.0 +
                  local_tm.tm_min * 60.0 +
                  local_tm.tm_sec +
                  (microsec / 1e6);

  return sysSec;
}

void WaveDetection::process()
{
  if (current_cv_image_.empty()) // 이미지 데이터가 아직 없으면 처리하지 않음
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "'image' data not received or converted yet.");
    return;
  }
  
  rclcpp::Time current_time = this->now();
  // double cal_time = current_time.seconds(); // 사용되지 않음
  // double systime = get_sysSec_from_ros_time(current_time); // 현재는 이 노드에서 불필요

  // if (!start_time_set_) {
  //   start_time_sec_ = systime;
  //   start_time_set_ = true;
  // }
  // double time_since_start = systime - start_time_sec_; // 현재는 이 노드에서 불필요

  // 1. 관심 영역(ROI) 설정
  int img_height = current_cv_image_.rows;
  int img_width = current_cv_image_.cols;
  int roi_start_row = static_cast<int>(img_height * (wave_roi_start_row_percentage_ / 100.0));
  
  if (roi_start_row >= img_height) {
      RCLCPP_WARN(this->get_logger(), "ROI start row is out of bounds. Adjusting to img_height - 1.");
      roi_start_row = img_height - 1;
  }
  cv::Rect roi(0, roi_start_row, img_width, img_height - roi_start_row);
  cv::Mat wave_roi_image = current_cv_image_(roi);

  // ROI 이미지가 비어있는지 확인
  if (wave_roi_image.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Wave ROI image is empty. Check ROI parameters and input image dimensions.");
      return;
  }

  // 2. 파고(Wave Crest) 추출
  cv::Mat wave_crest_mask;
  cv::threshold(wave_roi_image, wave_crest_mask, crest_threshold_, 255, cv::THRESH_BINARY);

  // 3. 파저(Wave Trough) 추출
  cv::Mat wave_trough_mask;
  cv::threshold(wave_roi_image, wave_trough_mask, trough_threshold_, 255, cv::THRESH_BINARY_INV); // 임계값보다 낮은 픽셀을 흰색으로


  // 시각화를 위한 이미지 준비
  cv::Mat display_image;
  cv::cvtColor(wave_roi_image, display_image, cv::COLOR_GRAY2BGR); // ROI를 컬러 이미지로 변환

  // 파고 윤곽선 찾기
  std::vector<std::vector<cv::Point>> crest_contours;
  cv::findContours(wave_crest_mask.clone(), crest_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 파고 윤곽선 그리기 (초록색)
  for (const auto& contour : crest_contours) {
      // 원본 이미지 좌표계로 변환 (ROI 시작점 고려)
      std::vector<cv::Point> transformed_contour;
      for (const auto& pt : contour) {
          transformed_contour.push_back(cv::Point(pt.x, pt.y + roi_start_row));
      }
      cv::drawContours(display_image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2); // 초록색 (BGR)
  }

  // 파저 윤곽선 찾기
  std::vector<std::vector<cv::Point>> trough_contours;
  cv::findContours(wave_trough_mask.clone(), trough_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 파저 윤곽선 그리기 (파란색)
  for (const auto& contour : trough_contours) {
      // 원본 이미지 좌표계로 변환 (ROI 시작점 고려)
      std::vector<cv::Point> transformed_contour;
      for (const auto& pt : contour) {
          transformed_contour.push_back(cv::Point(pt.x, pt.y + roi_start_row));
      }
      cv::drawContours(display_image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255, 0, 0), 2); // 파란색 (BGR)
  }


  // OpenCV 창에 실시간으로 표시 (로컬에서 디버깅 시 유용)
  // cv::imshow("Original Image (ROI)", wave_roi_image); // ROI만 볼 때
  cv::imshow("Wave Detection Result", display_image);
  cv::imshow("Wave Crests Mask", wave_crest_mask);
  cv::imshow("Wave Troughs Mask", wave_trough_mask);
  cv::waitKey(1); // 1ms 대기, UI 업데이트

  // RViz2에 퍼블리시
  // 처리된 이미지 (컬러)
  try {
      std_msgs::msg::Header header = image_data_->header; // 원본 이미지의 헤더 사용
      sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(header, "bgr8", display_image).toImageMsg();
      processed_image_publisher_->publish(*processed_msg);

      // 파고 마스크 이미지 (흑백)
      sensor_msgs::msg::Image::SharedPtr crest_mask_msg = cv_bridge::CvImage(header, "mono8", wave_crest_mask).toImageMsg();
      wave_crest_image_publisher_->publish(*crest_mask_msg);

      // 파저 마스크 이미지 (흑백)
      sensor_msgs::msg::Image::SharedPtr trough_mask_msg = cv_bridge::CvImage(header, "mono8", wave_trough_mask).toImageMsg();
      wave_trough_image_publisher_->publish(*trough_mask_msg);

  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception during publishing: %s", e.what());
  }
}