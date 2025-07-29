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
  this->declare_parameter<int>("wave_roi_start_row_percentage", 70); // 예시: 이미지 높이의 60%부터 파도 ROI 시작
  this->declare_parameter<int>("crest_threshold", 50); // 파고 임계값 (0-255)
  this->declare_parameter<int>("trough_threshold", 200);  // 파저 임계값 (0-255)
  // Canny 임계값 파라미터 선언 및 기본값 설정
  this->declare_parameter<int>("canny_low_threshold", 50);
  this->declare_parameter<int>("canny_high_threshold", 150);

  std::string image_topic  = this->get_parameter("image_topic").as_string();
  wave_roi_start_row_percentage_ = this->get_parameter("wave_roi_start_row_percentage").as_int();
  crest_threshold_ = this->get_parameter("crest_threshold").as_int();
  trough_threshold_ = this->get_parameter("trough_threshold").as_int();
  canny_low_threshold_ = this->get_parameter("canny_low_threshold").as_int();
  canny_high_threshold_ = this->get_parameter("canny_high_threshold").as_int();



  RCLCPP_INFO(this->get_logger(), "Run wave_detection");
  RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", image_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Wave ROI start percentage: %d%%", wave_roi_start_row_percentage_);
  RCLCPP_INFO(this->get_logger(), "Crest threshold: %d", crest_threshold_);
  RCLCPP_INFO(this->get_logger(), "Trough threshold: %d", trough_threshold_);
  RCLCPP_INFO(this->get_logger(), "Canny Low Threshold: %d", canny_low_threshold_);
  RCLCPP_INFO(this->get_logger(), "Canny High Threshold: %d", canny_high_threshold_);


  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(), std::bind(&WaveDetection::image_callback, this, _1));
  
  // 처리된 이미지를 퍼블리시할 퍼블리셔 생성
  processed_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/wave_detection/processed_image", 10);
  wave_crest_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/wave_detection/wave_crest_image", 10);
  wave_trough_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/wave_detection/wave_trough_image", 10);

  // // 마커 퍼블리셔 (선택 사항)
  // marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/wave_detection/markers", 10);

  // OpenCV 창과 트랙바 생성
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE); // resizable 창 생성
  cv::createTrackbar("ROI Start %", window_name_, &wave_roi_start_row_percentage_, 100, on_trackbar_roi, this);
  cv::createTrackbar("Crest Thresh", window_name_, &crest_threshold_, 255, on_trackbar_crest, this);
  cv::createTrackbar("Trough Thresh", window_name_, &trough_threshold_, 255, on_trackbar_trough, this);
  cv::createTrackbar("Canny Low", window_name_, &canny_low_threshold_, 510, on_trackbar_canny_low, this);
  cv::createTrackbar("Canny High", window_name_, &canny_high_threshold_, 510, on_trackbar_canny_high, this);
  
  // 초기값으로 설정된 파라미터가 트랙바에 반영되도록 설정 (필수)
  cv::setTrackbarPos("ROI Start %", window_name_, wave_roi_start_row_percentage_);
  cv::setTrackbarPos("Crest Thresh", window_name_, crest_threshold_);
  cv::setTrackbarPos("Trough Thresh", window_name_, trough_threshold_);
  cv::setTrackbarPos("Canny Low", window_name_, canny_low_threshold_);
  cv::setTrackbarPos("Canny High", window_name_, canny_high_threshold_);




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
  if (current_cv_image_.empty())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "'image' data not received or converted yet.");
    return;
  }
  
  // rclcpp::Time current_time = this->now(); // 사용되지 않음

  // 1. 관심 영역(ROI) 설정
  int img_height = current_cv_image_.rows;
  int img_width = current_cv_image_.cols;
  // 트랙바를 통해 업데이트된 wave_roi_start_row_percentage_ 사용
  int roi_start_row = static_cast<int>(img_height * (wave_roi_start_row_percentage_ / 100.0));
  
  if (roi_start_row >= img_height) {
      RCLCPP_WARN(this->get_logger(), "ROI start row is out of bounds. Adjusting to img_height - 1.");
      roi_start_row = img_height - 1;
  }
  cv::Rect roi(0, roi_start_row, img_width, img_height - roi_start_row);
  cv::Mat wave_roi_image = current_cv_image_(roi);

  if (wave_roi_image.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Wave ROI image is empty. Check ROI parameters and input image dimensions.");
      return;
  }

  // 3. 파저(Wave Trough) 추출
  cv::Mat wave_trough_mask;
  // 트랙바를 통해 업데이트된 trough_threshold_ 사용
  cv::threshold(wave_roi_image, wave_trough_mask, trough_threshold_, 255, cv::THRESH_BINARY);


  // 시각화를 위한 이미지 준비
  cv::Mat display_image;
  cv::cvtColor(wave_roi_image, display_image, cv::COLOR_GRAY2BGR); // ROI를 컬러 이미지로 변환

  // 파저 윤곽선 찾기
  std::vector<std::vector<cv::Point>> trough_contours;
  cv::findContours(wave_trough_mask.clone(), trough_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 파저 윤곽선 그리기 (파란색)
  for (const auto& contour : trough_contours) {
      // 위와 동일하게 display_image는 ROI 이미지이므로 변환 불필요
      cv::drawContours(display_image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255, 0, 0), 2); // 파란색 (BGR)
  }

  cv::Mat canny_output_crest;
  // Canny 임계값도 조절 필요합니다. wave_crest_mask가 이진 이미지이므로
  // Canny는 사실상 엣지를 더 가늘게 만듭니다.
  cv::Canny(wave_roi_image, canny_output_crest, canny_low_threshold_, canny_high_threshold_, 3);

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(canny_output_crest, lines, 1, CV_PI / 180,
                  20,    // threshold: 투표 수를 낮춰 더 많은 선 검출 시도
                  30,    // minLineLength: 최소 직선 길이를 줄여 짧은 선도 검출 시도
                  10);   // maxLineGap: 간격 허용

  // 검출된 모든 파고 직선들을 display_image에 그리기 (빨간색)
  // 각도 필터링은 일단 제거하여 모든 직선이 그려지는지 확인합니다.
  for (const auto& l : lines) {
      cv::line(display_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA); // 빨간색 선
  }

  // OpenCV 창에 실시간으로 표시
  cv::imshow(window_name_, display_image); // 창 이름 변경
  cv::imshow("Wave Troughs Mask", wave_trough_mask);
    cv::imshow("Canny Output Crest", canny_output_crest); // 이 창을 꼭 확인
  cv::waitKey(1); // 1ms 대기, UI 업데이트

  // RViz2에 퍼블리시
  try {
      std_msgs::msg::Header header = image_data_->header;
      sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(header, "bgr8", display_image).toImageMsg();
      processed_image_publisher_->publish(*processed_msg);

      // sensor_msgs::msg::Image::SharedPtr crest_mask_msg = cv_bridge::CvImage(header, "mono8", wave_crest_mask).toImageMsg();
      // wave_crest_image_publisher_->publish(*crest_mask_msg);

      sensor_msgs::msg::Image::SharedPtr trough_mask_msg = cv_bridge::CvImage(header, "mono8", wave_trough_mask).toImageMsg();
      wave_trough_image_publisher_->publish(*trough_mask_msg);

  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception during publishing: %s", e.what());
  }
}

// 트랙바 콜백 함수 구현
// static 함수이므로 클래스 외부에서 정의
void WaveDetection::on_trackbar_crest(int val, void* userdata) {
    WaveDetection* node = static_cast<WaveDetection*>(userdata);
    node->crest_threshold_ = val;
    RCLCPP_INFO(node->get_logger(), "Crest Threshold updated to: %d", val);
}

void WaveDetection::on_trackbar_trough(int val, void* userdata) {
    WaveDetection* node = static_cast<WaveDetection*>(userdata);
    node->trough_threshold_ = val;
    RCLCPP_INFO(node->get_logger(), "Trough Threshold updated to: %d", val);
}

void WaveDetection::on_trackbar_roi(int val, void* userdata) {
    WaveDetection* node = static_cast<WaveDetection*>(userdata);
    node->wave_roi_start_row_percentage_ = val;
    RCLCPP_INFO(node->get_logger(), "ROI Start Row Percentage updated to: %d", val);
}

// Canny 임계값 트랙바 콜백 함수 구현
void WaveDetection::on_trackbar_canny_low(int val, void* userdata) {
    WaveDetection* node = static_cast<WaveDetection*>(userdata);
    node->canny_low_threshold_ = val;
    RCLCPP_INFO(node->get_logger(), "Canny Low Threshold updated to: %d", val);
}

void WaveDetection::on_trackbar_canny_high(int val, void* userdata) {
    WaveDetection* node = static_cast<WaveDetection*>(userdata);
    node->canny_high_threshold_ = val;
    RCLCPP_INFO(node->get_logger(), "Canny High Threshold updated to: %d", val);
}