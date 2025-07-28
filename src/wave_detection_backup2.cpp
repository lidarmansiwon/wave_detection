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
#include <cmath> // For std::abs, std::atan2, CV_PI

using std::placeholders::_1;
using namespace std::chrono_literals;

WaveDetection::WaveDetection(const rclcpp::NodeOptions & node_options)
: Node("wave_detection", node_options)

{
  this->declare_parameter<std::string>("image_topic", "/ouster/signal_image");
  this->declare_parameter<int>("wave_roi_start_row_percentage", 70); // 예시: 이미지 높이의 60%부터 파도 ROI 시작
  this->declare_parameter<int>("crest_threshold", 150); // 파고 임계값 (0-255)
  this->declare_parameter<int>("trough_threshold", 200);  // 파저 임계값 (0-255)

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

  // OpenCV 창과 트랙바 생성
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE); // resizable 창 생성
  cv::createTrackbar("ROI Start %", window_name_, &wave_roi_start_row_percentage_, 100, on_trackbar_roi, this);
  cv::createTrackbar("Crest Thresh", window_name_, &crest_threshold_, 255, on_trackbar_crest, this);
  cv::createTrackbar("Trough Thresh", window_name_, &trough_threshold_, 255, on_trackbar_trough, this);
  // 초기값으로 설정된 파라미터가 트랙바에 반영되도록 설정 (필수)
  cv::setTrackbarPos("ROI Start %", window_name_, wave_roi_start_row_percentage_);
  cv::setTrackbarPos("Crest Thresh", window_name_, crest_threshold_);
  cv::setTrackbarPos("Trough Thresh", window_name_, trough_threshold_);

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

  if (wave_roi_image.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Wave ROI image is empty. Check ROI parameters and input image dimensions.");
      return;
  }

  // 2. 파고(Wave Crest) 추출 (밝은 부분을 마스크)
  cv::Mat wave_crest_mask;
  // 파도 상단이 밝은 부분이라면 THRESH_BINARY를 사용합니다.
  cv::threshold(wave_roi_image, wave_crest_mask, crest_threshold_, 255, cv::THRESH_BINARY);

  // 시각화를 위한 이미지 준비
  cv::Mat display_image;
  cv::cvtColor(wave_roi_image, display_image, cv::COLOR_GRAY2BGR); // ROI를 컬러 이미지로 변환

  // -------------------------------------------------------------
  // 파고(Crest) 직선 추출 (허프 변환)
  // -------------------------------------------------------------
  cv::Mat canny_output_crest;
  // Canny 임계값도 조절 필요합니다. wave_crest_mask가 이진 이미지이므로
  // Canny는 사실상 엣지를 더 가늘게 만듭니다.
  cv::Canny(wave_crest_mask, canny_output_crest, 50, 150, 3); // Canny 엣지 검출 (minVal, maxVal 조정 필요)

  std::vector<cv::Vec4i> lines;

  // 확률적 허프 변환 적용
  // 초기 디버깅을 위해 HoughLinesP 파라미터를 좀 더 관대하게 설정합니다.
  // threshold를 낮게, minLineLength도 낮게, maxLineGap은 적절히.
  cv::HoughLinesP(canny_output_crest, lines, 1, CV_PI / 360,
                  30,    // threshold: 투표 수를 낮춰 더 많은 선 검출 시도
                  30,    // minLineLength: 최소 직선 길이를 줄여 짧은 선도 검출 시도
                  10);   // maxLineGap: 간격 허용

  

  // 검출된 모든 파고 직선들을 display_image에 그리기 (빨간색)
  // 각도 필터링은 일단 제거하여 모든 직선이 그려지는지 확인합니다.
  for (const auto& l : lines) {
      cv::line(display_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA); // 빨간색 선
  }

  // OpenCV 창에 실시간으로 표시
  cv::imshow(window_name_, display_image);
  cv::imshow("Wave Crests Mask", wave_crest_mask);
  cv::imshow("Canny Output Crest", canny_output_crest); // 이 창을 꼭 확인하세요! 엣지가 보이는지?
  cv::waitKey(1); // 1ms 대기, UI 업데이트

  // RViz2에 퍼블리시
  try {
      std_msgs::msg::Header header = image_data_->header;
      sensor_msgs::msg::Image::SharedPtr processed_msg = cv_bridge::CvImage(header, "bgr8", display_image).toImageMsg();
      processed_image_publisher_->publish(*processed_msg);

      sensor_msgs::msg::Image::SharedPtr crest_mask_msg = cv_bridge::CvImage(header, "mono8", wave_crest_mask).toImageMsg();
      wave_crest_image_publisher_->publish(*crest_mask_msg);

      // wave_trough_image_publisher_는 사용하지 않으므로 퍼블리시 코드도 주석 처리
      // sensor_msgs::msg::Image::SharedPtr trough_mask_msg = cv_bridge::CvImage(header, "mono8", wave_trough_mask).toImageMsg();
      // wave_trough_image_publisher_->publish(*trough_mask_msg);

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
