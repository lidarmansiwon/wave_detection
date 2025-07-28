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

using std::placeholders::_1;
using namespace std::chrono_literals;

WaveDetection::WaveDetection(const rclcpp::NodeOptions & node_options)
: Node("wave_detection", node_options)
{
  this->declare_parameter<std::string>("image_topic", "/ouster/signal_image");
  std::string image_topic  = this->get_parameter("image_topic").as_string();
  // 생성자에서 시작 표시.
  RCLCPP_INFO(this->get_logger(), "Run wave_detection");

  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(), std::bind(&WaveDetection::image_callback, this, _1));
  
  // 50ms --> 20Hz 
  timer_ = this->create_wall_timer(50ms, std::bind(&WaveDetection::process, this));
}

WaveDetection::~WaveDetection()
{

}

void WaveDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){image_data_ = msg;}

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

  if (!image_data_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "'image' data not received yet.");
    return;
  }
  

rclcpp::Time current_time = this->now();
double cal_time = current_time.seconds();
double systime = get_sysSec_from_ros_time(current_time); 

if (!start_time_set_) {
  start_time_sec_ = systime;
  start_time_set_ = true;
}

double time_since_start = systime - start_time_sec_;    // 상대 시간


}