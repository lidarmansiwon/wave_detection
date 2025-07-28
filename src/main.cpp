#include "wave_detection/wave_detection.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaveDetection>());
  rclcpp::shutdown();
  return 0;
}