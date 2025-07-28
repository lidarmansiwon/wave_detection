#include "wave_detection/wave_detection.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<WaveDetection>());
//   rclcpp::shutdown();
//   return 0;
// }
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor; // 스레드를 사용하여 콜백 처리 (필요시)
  rclcpp::NodeOptions options;
  auto node = std::make_shared<WaveDetection>(options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}