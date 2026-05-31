#include "james_perception/bottle_detector.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<james_perception::BottleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
