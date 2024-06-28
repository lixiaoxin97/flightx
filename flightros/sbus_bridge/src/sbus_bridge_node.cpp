#include "sbus_bridge/sbus_bridge.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sbus_bridge::SbusBridge>());
  rclcpp::shutdown();
  return 0;
}
