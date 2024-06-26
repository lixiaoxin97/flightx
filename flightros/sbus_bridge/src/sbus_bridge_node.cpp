#include "sbus_bridge/sbus_bridge.h"

int main(int argc, char * argv[]) // execute the node
{
  rclcpp::init(argc, argv); //  initialize ROS 2
  rclcpp::spin(std::make_shared<sbus_bridge::SbusBridge>()); //  starts processing data from the node
  rclcpp::shutdown(); //  shutdown the node
  return 0;
}
