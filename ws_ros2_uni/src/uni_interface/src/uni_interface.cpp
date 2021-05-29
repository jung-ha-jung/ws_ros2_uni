#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uni_interface/ros2_uni.h"

using namespace std::chrono_literals;

// ------------------------------------------------------------------
/// @param
/// @return 성공 여부(0=성공)
/// @brief  main 함수
// ------------------------------------------------------------------
int main(int argc, char * argv[])
{
  //Override the default ros sigint handler.
  //This must be set after the first NodeHandle is created.
  //signal(SIGINT, mySigintHandler);

  //uni_interface_node.run();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_uni>());
  rclcpp::shutdown();

  return(0);
}
