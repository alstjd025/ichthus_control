#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "pid_control/pid_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ichthus::PIDController>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
