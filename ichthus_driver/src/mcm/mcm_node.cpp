#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "ichthus_driver/mcm_management.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ichthus::IchthusCANMCMManager>(
    rclcpp::NodeOptions()));

  rclcpp::shutdown();
  return 0;
}
