#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "ichthus_driver/kia_reader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ichthus::IchthusCANKIAReader>(
    rclcpp::NodeOptions()));

  rclcpp::shutdown();
  return 0;
}
