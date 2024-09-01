#include <memory>
#include <ground_filter/ground_filter_component.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ground_filter::Component>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}