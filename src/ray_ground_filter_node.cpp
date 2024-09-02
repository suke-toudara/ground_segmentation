#include <memory>
#include <ground_segmentation/ray_ground_filter_component.cpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ground_segmentation::RayGroundFilterComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}