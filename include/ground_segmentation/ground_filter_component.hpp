#ifndef POINT_CLOUD_DUST_FILTER_HPP
#define POINT_CLOUD_DUST_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ground_filter
{
class PointCloudDustFilter : public rclcpp::Node
{
    public:
        explicit PointCloudDustFilter(const rclcpp::NodeOptions & options);
    private:
        void loadParameters();
        void interpretPointCloudXYZ(uint32_t index, float &x_m, float &y_m, float &z_m);
        bool isDustPointCloud(uint8_t probability);
        void updateHeightInfo();
        void updateOccupancyProbability();
        void filterDustPointCloud();
        void publishCostMap();
        void publishPointCloud();

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dust_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr without_dust_publisher_;

        sensor_msgs::msg::PointCloud2 input_point_cloud_;
        

        float min_dust_probability_;
        float max_dust_probability_;

        CostMap cost_map_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

}
}
