

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "ground_filter_component.hpp"



namespace ground_filter
{
PointCloudDustFilter::PointCloudDustFilter(const rclcpp::NodeOptions & options)
: Node("pointcloud_transformer"),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(*tf_buffer_)
{  
    cost_map_ = CostMap(node);
    dust_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("iv_dust", 10);
    without_dust_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("iv_without_dust", 10);
    loadParameters(node);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud_topic", 10,
        std::bind(&PointCloudTransformer::pointcloud_callback, this, std::placeholders::_1));
}

void PointCloudDustFilter::loadParameters()
{   
    declare_parameter("min_dust_probability",min_dust_probability_);
    declare_parameter("max_dust_probability",max_dust_probability_);
    const auto min_dust_probability_ = get_parameter("min_dust_probability").as_string();
    const auto max_dust_probability_ = get_parameter("max_dust_probability").as_string();
}

PointCloudDustFilter:pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
            "base_link", msg->header.frame_id, tf2::TimePointZero);
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        tf2::doTransform(*msg, transformed_cloud, transform_stamped);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
    }
    updateHeightInfo();
    filterDustPointCloud();
    filterDustPointCloud();
    publishCostMap();
    publishPointCloud();
}

void PointCloudDustFilter::filterDustPointCloud(pcl::PointCloud<pcl::PointXYZRGB> dust_pcl ,pcl::PointCloud<pcl::PointXYZRGB> without_dust_pcl)
{
    for (uint32_t index = 0; index < input_pcl_.size(); index++)
    {   
        //対応するインデックスの点群を取得
        pcl::PointXYZRGB point;
        point.x = input_pcl_.points[index].x;
        point.y = input_pcl_.points[index].y;
        point.z = input_pcl_.points[index].z;
        uint8_t probability = cost_map_.occupancyProbability(point.x,point.y);
        if (probability > min_dust_probability_ && probability <= max_dust_probability_)
        {
            point.r = 255, point.g = 0, point.b = 0;
            dust_pcl.push_back(point);
        }
        else
        {
            point.r = 0, point.g = 255, point.b = 0;
            without_dust_pcl.push_back(point);
        }
    }

    //publish filter_point_cloud
    pcl::PointCloud<pcl::PointXYZ> input_pcl_;
    pcl::PointCloud<pcl::PointXYZRGB> dust_ros_, without_dust_ros_;
    pcl::toROSMsg(dust_pcl, dust_ros_);
    pcl::toROSMsg(without_dust_pcl, without_dust_ros_);
    dust_ros_.header.stamp = rclcpp::Clock().now();
    dust_ros_.header.frame_id = "base_link";
    dust_publisher_->publish(dust_ros_);

    without_dust_ros_.header.stamp = rclcpp::Clock().now();
    without_dust_ros_.header.frame_id = "base_link";
    without_dust_publisher_->publish(without_dust_ros_);
}

void PointCloudDustFilter::updateHeightInfo()
{
    pcl::fromROSMsg(input_point_cloud_, input_pcl_);
    for (uint32_t index = 0; index< input_pcl_.size(); index++)
    {
        point.x = input_pcl_.points[index].x;
        point.y = input_pcl_.points[index].y;
        point.z = input_pcl_.points[index].z;
        cost_map_.updateHeightInfoInCell(point.x,point.y,point.z);
    }
}

void PointCloudDustFilter::updateOccupancyProbability()
{
    cost_map_.updateOccupancyProbability();
}

void PointCloudDustFilter::publishCostMap()
{
    cost_map_.publish();
}

};

