#ifndef COST_MAP_HPP
#define COST_MAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "cell_info.hpp"
#include "grid_map_utility.hpp"

class CostMap : public rclcpp::Node
{
public:
    CostMap() : Node("cost_map")
    {   
        config_publisher_ = create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", 10);
        map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

        this->declare_parameter("resolution_m", 0.05);
        this->declare_parameter("width_m", 10.0);
        this->declare_parameter("height_m", 10.0);
        this->declare_parameter("center_x_m", 0.0);
        this->declare_parameter("center_y_m", 0.0);
        this->declare_parameter("danger_likelihood", 0.8);
        this->declare_parameter("safe_likelihood", 0.2);
        this->declare_parameter("height_threshold", 1.0);

        this->get_parameter("resolution_m", resolution_m_);
        this->get_parameter("width_m", width_m_);
        this->get_parameter("height_m", height_m_);
        this->get_parameter("center_x_m", center_x_m_);
        this->get_parameter("center_y_m", center_y_m_);
        this->get_parameter("danger_likelihood", danger_likelihood_);
        this->get_parameter("safe_likelihood", safe_likelihood_);
        this->get_parameter("height_threshold", height_threshold_);

        config_.resolution = resolution_m_;
        config_.height = static_cast<uint32_t>(height_m_/resolution_m_);
        config_.width = static_cast<uint32_t>(width_m_/resolution_m_);
        all_cell_num_ = config_.width * config_.height;

        config_.origin.position.x = center_x_m_ - (width_m_/2.0);
        config_.origin.position.y = center_y_m_ - (height_m_/2.0);
        config_.origin.position.z = 0.0;

        config_.origin.orientation.x = 0.0;
        config_.origin.orientation.y = 0.0;
        config_.origin.orientation.z = 0.0;
        config_.origin.orientation.w = 1.0;
        
        original_map.resize(config_.width * config_.height);

        publish_map_.header.frame_id = "map";
        for (uint32_t i = 0; i < all_cell_num_; i++)
        {
            publish_map_.data.push_back(static_cast<uint8_t>(UNKNOWN_PROBABILITY*100.0f));
            original_map[i].setParameters(danger_likelihood_, safe_likelihood_, height_threshold_);
        }
    }

    void updateHeightInfoInCell(float x_m, float y_m, float z_m)
    {
        uint32_t i_v;
        if (calculateAndValidateMapVectorIndex(x_m, y_m, i_v))
        {
            original_map[i_v].updateCellInfo(z_m);
        }
    }

    void updateOccupancyProbability()
    {
        for (uint32_t i_v = 0; i_v < all_cell_num_; i_v++)
        {
            original_map[i_v].updateOccupancyProbability();
            publish_map_.data[i_v] = original_map[i_v].occupancyProbability();
        }
    }

    uint32_t occupancyProbability(float x_m, float y_m)
    {
        uint32_t i_v;
        if (calculateAndValidateMapVectorIndex(x_m, y_m, i_v))
        {
            return original_map[i_v].occupancyProbability();
        }
        else // if index is invalid, return minimum probability
        { 
            return static_cast<uint8_t>(MIN_PROBABILITY * 100.0f);
        }
    }

    void publish()
    {
        publishConfig();
        publishMap();
        publishTransform();
    }

private:
    void publishConfig()
    {
        config_.map_load_time = this->now();
        config_publisher_->publish(config_);
    }

    void publishMap()
    {
        publish_map_.header.stamp = this->now();
        publish_map_.info = config_;
        map_publisher_->publish(publish_map_);
    }

    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "map";

        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, 0.0);
        transform_stamped.transform.rotation.x = quaternion.x();
        transform_stamped.transform.rotation.y = quaternion.y();
        transform_stamped.transform.rotation.z = quaternion.z();
        transform_stamped.transform.rotation.w = quaternion.w();
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    bool calculateAndValidateMapVectorIndex(float x_m, float y_m, uint32_t &index)
    {
        uint32_t i_x, i_y;
        i_x = getGridIndexX(x_m, config_.origin.position.x, config_.resolution);
        i_y = getGridIndexY(y_m, config_.origin.position.y, config_.resolution);
        index = getVectorIndexRowMajorOrder(i_x, i_y, config_.height);
        return isVectorIndexValid(index, all_cell_num_);
    }

    nav_msgs::msg::MapMetaData config_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr config_publisher_;

    nav_msgs::msg::OccupancyGrid publish_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    float resolution_m_, width_m_, height_m_;
    float center_x_m_, center_y_m_;
    uint32_t all_cell_num_;

    float danger_likelihood_, safe_likelihood_;
    float height_threshold_;

    std::vector<CellInfo> original_map;
};

#endif // COST_MAP_HPP
