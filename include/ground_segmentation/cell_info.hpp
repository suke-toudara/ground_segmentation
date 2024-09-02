#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>

#define MIN_PROBABILITY 0.1f
#define MAX_PROBABILITY 1.0f
#define UNKNOWN_PROBABILITY 0.5f

/*

*/
class CellInfo
{
public:
    CellInfo()
    {
        delta_z_m_ = 0.0f;
        occupancy_probability_ = UNKNOWN_PROBABILITY;
        danger_likelihood_ = 0.0f;
        safe_likelihood_ = 0.0f;
        height_threshold_ = 0.0f;
    }

    void updateCellInfo(float z_m)
    {
        calculateDeltaZM(z_m);
    }

    void updateOccupancyProbability()
    {
        float prior, marginal_likelihood_, posterior;
        prior = occupancy_probability_;
        marginal_likelihood_ = (danger_likelihood_ * prior) + (safe_likelihood_ * (1.0f - prior));
        if (delta_z_m_ > height_threshold_)
        {
            occupancy_probability_ = (danger_likelihood_ * prior) / marginal_likelihood_;
        }
        else if (delta_z_m_ > 0 && delta_z_m_ <= height_threshold_)
        {
            occupancy_probability_ = (safe_likelihood_ * prior) / marginal_likelihood_;
        }
        else {} // do nothing

        if (occupancy_probability_ < MIN_PROBABILITY) { occupancy_probability_ = MIN_PROBABILITY; }
        else if (occupancy_probability_ > MAX_PROBABILITY) { occupancy_probability_ = MAX_PROBABILITY; }
        else {} // do nothing
    }

    uint8_t occupancyProbability() { return static_cast<uint8_t>(occupancy_probability_ * 100.0f); }

    void setParameters(float danger_likelihood, float safe_likelihood, float height_threshold)
    {
        danger_likelihood_ = danger_likelihood;
        safe_likelihood_ = safe_likelihood;
        height_threshold_ = height_threshold;
    }

private:
    void calculateDeltaZM(float z_m)
    {
        uint8_t i_1, i_2;
        z_vector_.push_back(z_m);
        if (z_vector_.size() >= 2)
        {
            i_1 = z_vector_.size() - 1;
            i_2 = i_1 - 1;
            delta_z_m_ = std::abs(z_vector_[i_1] - z_vector_[i_2]);
            z_vector_.erase(z_vector_.begin());
        }
    }       

    std::vector<float> z_vector_;
    float delta_z_m_;
    float occupancy_probability_;
    float danger_likelihood_, safe_likelihood_;
    float height_threshold_;
};







