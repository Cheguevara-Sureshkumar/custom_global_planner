#include "custom_global_planner/pso.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace custom_global_planner
{
    PSO::PSO(nav2_costmap_2d::Costmap2D * costmap, bool allow_unknown)
    : AlgorithmInterface(costmap, allow_unknown){};
    std::string PSO::getName()
    {
        return "PSO";
    }
    bool PSO::makePlan(const geometry_msgs::msg::PoseStamped & start, 
        const geometry_msgs::msg::PoseStamped & goal, 
        nav_msgs::msg::Path & path)
    {
        
        return false;
    }
}