#ifndef CUSTOM_GLOBAL_PLANNER__PSO_HPP_
#define CUSTOM_GLOBAL_PLANNER__PSO_HPP_

#include <vector>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "custom_global_planner/custom_global_planner.hpp"

namespace custom_global_planner
{
    class PSO : public AlgorithmInterface
    {
        public:
            PSO(nav2_costmap_2d::Costmap2D * costmap, bool allow_unknown);
            ~PSO() override = default;

            bool makePlan(const geometry_msgs::msg::PoseStamped & start, 
                const geometry_msgs::msg::PoseStamped & goal, 
                nav_msgs::msg::Path & plan) override;
            std::string getName() override;

        private:
            geometry_msgs::msg::PoseStamped start, goal;
            nav_msgs::msg::Path path;
    };
}
#endif