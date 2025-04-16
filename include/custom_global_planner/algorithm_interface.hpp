#ifndef CUSTOM_GLOBAL_PLANNER__ALGORITHM_INTERFACE_HPP_
#define CUSTOM_GLOBAL_PLANNER__ALGORITHM_INTERFACE_HPP_

#include <vector>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace custom_global_planner
{
    class AlgorithmInterface
    {
        public:
            AlgorithmInterface(nav2_costmap_2d::Costmap2D * costmap, bool allow_unknown):
            costmap_(costmap), allow_unknown_(allow_unknown)
            {

            }
            virtual ~AlgorithmInterface(){}  

            virtual bool makePlan(const geometry_msgs::msg::PoseStamped & start, 
                const geometry_msgs::msg::PoseStamped & goal,
                nav_msgs::msg::Path & plan) =0;

            virtual std::string getName() =0;

        protected:
            nav2_costmap_2d::Costmap2D * costmap_;
            bool allow_unknown_;
    };
}

#endif