#ifndef CUSTOM_GLOBAL_PLANNER__DIJKSTRA_HPP_
#define CUSTOM_GLOBAL_PLANNER__DIJKSTRA_HPP_

#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "custom_global_planner/algorithm_interface.hpp"

namespace custom_global_planner
{
    namespace dijkstra
    {
        struct Node
        {
            int idx;
            unsigned int x, y;
            float cost;
            int prev_idx;

            bool operator>(const Node & other) const
            {
                return cost > other.cost;  
            }
        };

    }
    class Dijkstra : public AlgorithmInterface
    {
        private:
            std::vector<float> cost;
            using Node = custom_global_planner::dijkstra::Node;

            rclcpp::Logger logger_{rclcpp::get_logger("Dijkstra")};

            const int DIRECTIONS[8][2] = {{0,1}, {1,1}, {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1, 1}};
            const float DIRECTIONS_COST[8] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};
            const double LETHAL_COST = 255.0;

            bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
            void maptoWorld(unsigned int mx, unsigned int my, double & wx, double & wy);
            int toIndex(int x, int y);
            std::vector<Node> findNeighbors(Node current);
            bool isValid(int x, int y);
            double getCost(unsigned int idx);
            double getCellCost(int direction, unsigned int idx);
            void reconstructPath(const Node & current, const std::unordered_map<int, Node> & nodes, nav_msgs::msg::Path & path);

        public:
            Dijkstra(nav2_costmap_2d::Costmap2D * costmap, bool allow_unknown);

            ~Dijkstra() override = default;

            std::string getName() override;
            bool makePlan(const geometry_msgs::msg::PoseStamped & start, 
                    const geometry_msgs::msg::PoseStamped & goal,
                nav_msgs::msg::Path & plan) override;

    };
}
#endif