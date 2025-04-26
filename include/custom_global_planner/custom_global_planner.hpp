#ifndef CUSTOM_GLOBAL_PLANNER__CUSTOM_GLOBAL_PLANNER_HPP_
#define CUSTOM_GLOBAL_PLANNER__CUSTOM_GLOBAL_PLANNER_HPP_
#include <string>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include "custom_global_planner/algorithm_interface.hpp"
 
namespace custom_global_planner
{
    class CustomGlobalPlanner : public nav2_core::GlobalPlanner
    {
        public:
            CustomGlobalPlanner() = default;
            ~CustomGlobalPlanner() override = default;
            
            void configure(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                std::string name, 
                std::shared_ptr<tf2_ros::Buffer> tf,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
                
            void cleanup() override ;
            void activate() override ;
            void deactivate() override ;
            
            nav_msgs::msg::Path createPlan(
                const geometry_msgs::msg::PoseStamped & start, 
                const geometry_msgs::msg::PoseStamped & goal) override;
                
        private:
            // Changed from SharedPtr to WeakPtr to match base class
            rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
            nav2_costmap_2d::Costmap2D * costmap_;
            std::shared_ptr<tf2_ros::Buffer> tf_;
            
            std::unique_ptr<AlgorithmInterface> createPlannerInstance();
            std::unique_ptr<AlgorithmInterface> planner_;
            enum class PlannerType{ A_STAR, DIJKSTRA, PSO, ACO};
            rclcpp::Logger logger_{rclcpp::get_logger("CustomGlobalPlanner")};  // Initialize logger
            PlannerType planner_type_{PlannerType::A_STAR};  // Initialize with default
            std::string planner_name_;
            std::string global_frame_;
            bool allow_unknown_{true};  // Initialize with default
            double default_tolerance_{0.5};  // Added tolerance parameter with default
    };
}
#endif