#include "custom_global_planner/custom_global_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <nav2_util/node_utils.hpp> 
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "custom_global_planner/a_star.hpp"
#include "custom_global_planner/dijkstra.hpp"
#include "custom_global_planner/pso.hpp"
#include "custom_global_planner/aco.hpp"



namespace custom_global_planner
{

    void CustomGlobalPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf, 
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        planner_name_ = name;
        tf_ = tf;
        
        auto node = parent.lock();
        if (!node) {
            throw std::runtime_error("Unable to lock node!");
        }
        
        nav2_util::declare_parameter_if_not_declared(
            node, name + ".planner_type", rclcpp::ParameterValue("A_STAR"));
        nav2_util::declare_parameter_if_not_declared(
            node, name + ".allow_unknown", rclcpp::ParameterValue(true));
        nav2_util::declare_parameter_if_not_declared(
            node, name + ".default_tolerance", rclcpp::ParameterValue(0.5));
    
        std::string planner_type_str;
        node->get_parameter(name + ".planner_type", planner_type_str);
        node->get_parameter(name + ".allow_unknown", allow_unknown_);
        node->get_parameter(name + ".default_tolerance", default_tolerance_);

        if (planner_type_str == "DIJKSTRA") {
            planner_type_ = PlannerType::DIJKSTRA;
            RCLCPP_INFO(logger_, "Using Dijkstra algorithm");
        } 
        else if(planner_type_str == "PSO")
        {
            planner_type_ = PlannerType::PSO;
            RCLCPP_INFO(logger_, "Using PSO algorithm");
        }
        else if(planner_type_str == "ACO")
        {
            planner_type_ = PlannerType::ACO;
            RCLCPP_INFO(logger_, "Using ACO algorithm");
        }
        else {
            planner_type_ = PlannerType::A_STAR;
            RCLCPP_INFO(logger_, "Using A* algorithm");
        }

        // Create planner instance
        planner_ = createPlannerInstance();
  
        RCLCPP_INFO(logger_, "Configured plugin %s", planner_name_.c_str());
    }

    void CustomGlobalPlanner::cleanup()
    {
        RCLCPP_INFO(logger_, "Cleaning up plugin %s", planner_name_.c_str());
        planner_.reset();
    }

    void CustomGlobalPlanner::activate()
    {
        RCLCPP_INFO(logger_, "Activating plugin %s", planner_name_.c_str());
    }

    void CustomGlobalPlanner::deactivate()
    {
        RCLCPP_INFO(logger_, "Deactivating plugin %s", planner_name_.c_str());
    }

    std::unique_ptr<AlgorithmInterface> CustomGlobalPlanner::createPlannerInstance()
    {
        if (planner_type_ == PlannerType::DIJKSTRA) {
            return std::make_unique<Dijkstra>(costmap_, allow_unknown_);
            // return;
        } 
        else if (planner_type_ == PlannerType::PSO) 
        {
            return std::make_unique<PSO>(costmap_, allow_unknown_);
            // return;
        }
        else if (planner_type_ == PlannerType::ACO) 
        {
            return std::make_unique<ACO>(costmap_, allow_unknown_);
            // return;
        }
        else {
            return std::make_unique<AStar>(costmap_, allow_unknown_);
        }
    }

    nav_msgs::msg::Path CustomGlobalPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path global_path;

        // Check if start and goal frames match global frame
        if (start.header.frame_id != global_frame_) 
        {
            RCLCPP_ERROR(logger_, "Start pose frame %s does not match global frame %s", 
                        start.header.frame_id.c_str(), global_frame_.c_str());
            return global_path;
        }
        
        if (goal.header.frame_id != global_frame_) 
        {
            RCLCPP_ERROR(logger_, "Goal pose frame %s does not match global frame %s", 
                        goal.header.frame_id.c_str(), global_frame_.c_str());
            return global_path;
        }

        // Initialize the path
        auto node = node_.lock();
        if (!node) {
            RCLCPP_ERROR(logger_, "Failed to lock node in createPlan");
            return global_path;
        }
        
        global_path.header.stamp = node->now();
        global_path.header.frame_id = global_frame_;

        // Calculate the plan
        if (!planner_->makePlan(start, goal, global_path)) {
            RCLCPP_WARN(logger_, "Planning algorithm %s failed to create a path!", planner_name_.c_str());
            return global_path;
        }
        
        // Check if path is empty
        if (global_path.poses.empty()) {
            RCLCPP_WARN(logger_, "Planning algorithm %s returned an empty path!", planner_name_.c_str());
        }

        return global_path;
    }
}
PLUGINLIB_EXPORT_CLASS(custom_global_planner::CustomGlobalPlanner, nav2_core::GlobalPlanner)    // Register this planner as a nav2_core::GlobalPlanner plugin