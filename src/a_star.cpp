#include "custom_global_planner/a_star.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nav2_costmap_2d/cost_values.hpp>

namespace custom_global_planner
{

    AStar::AStar(nav2_costmap_2d::Costmap2D * costmap, bool allow_unknown)
    : AlgorithmInterface(costmap, allow_unknown)
    {
    }

    std::string AStar::getName()
    {
        return "A_STAR";
    }

    bool AStar::makePlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal, nav_msgs::msg::Path & path)
    {
        // Initialize path
        path.poses.clear();
        
        if (!costmap_) 
        {
            RCLCPP_ERROR(logger_, "Costmap is null");
            return false;
        }

        unsigned int start_x, start_y;
        if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) 
        {
            RCLCPP_ERROR(
            logger_, "Start pose (%.2f, %.2f) is outside the map boundaries",
            start.pose.position.x, start.pose.position.y);
            return false;
        }
        
        unsigned int goal_x, goal_y;
        if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) 
        {
            RCLCPP_ERROR(
            logger_, "Goal pose (%.2f, %.2f) is outside the map boundaries",
            goal.pose.position.x, goal.pose.position.y);
            return false;
        }
        
        // Check if start and goal are the same
        if (start_x == goal_x && start_y == goal_y) 
        {
            RCLCPP_WARN(logger_, "Start and goal are at the same position");
            
            // Create a path with just the goal
            geometry_msgs::msg::PoseStamped pose = goal;
            pose.header.stamp = rclcpp::Clock().now();
            path.poses.push_back(pose);
            return true;
        }
        
        // Check if start or goal is in collision
        if (getCost(start_x, start_y) >= LETHAL_OBSTACLE) \
        {
            RCLCPP_WARN(logger_, "Start position is in collision");
            return false;
        }
        
        if (getCost(goal_x, goal_y) >= LETHAL_OBSTACLE) 
        {
            RCLCPP_WARN(logger_, "Goal position is in collision");
            return false;
        }
        
        // Initialize A* data structures
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
        std::unordered_map<int, Node> nodes;
        std::unordered_map<int, bool> closed_list;
        
        // Create start node
        int start_index = toIndex(start_x, start_y);
        Node start_node{start_index, start_x, start_y, 0.0,
            getHeuristicDistance(start_x, start_y, goal_x, goal_y), 0.0, -1};
        start_node.f_cost = start_node.g_cost + start_node.h_cost;
        
        // Add start node to open list and nodes map
        open_list.push(start_node);
        nodes[start_index] = start_node;
        
        // A* search
        while (!open_list.empty()) 
        {
            // Get node with lowest f_cost from open list
            Node current = open_list.top();
            open_list.pop();
            
            // Convert to index
            int current_index = toIndex(current.x, current.y);
            
            // Skip if already in closed list
            if (closed_list.find(current_index) != closed_list.end()) 
            {
                continue;
            }
            
            // Add to closed list
            closed_list[current_index] = true;
            
            // Check if reached goal (within tolerance)
            if (std::hypot(current.x - goal_x, current.y - goal_y) == 0.0) 
            {
                // Construct path
                constructPath(current, nodes, path);
                return true;
            }
            
            // Explore neighbors
            for (int i = 0; i < 8; ++i) 
            {
                int nx = current.x + DIRECTIONS[i][0];
                int ny = current.y + DIRECTIONS[i][1];
                int neighbor_index = toIndex(nx, ny);
                
                // Skip if not valid or already in closed list
                if (!isValid(nx, ny) || closed_list.find(neighbor_index) != closed_list.end()) 
                {
                    continue;
                }
                
                // Calculate new g cost
                double move_cost = DIRECTION_COSTS[i];
                double cell_cost = getCost(nx, ny);
                
                // Skip if lethal obstacle
                if (cell_cost >= LETHAL_OBSTACLE) 
                {
                    continue;
                }
                
                // Apply cost factor for non-lethal costs
                if (cell_cost > COST_NEUTRAL) 
                {
                    move_cost += COST_FACTOR * cell_cost;
                }
                
                double new_g_cost = current.g_cost + move_cost;
                
                // Check if new path is better
                if (nodes.find(neighbor_index) == nodes.end() || new_g_cost < nodes[neighbor_index].g_cost) 
                {
                    // Create new node
                    double h_cost = getHeuristicDistance(nx, ny, goal_x, goal_y);
                    Node new_node{neighbor_index, nx, ny, new_g_cost, h_cost, new_g_cost + h_cost, current_index};
                    
                    // Update nodes map
                    nodes[neighbor_index] = new_node;
                    
                    // Add to open list
                    open_list.push(new_node);
                }
            }
        }
        
        // No path found
        RCLCPP_WARN(logger_, "No path found from start to goal");
        return false;
    }

    double AStar::getHeuristicDistance(int x1, int y1, int x2, int y2)
    {
    // Euclidean distance
        return std::hypot(x2 - x1, y2 - y1);
    }

    bool AStar::isValid(int x, int y)
    {
        return x >= 0 && x < static_cast<int>(costmap_->getSizeInCellsX()) &&
            y >= 0 && y < static_cast<int>(costmap_->getSizeInCellsY());
    }

    double AStar::getCost(int x, int y)
    {
        unsigned char cost = costmap_->getCost(x, y);
        
        // If planning through unknown space is allowed
        if (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_) 
        {
            return COST_NEUTRAL;
        }
        
        return static_cast<double>(cost);
    }

    void AStar::constructPath(const Node & current, const std::unordered_map<int, Node> & nodes, nav_msgs::msg::Path & path)
    {
        // Build path from goal to start
        std::vector<geometry_msgs::msg::PoseStamped> reversed_path;
        
        // Add goal
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = path.header.frame_id;
        pose.header.stamp = path.header.stamp;
        
        // Start with current (goal) node
        int current_index = current.index;
        
        // Trace back to start
        while (current_index != -1) 
        {
            const Node & node = nodes.at(current_index);
            
            // Convert map coordinates to world coordinates
            double world_x, world_y;
            mapToWorld(node.x, node.y, world_x, world_y);
            
            // Create pose
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            reversed_path.push_back(pose);
            
            // Move to parent
            current_index = node.parent_index;
        }
        
        // Reverse path to get start-to-goal order
        path.poses.resize(reversed_path.size());
        for (size_t i = 0; i < reversed_path.size(); ++i) 
        {
            path.poses[i] = reversed_path[reversed_path.size() - 1 - i];
        }
    }

    bool AStar::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
    {
        return costmap_->worldToMap(wx, wy, mx, my);
    }

    void AStar::mapToWorld(int mx, int my, double & wx, double & wy)
    {
        costmap_->mapToWorld(mx, my, wx, wy);
    }

    int AStar::toIndex(int x, int y)
    {
        return y * costmap_->getSizeInCellsX() + x;
    }

}  // namespace custom_global_planner