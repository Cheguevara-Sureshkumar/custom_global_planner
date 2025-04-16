#include "custom_global_planner/dijkstra.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nav2_costmap_2d/cost_values.hpp>

namespace custom_global_planner
{
    using Node = custom_global_planner::dijkstra::Node;
    
    Dijkstra::Dijkstra(nav2_costmap_2d::Costmap2D * costmap, bool allow_unknown)
    : AlgorithmInterface(costmap, allow_unknown)
    {
        unsigned int size_x = costmap_->getSizeInCellsX();
        unsigned int size_y = costmap_->getSizeInCellsY();

        cost.resize(size_x * size_y, std::numeric_limits<float>::infinity());
        RCLCPP_INFO(logger_, "All cost is set to infinity");
    }

    std::string Dijkstra::getName()
    {
        return "DIJKSTRA";
    }

    bool Dijkstra::makePlan(const geometry_msgs::msg::PoseStamped & start, 
        const geometry_msgs::msg::PoseStamped & goal,
        nav_msgs::msg::Path & path)
    {
        RCLCPP_INFO(logger_, "Path is being planned.");
        path.poses.clear();         //to clear the previous path or intialise a empty path

        if(!costmap_)
        {
            RCLCPP_ERROR(logger_, "Costmap is null");
            return false;
        }

        unsigned int start_x, start_y;
        if(!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y))
        {
            RCLCPP_ERROR(logger_, "Start pose (%.2f, %.2f) is outside the map boundaries",start.pose.position.x, start.pose.position.y);
            return false;
        }

        unsigned int goal_x, goal_y;
        if(!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
        {
            RCLCPP_ERROR(logger_, "Goal pose (%.2f, %.2f) is outside the map boundaries",goal.pose.position.x, goal.pose.position.y);
            return false;
        }

        if(start_x == goal_x &&  start_y == goal_y)
        {
            RCLCPP_WARN(logger_, "Start and goal are at the same position");

            geometry_msgs::msg::PoseStamped pose = goal;
            pose.header.stamp = rclcpp::Clock().now();
            path.poses.push_back(pose);
            return true;
        }
        
        RCLCPP_INFO(logger_, "Start pose is (%.2f, %.2f) within the map.", start.pose.position.x, start.pose.position.y);
        RCLCPP_INFO(logger_, "Goal pose is (%.2f, %.2f) within the map.", goal.pose.position.x, goal.pose.position.y);

        std::fill(cost.begin(), cost.end(), std::numeric_limits<float>::infinity());

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> node_list;
        std::unordered_map<int, Node> nodes;
        int start_idx = toIndex(start_x, start_y);
        cost[start_idx] = 0.0;
        Node start_node{start_idx, start_x, start_y, cost[start_idx], -1};
        node_list.push(start_node);
        nodes[start_idx] = start_node;

        int goal_idx = toIndex(goal_x, goal_y);
        // cost[goal_idx] = std::numeric_limits<float>::infinity;     need to find the reason between not compatible.
        // cost[goal_idx] = INFINITY;

        Node goal_node{goal_idx, goal_x, goal_y, cost[goal_idx], -1};


        if(getCost(start_idx)>=LETHAL_COST || getCost(goal_idx)>=LETHAL_COST)
        {
            RCLCPP_WARN(logger_, "Start or goal are at the obstacle");
            return false;
        }

        while(!node_list.empty())
        {
            Node current = node_list.top();
            node_list.pop();

            if(current.idx == goal_node.idx)
            {
                reconstructPath(current, nodes, path);
                RCLCPP_INFO(logger_, "Path is found.");
                return true;
            }

            std::vector<Node> neighbors = findNeighbors(current);
            RCLCPP_INFO(logger_, "Found %zu neighbors for node at (%u, %u)", neighbors.size(), current.x, current.y);
            if(neighbors.empty())
            {
                RCLCPP_WARN(logger_, "No neighbors found.");
            }
            for(auto n : neighbors)
            {
                node_list.push(n);
                nodes[n.idx] = n;
            }    
            // current = node_list.top();
            
        }
        std::fill(cost.begin(), cost.end(), std::numeric_limits<float>::infinity());
        RCLCPP_INFO(logger_, "No Path is found.");
        return false;
    }

    bool Dijkstra::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
    {
        return costmap_->worldToMap(wx, wy, mx, my);
    }

    void Dijkstra::maptoWorld(unsigned int mx, unsigned int my, double & wx, double & wy)
    {
        costmap_->mapToWorld(mx, my, wx, wy);
    }

    int Dijkstra::toIndex(int x, int y)
    {
        return y * costmap_->getSizeInCellsX() + x;
    }

    std::vector<Node> Dijkstra::findNeighbors(Node current)
    {
        RCLCPP_INFO(logger_, "Finding neighbors for node at (%u, %u) with cost %.2f", current.x, current.y, current.cost);
        std::vector<Node> neighbors;
        for(int i=0; i<8 ; ++i)                 //Both Pre-increment and post-increment functionality is same, but the pre-increment is little more efficient
        {
            int nx = current.x + DIRECTIONS[i][0];
            int ny = current.y + DIRECTIONS[i][1];

            if (!isValid(nx, ny))
            {
                RCLCPP_INFO(logger_, "Neighbor at (%d, %d) is invalid (outside map)", nx, ny);
                continue;
            }

            unsigned int n_idx = toIndex(nx, ny);
            double n_cost = getCellCost(i, n_idx);

            RCLCPP_INFO(logger_, "Raw cell cost ( %.2f)", n_cost);
            if (n_cost >= LETHAL_COST)
            {
                RCLCPP_INFO(logger_, "Neighbor at (%d, %d) is an obstacle (cost %.2f)", nx, ny, n_cost);
                continue;
            }

            float total_cost = current.cost + n_cost;
            

            if(total_cost < cost[n_idx])
            {
                cost[n_idx] = total_cost;
                Node neighbor;
                neighbor.x = nx;
                neighbor.y = ny;
                neighbor.idx = n_idx;
                neighbor.cost = total_cost;
                neighbor.prev_idx = current.idx;
                neighbors.push_back(neighbor);
                RCLCPP_INFO(logger_, "Adding neighbor at (%d, %d) with cost %.2f", nx, ny, total_cost);
            }
            else
            {
                RCLCPP_INFO(logger_, "Can't add neighbor at (%d, %d) with cost %.2f and current cost %.2f", nx, ny, total_cost, cost[n_idx]);
            }
        }
        RCLCPP_INFO(logger_, "Number of neighbors : (%zu)", neighbors.size());
        return neighbors;
    }

    double Dijkstra::getCellCost(int direction, unsigned int idx)
    {
        double move_cost = DIRECTIONS_COST[direction];
        double cell_cost = getCost(idx);

        return move_cost + cell_cost;
    }

    bool Dijkstra::isValid(int x, int y)
    {
        RCLCPP_INFO(logger_, "Checking if (%d, %d) is valid. Map size: %d x %d", x, y, costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
        if (static_cast<int>(costmap_->getSizeInCellsX()) > x && x >= 0 && y >= 0 && static_cast<int>(costmap_->getSizeInCellsY()) >y)
        {
            return true;
        }
        return false;
    }

    double Dijkstra::getCost(unsigned int idx)
    {
        unsigned char cost = costmap_->getCost(idx);
        if (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_) 
        {
            return 50.0;
        }
        return static_cast<double> (cost);
    }

    void Dijkstra::reconstructPath(const Node & current, const std::unordered_map<int, Node> & nodes, nav_msgs::msg::Path & path)
    {
        std::vector<geometry_msgs::msg::PoseStamped> reversed_path;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = path.header.frame_id;
        pose.header.stamp = path.header.stamp;

        int c_idx = current.idx;

        while(c_idx != -1)
        {
            const Node & node = nodes.at(c_idx);

            double world_x, world_y;
            maptoWorld(node.x, node.y, world_x, world_y);

            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            reversed_path.push_back(pose);

            c_idx = node.prev_idx;
        }

        path.poses.resize(reversed_path.size());
        for (size_t i = 0; i < reversed_path.size(); ++i) 
        {
            path.poses[i] = reversed_path[reversed_path.size() - 1 - i];
        }
    }
}