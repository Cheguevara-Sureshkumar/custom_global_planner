#include "custom_global_planner/aco.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <random>
#include <limits>
#include <cmath>
#include <algorithm>

namespace custom_global_planner
{

ACO::ACO(nav2_costmap_2d::Costmap2D* costmap, bool allow_unknown)
: AlgorithmInterface(costmap, allow_unknown),
  num_ants_(30),
  max_iterations_(50),
  alpha_(1.0),      // pheromone importance
  beta_(2.0),       // heuristic importance
  evaporation_rate_(0.1),
  q_(100.0),        // pheromone deposit quantity
  initial_pheromone_(1.0)
{
    RCLCPP_INFO(logger_, "ACO Planner initialized");
}

std::string ACO::getName()
{
    return "ACO";
}

bool ACO::makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path)
{
    RCLCPP_INFO(logger_, "Path is being planned using ACO algorithm.");
    path.poses.clear();

    if (!costmap_) {
        RCLCPP_ERROR(logger_, "Costmap is null");
        return false;
    }

    unsigned int start_x, start_y;
    if (!worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
        RCLCPP_ERROR(logger_, "Start pose (%.2f, %.2f) is outside the map boundaries",
            start.pose.position.x, start.pose.position.y);
        return false;
    }

    unsigned int goal_x, goal_y;
    if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        RCLCPP_ERROR(logger_, "Goal pose (%.2f, %.2f) is outside the map boundaries",
            goal.pose.position.x, goal.pose.position.y);
        return false;
    }

    // Check if start and goal are the same
    if (start_x == goal_x && start_y == goal_y) {
        RCLCPP_WARN(logger_, "Start and goal are at the same position");
        geometry_msgs::msg::PoseStamped pose = goal;
        pose.header.stamp = rclcpp::Clock().now();
        path.poses.push_back(pose);
        return true;
    }

    // Check if start or goal are obstacles
    int start_idx = toIndex(start_x, start_y);
    int goal_idx = toIndex(goal_x, goal_y);

    if (getCost(start_idx) >= LETHAL_COST || getCost(goal_idx) >= LETHAL_COST) {
        RCLCPP_WARN(logger_, "Start or goal are at an obstacle");
        return false;
    }

    RCLCPP_INFO(logger_, "Planning path from (%u, %u) to (%u, %u)", 
        start_x, start_y, goal_x, goal_y);

    // Initialize pheromone matrix
    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    pheromones_.resize(size_y, std::vector<double>(size_x, initial_pheromone_));

    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Best path found so far
    std::vector<int> best_path;
    double best_cost = std::numeric_limits<double>::max();

    // Main ACO loop
    for (int iter = 0; iter < max_iterations_; ++iter) {
        RCLCPP_INFO(logger_, "ACO iteration %d/%d", iter + 1, max_iterations_);
        
        std::vector<Ant> ants(num_ants_);
        
        // For each ant
        for (int a = 0; a < num_ants_; ++a) {
            // Starting point
            int current_x = start_x;
            int current_y = start_y;
            int current_idx = start_idx;
            
            // Initialize ant path
            ants[a].path_indices.push_back(current_idx);
            ants[a].path_cost = 0.0;
            ants[a].reached_goal = false;
            
            // Visited nodes
            std::vector<bool> visited(size_x * size_y, false);
            visited[current_idx] = true;
            
            // Maximum steps to prevent infinite loops
            int max_steps = size_x * size_y / 2;
            int steps = 0;
            
            // While not reached goal and not exceeded maximum steps
            while (current_idx != goal_idx && steps < max_steps) {
                // Get valid neighbors
                std::vector<ACONode> neighbors = getNeighbors(current_x, current_y, goal_x, goal_y);
                
                if (neighbors.empty()) {
                    break; // Stuck, no valid neighbors
                }
                
                // Calculate total selection probability
                double total_prob = 0.0;
                std::vector<double> probs;
                
                for (const auto & neighbor : neighbors) {
                    int nx = neighbor.x;
                    int ny = neighbor.y;
                    int nidx = neighbor.idx;
                    
                    if (visited[nidx]) continue;
                    
                    double pheromone = pheromones_[ny][nx];
                    double heuristic = 1.0 / (1.0 + getCost(nidx));
                    double probability = std::pow(pheromone, alpha_) * std::pow(heuristic, beta_);
                    
                    probs.push_back(probability);
                    total_prob += probability;
                }
                
                // If no valid unvisited neighbors
                if (total_prob == 0.0) {
                    break;
                }
                
                // Choose next node probabilistically
                double r = dis(gen) * total_prob;
                double sum = 0.0;
                int chosen_idx = -1;
                
                for (size_t i = 0; i < probs.size(); ++i) {
                    sum += probs[i];
                    if (r <= sum) {
                        chosen_idx = i;
                        break;
                    }
                }
                
                if (chosen_idx == -1) {
                    chosen_idx = probs.size() - 1; // Default to last if something went wrong
                }
                
                // Move to chosen neighbor
                const auto & chosen = neighbors[chosen_idx];
                current_x = chosen.x;
                current_y = chosen.y;
                current_idx = chosen.idx;
                
                // Add to path
                ants[a].path_indices.push_back(current_idx);
                ants[a].path_cost += getCost(current_idx);
                
                // Mark as visited
                visited[current_idx] = true;
                
                // Check if reached goal
                if (current_idx == goal_idx) {
                    ants[a].reached_goal = true;
                    break;
                }
                
                steps++;
            }
            
            // Calculate total distance (path length)
            double path_length = 0.0;
            for (size_t i = 1; i < ants[a].path_indices.size(); ++i) {
                int prev_idx = ants[a].path_indices[i-1];
                int curr_idx = ants[a].path_indices[i];
                
                int prev_x, prev_y, curr_x, curr_y;
                fromIndex(prev_idx, prev_x, prev_y);
                fromIndex(curr_idx, curr_x, curr_y);
                
                double dx = curr_x - prev_x;
                double dy = curr_y - prev_y;
                path_length += std::sqrt(dx*dx + dy*dy);
            }
            
            // Add path length to cost
            ants[a].path_cost += path_length * 0.1;
            
            // Update best path if this ant reached the goal and has better path
            if (ants[a].reached_goal && ants[a].path_cost < best_cost) {
                best_path = ants[a].path_indices;
                best_cost = ants[a].path_cost;
                RCLCPP_INFO(logger_, "Found better path with cost: %.2f", best_cost);
            }
        }
        
        // Update pheromones
        updatePheromones(ants);
    }
    
    // If a path was found
    if (!best_path.empty()) {
        // Convert to ROS path message
        path.header.frame_id = start.header.frame_id;
        path.header.stamp = rclcpp::Clock().now();
        
        pathToMsg(best_path, path);
        RCLCPP_INFO(logger_, "ACO planning completed. Path found with %zu waypoints.", path.poses.size());
        return true;
    }
    
    RCLCPP_WARN(logger_, "ACO failed to find a path");
    return false;
}

void ACO::updatePheromones(const std::vector<Ant> & ants)
{
    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    
    // Evaporation
    for (unsigned int y = 0; y < size_y; ++y) {
        for (unsigned int x = 0; x < size_x; ++x) {
            pheromones_[y][x] *= (1.0 - evaporation_rate_);
        }
    }
    
    // Deposit new pheromones from ants that reached the goal
    for (const auto & ant : ants) {
        if (!ant.reached_goal) continue;
        
        double deposit = q_ / ant.path_cost;
        
        for (size_t i = 0; i < ant.path_indices.size() - 1; ++i) {
            int idx = ant.path_indices[i];
            int x, y;
            fromIndex(idx, x, y);
            pheromones_[y][x] += deposit;
        }
    }
}

std::vector<ACONode> ACO::getNeighbors(int x, int y, int goal_x, int goal_y)
{
    std::vector<ACONode> neighbors;
    
    for (int i = 0; i < 8; ++i) {
        int nx = x + DIRECTIONS[i][0];
        int ny = y + DIRECTIONS[i][1];
        
        if (isValid(nx, ny)) {
            int nidx = toIndex(nx, ny);
            ACONode node;
            node.x = nx;
            node.y = ny;
            node.idx = nidx;
            node.pheromone = pheromones_[ny][nx];
            node.prev_idx = toIndex(x, y);
            neighbors.push_back(node);
        }
    }
    
    return neighbors;
}

double ACO::getHeuristic(int x1, int y1, int x2, int y2)
{
    // Euclidean distance
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

void ACO::pathToMsg(const std::vector<int> & path_indices, nav_msgs::msg::Path & path)
{
    path.poses.clear();
    
    for (int idx : path_indices) {
        int x, y;
        fromIndex(idx, x, y);
        
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = path.header.frame_id;
        pose.header.stamp = path.header.stamp;
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        
        path.poses.push_back(pose);
    }
    
    // Smooth the path a bit if needed
    if (path.poses.size() > 3) {
        std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
        smoothed_path.push_back(path.poses.front());
        
        for (size_t i = 1; i < path.poses.size() - 1; ++i) {
            if (i % 2 == 0) {  // Simple decimation - keep every other point
                smoothed_path.push_back(path.poses[i]);
            }
        }
        
        smoothed_path.push_back(path.poses.back());
        path.poses = smoothed_path;
    }
}

std::vector<int> ACO::reconstructPath(
    int start_idx, int goal_idx, 
    const std::vector<int> & came_from)
{
    std::vector<int> path;
    int current = goal_idx;
    path.push_back(current);
    
    while (current != start_idx) {
        current = came_from[current];
        path.push_back(current);
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

bool ACO::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
    return costmap_->worldToMap(wx, wy, mx, my);
}

void ACO::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy)
{
    costmap_->mapToWorld(mx, my, wx, wy);
}

int ACO::toIndex(int x, int y)
{
    return y * costmap_->getSizeInCellsX() + x;
}

void ACO::fromIndex(int idx, int & x, int & y)
{
    unsigned int size_x = costmap_->getSizeInCellsX();
    y = idx / size_x;
    x = idx % size_x;
}

bool ACO::isValid(int x, int y)
{
    if (x >= 0 && x < static_cast<int>(costmap_->getSizeInCellsX()) && 
        y >= 0 && y < static_cast<int>(costmap_->getSizeInCellsY())) {
        unsigned int idx = toIndex(x, y);
        return getCost(idx) < LETHAL_COST;  // Not a lethal obstacle
    }
    return false;
}

double ACO::getCost(unsigned int idx)
{
    unsigned char cost = costmap_->getCost(idx);
    if (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_) {
        return 50.0;  // Moderate cost for unknown cells if allowed
    }
    return static_cast<double>(cost);
}

} // namespace custom_global_planner

// PLUGINLIB_EXPORT_CLASS(custom_global_planner::ACO, nav2_core::GlobalPlanner)