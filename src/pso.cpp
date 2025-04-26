#include "custom_global_planner/pso.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <random>
#include <limits>
#include <cmath>

namespace custom_global_planner
{

PSO::PSO(nav2_costmap_2d::Costmap2D* costmap, bool allow_unknown)
: AlgorithmInterface(costmap, allow_unknown),
  swarm_size_(30),
  max_iterations_(100),
  w_(0.5),
  c1_(1.5),
  c2_(1.5),
  num_waypoints_(10)
{
  RCLCPP_INFO(logger_, "PSO Planner initialized");
}

std::string PSO::getName()
{
  return "PSO";
}

bool PSO::makePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(logger_, "Path is being planned using PSO algorithm.");
  path.poses.clear();  // Clear previous path or initialize empty path

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
  const double LETHAL_COST = 255.0;

  if (getCost(start_idx) >= LETHAL_COST || getCost(goal_idx) >= LETHAL_COST) {
    RCLCPP_WARN(logger_, "Start or goal are at an obstacle");
    return false;
  }

  RCLCPP_INFO(logger_, "Start pose is (%.2f, %.2f) within the map.", 
    start.pose.position.x, start.pose.position.y);
  RCLCPP_INFO(logger_, "Goal pose is (%.2f, %.2f) within the map.", 
    goal.pose.position.x, goal.pose.position.y);

  // Initialize particle swarm
  std::vector<Particle> swarm(swarm_size_);
  Particle global_best;
  global_best.cost = std::numeric_limits<double>::max();

  // Set up random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> random_factor(0.0, 1.0);

  // Initialize particles with random paths
  for (auto & particle : swarm) {
    particle.path = generateRandomPath(start, goal);
    particle.cost = calculateCost(particle.path);

    if (particle.cost < global_best.cost) {
      global_best = particle;
    }
  }

  RCLCPP_INFO(logger_, "Initialized swarm with %d particles", swarm_size_);

  // PSO main loop
  for (int iter = 0; iter < max_iterations_; ++iter) {
    RCLCPP_INFO(logger_, "PSO iteration %d/%d", iter + 1, max_iterations_);
    
    for (auto & particle : swarm) {
      std::vector<geometry_msgs::msg::PoseStamped> new_path = particle.path;

      // Skip start and goal positions (keep them fixed)
      for (size_t i = 1; i < particle.path.size() - 1; ++i) {
        geometry_msgs::msg::PoseStamped & pose = new_path[i];
        
        // Calculate cognitive component (personal best)
        double r1 = random_factor(gen);
        
        // Calculate social component (global best)
        double r2 = random_factor(gen);
        
        // Update position based on PSO formula
        pose.pose.position.x += w_ * (pose.pose.position.x - particle.path[i].pose.position.x) + 
                                c1_ * r1 * (global_best.path[i].pose.position.x - pose.pose.position.x) + 
                                c2_ * r2 * (global_best.path[i].pose.position.x - pose.pose.position.x);
        
        pose.pose.position.y += w_ * (pose.pose.position.y - particle.path[i].pose.position.y) + 
                                c1_ * r1 * (global_best.path[i].pose.position.y - pose.pose.position.y) + 
                                c2_ * r2 * (global_best.path[i].pose.position.y - pose.pose.position.y);
        
        // Ensure the point is valid
        unsigned int mx, my;
        if (worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
          if (!isValid(mx, my)) {
            // If invalid, use the original position
            pose = particle.path[i];
          }
        } else {
          // If out of bounds, use the original position
          pose = particle.path[i];
        }
      }

      particle.path = new_path;
      particle.cost = calculateCost(particle.path);

      if (particle.cost < global_best.cost) {
        global_best = particle;
        RCLCPP_INFO(logger_, "Found new global best with cost: %.2f", global_best.cost);
      }
    }
  }

  // Set the final path
  path.header.frame_id = start.header.frame_id;
  path.header.stamp = rclcpp::Clock().now();
  path.poses = global_best.path;
  
  RCLCPP_INFO(logger_, "PSO planning completed. Path found with %zu waypoints.", path.poses.size());
  return !path.poses.empty();
}

std::vector<geometry_msgs::msg::PoseStamped> PSO::generateRandomPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.push_back(start);  // Add start pose

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-0.5, 0.5);  // For random perturbation

  // Linear interpolation with random perturbation
  for (int i = 1; i <= num_waypoints_; ++i) {
    double ratio = static_cast<double>(i) / (num_waypoints_ + 1);
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = start.header.frame_id;
    pose.header.stamp = rclcpp::Clock().now();
    
    // Linear interpolation
    pose.pose.position.x = start.pose.position.x + 
      (goal.pose.position.x - start.pose.position.x) * ratio;
    pose.pose.position.y = start.pose.position.y + 
      (goal.pose.position.y - start.pose.position.y) * ratio;
    
    // Add random perturbation
    double perturbation_x = dis(gen) * std::abs(goal.pose.position.x - start.pose.position.x) * 0.3;
    double perturbation_y = dis(gen) * std::abs(goal.pose.position.y - start.pose.position.y) * 0.3;
    
    pose.pose.position.x += perturbation_x;
    pose.pose.position.y += perturbation_y;
    
    // Keep quaternion normalized
    pose.pose.orientation.w = 1.0;
    
    // Check if position is valid
    unsigned int mx, my;
    if (worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      if (isValid(mx, my)) {
        path.push_back(pose);
      } else {
        // If not valid, use linear position without perturbation
        pose.pose.position.x -= perturbation_x;
        pose.pose.position.y -= perturbation_y;
        path.push_back(pose);
      }
    } else {
      // If outside map, use linear position without perturbation
      pose.pose.position.x -= perturbation_x;
      pose.pose.position.y -= perturbation_y;
      path.push_back(pose);
    }
  }

  path.push_back(goal);  // Add goal pose
  return path;
}

double PSO::calculateCost(const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  double total_cost = 0.0;
  
  // Sum cost of all points
  for (const auto & pose : path) {
    unsigned int mx, my;
    if (worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      unsigned int idx = toIndex(mx, my);
      double cell_cost = getCost(idx);
      total_cost += cell_cost;
    } else {
      total_cost += 255.0;  // Heavily penalize points outside the map
    }
  }

  // Add path smoothness cost
  for (size_t i = 1; i < path.size() - 1; ++i) {
    const auto & prev = path[i-1];
    const auto & curr = path[i];
    const auto & next = path[i+1];
    
    // Calculate angle between consecutive segments
    double dx1 = curr.pose.position.x - prev.pose.position.x;
    double dy1 = curr.pose.position.y - prev.pose.position.y;
    double dx2 = next.pose.position.x - curr.pose.position.x;
    double dy2 = next.pose.position.y - curr.pose.position.y;
    
    // Calculate angle between segments using dot product
    double dot = dx1 * dx2 + dy1 * dy2;
    double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    
    // Avoid division by zero
    if (mag1 > 0.0001 && mag2 > 0.0001) {
      double cos_angle = dot / (mag1 * mag2);
      // Clamp to avoid numerical errors
      cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
      
      // Sharp turns are penalized
      double angle_penalty = (1.0 - cos_angle) * 50.0;
      total_cost += angle_penalty;
    }
  }
  
  // Cost for path length
  double path_length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & prev = path[i-1];
    const auto & curr = path[i];
    
    double dx = curr.pose.position.x - prev.pose.position.x;
    double dy = curr.pose.position.y - prev.pose.position.y;
    path_length += std::sqrt(dx * dx + dy * dy);
  }
  
  // Add a scaled path length component to encourage shorter paths
  total_cost += path_length * 0.5;
  
  return total_cost;
}

bool PSO::isValid(int x, int y)
{
  if (x >= 0 && x < static_cast<int>(costmap_->getSizeInCellsX()) && 
      y >= 0 && y < static_cast<int>(costmap_->getSizeInCellsY())) {
    unsigned int idx = toIndex(x, y);
    return getCost(idx) < 254.0;  // Not a lethal obstacle
  }
  return false;
}

bool PSO::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

void PSO::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy)
{
  costmap_->mapToWorld(mx, my, wx, wy);
}

double PSO::getCost(unsigned int idx)
{
  unsigned char cost = costmap_->getCost(idx);
  if (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_) {
    return 50.0;  // Moderate cost for unknown cells if allowed
  }
  return static_cast<double>(cost);
}

int PSO::toIndex(int x, int y)
{
  return y * costmap_->getSizeInCellsX() + x;
}

} // namespace custom_global_planner

// PLUGINLIB_EXPORT_CLASS(custom_global_planner::PSO, nav2_core::GlobalPlanner)