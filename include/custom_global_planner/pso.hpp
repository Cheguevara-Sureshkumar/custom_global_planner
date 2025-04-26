#ifndef CUSTOM_GLOBAL_PLANNER__PSO_HPP_
#define CUSTOM_GLOBAL_PLANNER__PSO_HPP_

#include "custom_global_planner/algorithm_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include <vector>
#include <memory>
#include <queue>

namespace custom_global_planner
{

struct Particle
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  double cost;
};

class PSO : public AlgorithmInterface
{
public:
  PSO(nav2_costmap_2d::Costmap2D* costmap, bool allow_unknown);

  ~PSO() override = default;

  std::string getName() override;
  bool makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & plan) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("PSO")};

  int swarm_size_;
  int max_iterations_;
  double w_;      // inertia weight
  double c1_;     // cognitive coefficient
  double c2_;     // social coefficient
  int num_waypoints_; // number of waypoints between start and goal

  std::vector<geometry_msgs::msg::PoseStamped> generateRandomPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  double calculateCost(const std::vector<geometry_msgs::msg::PoseStamped> & path);
  bool isValid(int x, int y);
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy);
  double getCost(unsigned int idx);
  int toIndex(int x, int y);
};

}  // namespace custom_global_planner

#endif  // CUSTOM_GLOBAL_PLANNER__PSO_HPP_