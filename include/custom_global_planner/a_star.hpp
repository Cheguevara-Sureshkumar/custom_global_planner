#ifndef CUSTOM_GLOBAL_PLANNER__A_STAR_HPP_
#define CUSTOM_GLOBAL_PLANNER__A_STAR_HPP_

#include <string>
#include <vector>
#include <queue>
#include <functional>
#include <unordered_map>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "custom_global_planner/algorithm_interface.hpp"

namespace custom_global_planner
{

/**
 * @struct Node
 * @brief A node in the A* search
 */
struct Node
{
  int index;               // Index in the flattened map
  int x, y;                // Coordinates in the map
  double g_cost;           // Cost from start to this node
  double h_cost;           // Heuristic cost (estimated cost to goal)
  double f_cost;           // Total cost (g_cost + h_cost)
  int parent_index;        // Parent node index

  // Comparison operator for priority queue
  bool operator>(const Node & other) const
  {
    return f_cost > other.f_cost;
  }
};

/**
 * @class AStar
 * @brief Implements A* algorithm for path planning
 */
class AStar : public AlgorithmInterface
{
public:
  /**
   * @brief Constructor for A* planner
   * @param costmap Pointer to the costmap
   * @param allow_unknown Whether to allow planning through unknown space
   */
  AStar(nav2_costmap_2d::Costmap2D * costmap, bool allow_unknown);

  /**
   * @brief Destructor
   */
  ~AStar() override = default;

  /**
   * @brief Create a plan using A* algorithm
   * @param start The starting pose
   * @param goal The goal pose
   * @param plan The path created by the algorithm
   * @return Whether successfully created a path
   */
  bool makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & plan) override;

  /**
   * @brief Get the name of the planner
   * @return The planner name
   */
  std::string getName() override;

private:
  // Cost values
  static constexpr double DEFAULT_LETHAL_COST = 253.0;
  static constexpr double INSCRIBED_INFLATED_OBSTACLE = 254.0;
  static constexpr double LETHAL_OBSTACLE = 255.0;
  static constexpr double COST_NEUTRAL = 50.0;
  static constexpr double COST_FACTOR = 0.8;

  // Directions: 8-connected grid
  const int DIRECTIONS[8][2] = {
    {1, 0}, {0, 1}, {-1, 0}, {0, -1},   // 4-connected
    {1, 1}, {-1, 1}, {-1, -1}, {1, -1}  // Diagonals
  };
  const double DIRECTION_COSTS[8] = {
    1.0, 1.0, 1.0, 1.0,    // Straight cost
    1.414, 1.414, 1.414, 1.414  // Diagonal cost (√2)
  };

  /**
   * @brief Calculate heuristic distance (Manhattan)
   * @param x1 Start x coordinate
   * @param y1 Start y coordinate
   * @param x2 Goal x coordinate
   * @param y2 Goal y coordinate
   * @return Heuristic distance
   */
  double getHeuristicDistance(int x1, int y1, int x2, int y2);

  /**
   * @brief Check if a cell is valid for planning
   * @param x X coordinate
   * @param y Y coordinate
   * @return Whether the cell is valid
   */
  bool isValid(int x, int y);

  /**
   * @brief Get cost at a specific cell
   * @param x X coordinate
   * @param y Y coordinate
   * @return Cost value
   */
  double getCost(int x, int y);

  /**
   * @brief Create path from node list
   * @param current Final node
   * @param start_x Start x coordinate
   * @param start_y Start y coordinate
   * @param nodes Map of nodes
   * @param path Resulting path
   */
  void constructPath(
    const Node & current,
    const std::unordered_map<int, Node> & nodes,
    nav_msgs::msg::Path & path);

  /**
   * @brief Convert world coordinates to map cell
   * @param wx World x coordinate
   * @param wy World y coordinate
   * @param mx Map x coordinate (output)
   * @param my Map y coordinate (output)
   * @return Whether conversion was successful
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  /**
   * @brief Convert map cell to world coordinates
   * @param mx Map x coordinate
   * @param my Map y coordinate
   * @param wx World x coordinate (output)
   * @param wy World y coordinate (output)
   */
  void mapToWorld(int mx, int my, double & wx, double & wy);

  /**
   * @brief Flatten 2D coordinates to 1D index
   * @param x X coordinate
   * @param y Y coordinate
   * @return 1D index
   */
  int toIndex(int x, int y);

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("AStar")};
};

}  // namespace custom_global_planner

#endif  // CUSTOM_GLOBAL_PLANNER__A_STAR_HPP_