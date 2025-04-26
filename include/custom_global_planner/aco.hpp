#ifndef CUSTOM_GLOBAL_PLANNER__ACO_HPP_
#define CUSTOM_GLOBAL_PLANNER__ACO_HPP_

#include "custom_global_planner/algorithm_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <vector>
#include <queue>
#include <memory>

namespace custom_global_planner
{

// Structure to represent a node in the grid
struct ACONode {
    int x;
    int y;
    int idx;
    double pheromone;
    int prev_idx;  // for path reconstruction
};

// Structure to represent an ant agent
struct Ant {
    std::vector<int> path_indices;  // indices of visited nodes
    double path_cost;
    bool reached_goal;
};

class ACO : public AlgorithmInterface
{
public:
    ACO(nav2_costmap_2d::Costmap2D* costmap, bool allow_unknown);

    ~ACO() override = default;

    std::string getName() override;
    bool makePlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        nav_msgs::msg::Path & plan) override;

private:
    rclcpp::Logger logger_{rclcpp::get_logger("ACO")};

    // ACO parameters
    int num_ants_;             // number of ants
    int max_iterations_;       // maximum number of iterations
    double alpha_;             // pheromone importance factor
    double beta_;              // heuristic importance factor
    double evaporation_rate_;  // pheromone evaporation rate
    double q_;                 // pheromone deposit factor
    double initial_pheromone_; // initial pheromone value

    // Grid movement directions: {dx, dy, cost}
    const int DIRECTIONS[8][2] = {{0,1}, {1,1}, {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}};
    const float DIRECTIONS_COST[8] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};
    const double LETHAL_COST = 255.0;

    // Pheromone matrix
    std::vector<std::vector<double>> pheromones_;

    // Utility functions
    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
    void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy);
    int toIndex(int x, int y);
    void fromIndex(int idx, int & x, int & y);
    bool isValid(int x, int y);
    double getCost(unsigned int idx);
    double getHeuristic(int x1, int y1, int x2, int y2);
    std::vector<ACONode> getNeighbors(int x, int y, int goal_x, int goal_y);
    void updatePheromones(const std::vector<Ant> & ants);
    std::vector<int> reconstructPath(
        int start_idx, int goal_idx, 
        const std::vector<int> & came_from);
    void pathToMsg(
        const std::vector<int> & path_indices, 
        nav_msgs::msg::Path & path);
};

}  // namespace custom_global_planner

#endif  // CUSTOM_GLOBAL_PLANNER__ACO_HPP_