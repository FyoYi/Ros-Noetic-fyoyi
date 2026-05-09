#ifndef ROS_FYOYI_EXPLORATION_STRATEGY_HPP
#define ROS_FYOYI_EXPLORATION_STRATEGY_HPP

#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ros_fyoyi
{

struct RobotPose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct ExplorationGoal
{
  geometry_msgs::PoseStamped pose;
  int frontier_cells = 0;
  double distance = 0.0;
  double score = 0.0;
};

struct ExplorationStats
{
  int frontier_cells = 0;
  int frontier_clusters = 0;
  int rejected_clusters = 0;
  std::string status;
};

class ExplorationStrategy
{
public:
  virtual ~ExplorationStrategy() = default;

  virtual std::string name() const = 0;

  virtual bool chooseGoal(const nav_msgs::OccupancyGrid& map,
                          const RobotPose2D& robot_pose,
                          const std::vector<RobotPose2D>& rejected_goals,
                          ExplorationGoal* goal,
                          ExplorationStats* stats) = 0;
};

}  // namespace ros_fyoyi

#endif  // ROS_FYOYI_EXPLORATION_STRATEGY_HPP
