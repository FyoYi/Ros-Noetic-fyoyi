#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ros_fyoyi/exploration_strategy.hpp>

namespace ros_fyoyi
{
namespace
{

int indexOf(int x, int y, int width)
{
  return y * width + x;
}

bool isFreeCell(int8_t value)
{
  return value == 0;
}

bool isOccupiedCell(int8_t value)
{
  return value >= 50;
}

geometry_msgs::Quaternion yawToQuaternion(double yaw)
{
  return tf::createQuaternionMsgFromYaw(yaw);
}

class SafeFrontierStrategy : public ExplorationStrategy
{
public:
  SafeFrontierStrategy(ros::NodeHandle& nh)
  {
    nh.param("min_frontier_cells", min_frontier_cells_, 10);
    nh.param("frontier_gain_scale", gain_scale_, 0.35);
    nh.param("frontier_goal_offset", frontier_goal_offset_, 0.8);
    nh.param("candidate_search_radius", candidate_search_radius_, 1.5);
    nh.param("min_goal_distance", min_goal_distance_, 0.8);
    nh.param("max_goal_distance", max_goal_distance_, 8.0);
    nh.param("obstacle_clearance", obstacle_clearance_, 0.45);
    nh.param("rejected_goal_radius", rejected_goal_radius_, 1.0);
    nh.param("occupied_threshold", occupied_threshold_, 50);
    nh.param("frontier_preprocess_iterations", preprocess_iterations_, 3);
    nh.param("frontier_distance_alpha", distance_alpha_, 0.85);
    nh.param("frontier_size_weight", size_weight_, 0.015);
  }

  std::string name() const override
  {
    return "safe_frontier";
  }

  bool chooseGoal(const nav_msgs::OccupancyGrid& map,
                  const RobotPose2D& robot_pose,
                  const std::vector<RobotPose2D>& rejected_goals,
                  ExplorationGoal* goal,
                  ExplorationStats* stats) override
  {
    if (!goal || !stats)
    {
      return false;
    }

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    if (width < 3 || height < 3)
    {
      stats->status = "map is too small; waiting for more SLAM data";
      return false;
    }

    std::vector<int8_t> processed = preprocessMap(map, preprocess_iterations_);
    std::vector<uint8_t> frontier(width * height, 0);
    for (int y = 1; y < height - 1; ++y)
    {
      for (int x = 1; x < width - 1; ++x)
      {
        const int idx = indexOf(x, y, width);
        if (isFreeCell(processed[idx]) && hasUnknownNeighbor(processed, width, x, y))
        {
          frontier[idx] = 1;
          ++stats->frontier_cells;
        }
      }
    }

    std::vector<uint8_t> visited(width * height, 0);
    bool found = false;
    ExplorationGoal best_goal;
    best_goal.score = std::numeric_limits<double>::infinity();

    for (int idx = 0; idx < width * height; ++idx)
    {
      if (!frontier[idx] || visited[idx])
      {
        continue;
      }

      std::vector<int> cluster = collectCluster(idx, frontier, visited, width, height);
      if (static_cast<int>(cluster.size()) < min_frontier_cells_)
      {
        ++stats->rejected_clusters;
        continue;
      }
      ++stats->frontier_clusters;

      ExplorationGoal candidate;
      if (!buildClusterGoal(map, processed, cluster, robot_pose, rejected_goals, &candidate))
      {
        ++stats->rejected_clusters;
        continue;
      }

      if (candidate.score < best_goal.score)
      {
        best_goal = candidate;
        found = true;
      }
    }

    if (!found)
    {
      stats->status = "no safe frontier goal";
      return false;
    }

    *goal = best_goal;
    stats->status = "safe frontier goal found";
    return true;
  }

private:
  std::vector<int8_t> preprocessMap(const nav_msgs::OccupancyGrid& map, int iterations) const
  {
    std::vector<int8_t> current = map.data;
    std::vector<int8_t> output = current;
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);

    for (int iter = 0; iter < iterations; ++iter)
    {
      for (int y = 0; y < height; ++y)
      {
        for (int x = 0; x < width; ++x)
        {
          const int idx = indexOf(x, y, width);
          if (current[idx] >= occupied_threshold_)
          {
            output[idx] = 100;
            continue;
          }

          int free = 0;
          int unknown = 0;
          forEachNeighbor8(width, height, x, y, [&](int nx, int ny) {
            const int8_t value = current[indexOf(nx, ny, width)];
            if (value == 0)
            {
              ++free;
            }
            else if (value == -1)
            {
              ++unknown;
            }
          });

          output[idx] = unknown >= free ? -1 : 0;
        }
      }
      current = output;
    }
    return output;
  }

  template <typename Callback>
  void forEachNeighbor8(int width, int height, int x, int y, Callback callback) const
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        if (dx == 0 && dy == 0)
        {
          continue;
        }
        const int nx = x + dx;
        const int ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height)
        {
          continue;
        }
        callback(nx, ny);
      }
    }
  }

  bool hasUnknownNeighbor(const std::vector<int8_t>& grid, int width, int x, int y) const
  {
    return grid[indexOf(x - 1, y, width)] == -1 || grid[indexOf(x + 1, y, width)] == -1 ||
           grid[indexOf(x, y - 1, width)] == -1 || grid[indexOf(x, y + 1, width)] == -1;
  }

  std::vector<int> collectCluster(int start_idx,
                                  const std::vector<uint8_t>& frontier,
                                  std::vector<uint8_t>& visited,
                                  int width,
                                  int height) const
  {
    std::queue<int> queue;
    std::vector<int> cluster;
    queue.push(start_idx);
    visited[start_idx] = 1;

    while (!queue.empty())
    {
      const int idx = queue.front();
      queue.pop();
      cluster.push_back(idx);
      const int x = idx % width;
      const int y = idx / width;

      const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
      const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
      for (int i = 0; i < 8; ++i)
      {
        const int nx = x + dx[i];
        const int ny = y + dy[i];
        if (nx <= 0 || ny <= 0 || nx >= width - 1 || ny >= height - 1)
        {
          continue;
        }
        const int nidx = indexOf(nx, ny, width);
        if (frontier[nidx] && !visited[nidx])
        {
          visited[nidx] = 1;
          queue.push(nidx);
        }
      }
    }

    return cluster;
  }

  bool buildClusterGoal(const nav_msgs::OccupancyGrid& map,
                        const std::vector<int8_t>& processed,
                        const std::vector<int>& cluster,
                        const RobotPose2D& robot_pose,
                        const std::vector<RobotPose2D>& rejected_goals,
                        ExplorationGoal* goal) const
  {
    double cx = 0.0;
    double cy = 0.0;
    for (const int idx : cluster)
    {
      double wx = 0.0;
      double wy = 0.0;
      indexToWorld(map, idx, &wx, &wy);
      cx += wx;
      cy += wy;
    }
    cx /= static_cast<double>(cluster.size());
    cy /= static_cast<double>(cluster.size());

    double vx = robot_pose.x - cx;
    double vy = robot_pose.y - cy;
    const double len = std::hypot(vx, vy);
    if (len > 1e-6)
    {
      vx /= len;
      vy /= len;
    }
    else
    {
      vx = std::cos(robot_pose.yaw);
      vy = std::sin(robot_pose.yaw);
    }

    const double target_x = cx + vx * frontier_goal_offset_;
    const double target_y = cy + vy * frontier_goal_offset_;

    int center_mx = 0;
    int center_my = 0;
    if (!worldToMap(map, target_x, target_y, &center_mx, &center_my))
    {
      return false;
    }

    const int search_cells = std::max(1, static_cast<int>(std::ceil(candidate_search_radius_ / map.info.resolution)));
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    bool found = false;
    double best_score = std::numeric_limits<double>::infinity();
    double best_x = 0.0;
    double best_y = 0.0;
    double best_distance = 0.0;

    for (int y = std::max(1, center_my - search_cells); y <= std::min(height - 2, center_my + search_cells); ++y)
    {
      for (int x = std::max(1, center_mx - search_cells); x <= std::min(width - 2, center_mx + search_cells); ++x)
      {
        const int idx = indexOf(x, y, width);
        if (!isFreeCell(processed[idx]) || !isFreeCell(map.data[idx]) || !isSafeCell(map, x, y))
        {
          continue;
        }

        double wx = 0.0;
        double wy = 0.0;
        indexToWorld(map, idx, &wx, &wy);
        const double distance = std::hypot(wx - robot_pose.x, wy - robot_pose.y);
        if (distance < min_goal_distance_ || distance > max_goal_distance_)
        {
          continue;
        }
        if (isRejected(wx, wy, rejected_goals))
        {
          continue;
        }

        const double target_error = std::hypot(wx - target_x, wy - target_y);
        const double frontier_distance = std::hypot(wx - cx, wy - cy);
        const double reference_utility = distance_alpha_ / std::max(distance, 0.1) +
                                         size_weight_ * static_cast<double>(cluster.size());
        const double information_gain = gain_scale_ * std::sqrt(static_cast<double>(cluster.size()));
        const double score = -reference_utility - information_gain + 0.8 * target_error + 0.2 * frontier_distance;
        if (score < best_score)
        {
          best_score = score;
          best_x = wx;
          best_y = wy;
          best_distance = distance;
          found = true;
        }
      }
    }

    if (!found)
    {
      return false;
    }

    const double yaw = std::atan2(cy - best_y, cx - best_x);
    goal->pose.header.frame_id = map.header.frame_id.empty() ? "map" : map.header.frame_id;
    goal->pose.header.stamp = ros::Time::now();
    goal->pose.pose.position.x = best_x;
    goal->pose.pose.position.y = best_y;
    goal->pose.pose.orientation = yawToQuaternion(yaw);
    goal->frontier_cells = static_cast<int>(cluster.size());
    goal->distance = best_distance;
    goal->score = best_score;
    return true;
  }

  bool isSafeCell(const nav_msgs::OccupancyGrid& map, int mx, int my) const
  {
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(obstacle_clearance_ / map.info.resolution)));
    for (int y = std::max(0, my - radius_cells); y <= std::min(height - 1, my + radius_cells); ++y)
    {
      for (int x = std::max(0, mx - radius_cells); x <= std::min(width - 1, mx + radius_cells); ++x)
      {
        const double d = std::hypot(static_cast<double>(x - mx), static_cast<double>(y - my)) * map.info.resolution;
        if (d > obstacle_clearance_)
        {
          continue;
        }
        const int8_t value = map.data[indexOf(x, y, width)];
        if (value >= occupied_threshold_ || isOccupiedCell(value))
        {
          return false;
        }
      }
    }
    return true;
  }

  bool isRejected(double x, double y, const std::vector<RobotPose2D>& rejected_goals) const
  {
    for (const RobotPose2D& rejected : rejected_goals)
    {
      if (std::hypot(x - rejected.x, y - rejected.y) <= rejected_goal_radius_)
      {
        return true;
      }
    }
    return false;
  }

  bool worldToMap(const nav_msgs::OccupancyGrid& map, double wx, double wy, int* mx, int* my) const
  {
    const double ox = map.info.origin.position.x;
    const double oy = map.info.origin.position.y;
    const double resolution = map.info.resolution;
    if (wx < ox || wy < oy)
    {
      return false;
    }
    const int x = static_cast<int>((wx - ox) / resolution);
    const int y = static_cast<int>((wy - oy) / resolution);
    if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) || y >= static_cast<int>(map.info.height))
    {
      return false;
    }
    *mx = x;
    *my = y;
    return true;
  }

  void indexToWorld(const nav_msgs::OccupancyGrid& map, int idx, double* wx, double* wy) const
  {
    const int width = static_cast<int>(map.info.width);
    const int mx = idx % width;
    const int my = idx / width;
    *wx = map.info.origin.position.x + (static_cast<double>(mx) + 0.5) * map.info.resolution;
    *wy = map.info.origin.position.y + (static_cast<double>(my) + 0.5) * map.info.resolution;
  }

  int min_frontier_cells_ = 10;
  double gain_scale_ = 0.35;
  double frontier_goal_offset_ = 0.8;
  double candidate_search_radius_ = 1.5;
  double min_goal_distance_ = 0.8;
  double max_goal_distance_ = 8.0;
  double obstacle_clearance_ = 0.45;
  double rejected_goal_radius_ = 1.0;
  int occupied_threshold_ = 50;
  int preprocess_iterations_ = 3;
  double distance_alpha_ = 0.85;
  double size_weight_ = 0.015;
};

std::unique_ptr<ExplorationStrategy> createStrategy(const std::string& strategy_name, ros::NodeHandle& nh)
{
  if (strategy_name == "safe_frontier")
  {
    return std::unique_ptr<ExplorationStrategy>(new SafeFrontierStrategy(nh));
  }

  ROS_WARN("Explorer: unknown strategy '%s'; fallback to safe_frontier", strategy_name.c_str());
  return std::unique_ptr<ExplorationStrategy>(new SafeFrontierStrategy(nh));
}

class AutonomousExplorer
{
public:
  AutonomousExplorer()
    : private_nh_("~")
    , move_base_client_("move_base", true)
  {
    private_nh_.param("map_frame", map_frame_, std::string("map"));
    private_nh_.param("robot_base_frame", robot_base_frame_, std::string("base_footprint"));
    private_nh_.param("map_topic", map_topic_, std::string("/map"));
    private_nh_.param("planner_period", planner_period_, 5.0);
    private_nh_.param("goal_timeout", goal_timeout_, 45.0);
    private_nh_.param("no_progress_timeout", no_progress_timeout_, 10.0);
    private_nh_.param("stop_motion_timeout", stop_motion_timeout_, 10.0);
    private_nh_.param("stop_motion_epsilon", stop_motion_epsilon_, 0.05);
    private_nh_.param("max_stuck_replans_before_prompt", max_stuck_replans_before_prompt_, 4);
    private_nh_.param("progress_epsilon", progress_epsilon_, 0.08);
    private_nh_.param("make_plan_tolerance", make_plan_tolerance_, 0.25);
    private_nh_.param("max_goal_search_attempts", max_goal_search_attempts_, 8);
    private_nh_.param("completion_no_goal_duration", completion_no_goal_duration_, 35.0);
    private_nh_.param("replan_after_failure_delay", replan_after_failure_delay_, 2.0);
    private_nh_.param("strategy", strategy_name_, std::string("safe_frontier"));

    strategy_ = createStrategy(strategy_name_, private_nh_);
    map_sub_ = nh_.subscribe(map_topic_, 1, &AutonomousExplorer::mapCallback, this);
    command_sub_ = nh_.subscribe("/ros_fyoyi/explorer_command", 1, &AutonomousExplorer::commandCallback, this);
    state_pub_ = nh_.advertise<std_msgs::String>("/ros_fyoyi/explorer_state", 1, true);
    make_plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
    timer_ = nh_.createTimer(ros::Duration(planner_period_), &AutonomousExplorer::planTimer, this);

    ROS_INFO("Explorer: C++ node started, strategy=%s, waiting for move_base", strategy_->name().c_str());
    move_base_client_.waitForServer(ros::Duration(10.0));
    if (!move_base_client_.isServerConnected())
    {
      ROS_WARN("Explorer: move_base action server is not connected yet; will keep waiting");
    }
  }

private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    latest_map_ = *msg;
    have_map_ = true;
  }

  void commandCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::istringstream stream(msg->data);
    std::string command;
    stream >> command;

    if (command == "pause")
    {
      move_base_client_.cancelAllGoals();
      has_active_goal_ = false;
      manual_escape_active_ = false;
      paused_ = true;
      publishState("paused");
      return;
    }

    if (command == "resume")
    {
      paused_ = false;
      manual_escape_active_ = false;
      consecutive_stuck_replans_ = 0;
      no_goal_since_ = ros::Time();
      motion_initialized_ = false;
      publishState("resumed");
      return;
    }

    if (command == "escape")
    {
      double x = 0.0;
      double y = 0.0;
      double yaw = 0.0;
      if (!(stream >> x >> y >> yaw))
      {
        ROS_WARN("Explorer: invalid escape command, expected: escape x y yaw");
        return;
      }
      sendEscapeGoal(x, y, yaw);
      return;
    }

    ROS_WARN("Explorer: unknown command '%s'", msg->data.c_str());
  }

  void planTimer(const ros::TimerEvent&)
  {
    if (!have_map_)
    {
      publishState("waiting_for_map");
      ROS_INFO_THROTTLE(10.0, "Explorer: waiting for SLAM map");
      return;
    }
    if (!move_base_client_.isServerConnected())
    {
      publishState("waiting_for_move_base");
      ROS_INFO_THROTTLE(10.0, "Explorer: waiting for move_base action server");
      move_base_client_.waitForServer(ros::Duration(0.5));
      return;
    }

    if (manual_escape_active_)
    {
      handleManualEscape();
      return;
    }

    if (paused_)
    {
      publishState("paused");
      return;
    }

    RobotPose2D robot_pose;
    if (!lookupRobotPose(&robot_pose))
    {
      return;
    }

    const actionlib::SimpleClientGoalState state = move_base_client_.getState();
    if (has_active_goal_)
    {
      const double goal_distance = std::hypot(active_goal_.pose.pose.position.x - robot_pose.x,
                                              active_goal_.pose.pose.position.y - robot_pose.y);
      if (state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING)
      {
        updateProgress(goal_distance);
        if ((ros::Time::now() - last_progress_time_).toSec() > no_progress_timeout_)
        {
          ROS_WARN("Explorer: no progress for %.1fs at goal x=%.2f y=%.2f; canceling and trying another frontier",
                   no_progress_timeout_,
                   active_goal_.pose.pose.position.x,
                   active_goal_.pose.pose.position.y);
          move_base_client_.cancelGoal();
          rememberRejectedGoal(active_goal_);
          has_active_goal_ = false;
          motion_initialized_ = false;
          ++consecutive_stuck_replans_;
          if (consecutive_stuck_replans_ >= max_stuck_replans_before_prompt_)
          {
            ROS_WARN("Explorer: %d consecutive goals got stuck; entering user decision prompt",
                     consecutive_stuck_replans_);
            paused_ = true;
            publishState("stopped_over_10s");
            return;
          }
          publishState("goal_stuck_replan");
          last_failure_time_ = ros::Time::now();
          return;
        }

        if (detectStoppedTooLong(robot_pose))
        {
          return;
        }

        if ((ros::Time::now() - active_goal_sent_at_).toSec() <= goal_timeout_)
        {
          ROS_INFO_THROTTLE(5.0,
                            "Explorer: moving to goal x=%.2f y=%.2f, straight_distance=%.2fm",
                            active_goal_.pose.pose.position.x,
                            active_goal_.pose.pose.position.y,
                            goal_distance);
          return;
        }

        ROS_WARN("Explorer: goal timeout; cancel and blacklist this area");
        move_base_client_.cancelGoal();
        publishState("goal_timeout");
        rememberRejectedGoal(active_goal_);
        has_active_goal_ = false;
        motion_initialized_ = false;
        ++consecutive_stuck_replans_;
        last_failure_time_ = ros::Time::now();
        return;
      }

      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Explorer: goal reached; searching next frontier");
        publishState("goal_reached");
        has_active_goal_ = false;
        consecutive_stuck_replans_ = 0;
        motion_initialized_ = false;
      }
      else
      {
        ROS_WARN("Explorer: move_base ended with state=%s; choosing another goal", state.toString().c_str());
        publishState("goal_failed");
        rememberRejectedGoal(active_goal_);
        has_active_goal_ = false;
        motion_initialized_ = false;
        ++consecutive_stuck_replans_;
        last_failure_time_ = ros::Time::now();
        return;
      }
    }

    if ((ros::Time::now() - last_failure_time_).toSec() < replan_after_failure_delay_)
    {
      return;
    }

    ExplorationStats stats;
    ExplorationGoal next_goal;
    if (!findPlannableGoal(robot_pose, &next_goal, &stats))
    {
      if (no_goal_since_.isZero())
      {
        no_goal_since_ = ros::Time::now();
      }
      const double no_goal_duration = (ros::Time::now() - no_goal_since_).toSec();
      if (no_goal_duration >= completion_no_goal_duration_)
      {
        ROS_WARN("Explorer: no plannable frontier for %.1fs; stopping exploration", no_goal_duration);
        move_base_client_.cancelAllGoals();
        has_active_goal_ = false;
        motion_initialized_ = false;
        paused_ = true;
        publishState("exploration_done");
        return;
      }
      publishState("searching_no_goal");
      ROS_INFO_THROTTLE(10.0,
                        "Explorer: checking frontiers, frontier_cells=%d clusters=%d rejected=%d no_goal_for=%.1fs status=%s",
                        stats.frontier_cells,
                        stats.frontier_clusters,
                        stats.rejected_clusters,
                        no_goal_duration,
                        stats.status.c_str());
      return;
    }

    no_goal_since_ = ros::Time();
    publishState("active_goal");
    ROS_INFO("Explorer: sending goal x=%.2f y=%.2f frontier_size=%d distance=%.2fm score=%.2f",
             next_goal.pose.pose.position.x,
             next_goal.pose.pose.position.y,
             next_goal.frontier_cells,
             next_goal.distance,
             next_goal.score);
    sendGoal(next_goal);
  }

  bool lookupRobotPose(RobotPose2D* pose)
  {
    try
    {
      tf::StampedTransform transform;
      tf_listener_.waitForTransform(map_frame_, robot_base_frame_, ros::Time(0), ros::Duration(0.5));
      tf_listener_.lookupTransform(map_frame_, robot_base_frame_, ros::Time(0), transform);
      pose->x = transform.getOrigin().x();
      pose->y = transform.getOrigin().y();
      pose->yaw = tf::getYaw(transform.getRotation());
      return true;
    }
    catch (const tf::TransformException& ex)
    {
      ROS_WARN_THROTTLE(5.0, "Explorer: waiting for TF %s -> %s: %s", map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool findPlannableGoal(const RobotPose2D& robot_pose, ExplorationGoal* selected_goal, ExplorationStats* final_stats)
  {
    for (int attempt = 0; attempt < max_goal_search_attempts_; ++attempt)
    {
      ExplorationStats stats;
      ExplorationGoal goal;
      if (!strategy_->chooseGoal(latest_map_, robot_pose, rejected_goals_, &goal, &stats))
      {
        if (final_stats)
        {
          *final_stats = stats;
        }
        return false;
      }

      if (canMakePlan(robot_pose, goal))
      {
        if (selected_goal)
        {
          *selected_goal = goal;
        }
        if (final_stats)
        {
          *final_stats = stats;
        }
        if (attempt > 0)
        {
          ROS_INFO("Explorer: selected plannable fallback goal after %d rejected candidates", attempt);
        }
        return true;
      }

      ROS_WARN("Explorer: candidate has no global plan, reject x=%.2f y=%.2f",
               goal.pose.pose.position.x,
               goal.pose.pose.position.y);
      rememberRejectedGoal(goal);
      if (final_stats)
      {
        *final_stats = stats;
      }
    }

    if (final_stats)
    {
      final_stats->status = "all candidate goals failed make_plan";
    }
    return false;
  }

  bool canMakePlan(const RobotPose2D& robot_pose, const ExplorationGoal& goal)
  {
    if (!make_plan_client_.exists())
    {
      make_plan_client_.waitForExistence(ros::Duration(0.2));
    }
    if (!make_plan_client_.exists())
    {
      ROS_WARN_THROTTLE(10.0, "Explorer: /move_base/make_plan is unavailable; sending goals without precheck");
      return true;
    }

    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = map_frame_;
    srv.request.start.header.stamp = ros::Time::now();
    srv.request.start.pose.position.x = robot_pose.x;
    srv.request.start.pose.position.y = robot_pose.y;
    srv.request.start.pose.orientation = yawToQuaternion(robot_pose.yaw);
    srv.request.goal = goal.pose;
    srv.request.goal.header.stamp = ros::Time::now();
    srv.request.tolerance = make_plan_tolerance_;

    if (!make_plan_client_.call(srv))
    {
      ROS_WARN_THROTTLE(10.0, "Explorer: make_plan service call failed; sending goals without precheck");
      return true;
    }
    return !srv.response.plan.poses.empty();
  }

  void updateProgress(double goal_distance)
  {
    if (!have_progress_measurement_ || goal_distance < best_active_goal_distance_ - progress_epsilon_)
    {
      best_active_goal_distance_ = goal_distance;
      last_progress_time_ = ros::Time::now();
      have_progress_measurement_ = true;
    }
  }

  void sendGoal(const ExplorationGoal& goal)
  {
    move_base_msgs::MoveBaseGoal move_goal;
    move_goal.target_pose = goal.pose;
    move_goal.target_pose.header.stamp = ros::Time::now();
    move_base_client_.sendGoal(move_goal);
    active_goal_ = goal;
    active_goal_sent_at_ = ros::Time::now();
    last_progress_time_ = active_goal_sent_at_;
    best_active_goal_distance_ = std::numeric_limits<double>::infinity();
    have_progress_measurement_ = false;
    has_active_goal_ = true;
    ever_sent_goal_ = true;
  }

  void sendEscapeGoal(double x, double y, double yaw)
  {
    move_base_client_.cancelAllGoals();
    has_active_goal_ = false;
    paused_ = false;
    manual_escape_active_ = true;
    ever_sent_goal_ = true;
    manual_escape_sent_at_ = ros::Time::now();

    move_base_msgs::MoveBaseGoal move_goal;
    move_goal.target_pose.header.frame_id = map_frame_;
    move_goal.target_pose.header.stamp = ros::Time::now();
    move_goal.target_pose.pose.position.x = x;
    move_goal.target_pose.pose.position.y = y;
    move_goal.target_pose.pose.orientation = yawToQuaternion(yaw);
    move_base_client_.sendGoal(move_goal);

    publishState("manual_escape_goal");
    ROS_INFO("Explorer: manual escape goal sent x=%.2f y=%.2f yaw=%.2f", x, y, yaw);
  }

  void handleManualEscape()
  {
    const actionlib::SimpleClientGoalState state = move_base_client_.getState();
    if (state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING)
    {
      if ((ros::Time::now() - manual_escape_sent_at_).toSec() <= goal_timeout_)
      {
        publishState("manual_escape_active");
        ROS_INFO_THROTTLE(5.0, "Explorer: executing manual escape goal");
        return;
      }
      ROS_WARN("Explorer: manual escape goal timeout; resuming autonomous exploration");
      move_base_client_.cancelGoal();
    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Explorer: manual escape goal reached; resuming autonomous exploration");
    }
    else
    {
      ROS_WARN("Explorer: manual escape finished with state=%s; resuming autonomous exploration", state.toString().c_str());
    }

    manual_escape_active_ = false;
    no_goal_since_ = ros::Time();
    last_failure_time_ = ros::Time::now();
    publishState("manual_escape_done");
  }

  bool detectStoppedTooLong(const RobotPose2D& robot_pose)
  {
    if (!has_active_goal_)
    {
      motion_initialized_ = false;
      return false;
    }

    if (!motion_initialized_)
    {
      last_motion_x_ = robot_pose.x;
      last_motion_y_ = robot_pose.y;
      last_motion_time_ = ros::Time::now();
      motion_initialized_ = true;
      return false;
    }

    const double moved = std::hypot(robot_pose.x - last_motion_x_, robot_pose.y - last_motion_y_);
    if (moved >= stop_motion_epsilon_)
    {
      last_motion_x_ = robot_pose.x;
      last_motion_y_ = robot_pose.y;
      last_motion_time_ = ros::Time::now();
      return false;
    }

    if ((ros::Time::now() - last_motion_time_).toSec() < stop_motion_timeout_)
    {
      return false;
    }

    ROS_WARN("Explorer: robot pose has not moved more than %.2fm for %.1fs; trying another frontier",
             stop_motion_epsilon_,
             stop_motion_timeout_);
    move_base_client_.cancelGoal();
    rememberRejectedGoal(active_goal_);
    has_active_goal_ = false;
    ++consecutive_stuck_replans_;
    motion_initialized_ = false;
    if (consecutive_stuck_replans_ >= max_stuck_replans_before_prompt_)
    {
      ROS_WARN("Explorer: %d consecutive goals got stuck; entering user decision prompt",
               consecutive_stuck_replans_);
      paused_ = true;
      publishState("stopped_over_10s");
      return true;
    }
    publishState("goal_stuck_replan");
    last_failure_time_ = ros::Time::now();
    return true;
  }

  void publishState(const std::string& state)
  {
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
  }

  void rememberRejectedGoal(const ExplorationGoal& goal)
  {
    RobotPose2D rejected;
    rejected.x = goal.pose.pose.position.x;
    rejected.y = goal.pose.pose.position.y;
    rejected_goals_.push_back(rejected);
    if (rejected_goals_.size() > 40)
    {
      rejected_goals_.erase(rejected_goals_.begin());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber command_sub_;
  ros::Publisher state_pub_;
  ros::ServiceClient make_plan_client_;
  ros::Timer timer_;
  tf::TransformListener tf_listener_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
  std::unique_ptr<ExplorationStrategy> strategy_;

  std::string map_frame_;
  std::string robot_base_frame_;
  std::string map_topic_;
  std::string strategy_name_;
  double planner_period_ = 5.0;
  double goal_timeout_ = 45.0;
  double no_progress_timeout_ = 10.0;
  double stop_motion_timeout_ = 10.0;
  double stop_motion_epsilon_ = 0.05;
  double progress_epsilon_ = 0.08;
  double make_plan_tolerance_ = 0.25;
  double completion_no_goal_duration_ = 18.0;
  double replan_after_failure_delay_ = 2.0;
  int max_goal_search_attempts_ = 8;
  int max_stuck_replans_before_prompt_ = 4;

  nav_msgs::OccupancyGrid latest_map_;
  bool have_map_ = false;
  bool has_active_goal_ = false;
  bool paused_ = false;
  bool manual_escape_active_ = false;
  bool ever_sent_goal_ = false;
  bool motion_initialized_ = false;
  int consecutive_stuck_replans_ = 0;
  ExplorationGoal active_goal_;
  ros::Time active_goal_sent_at_;
  ros::Time manual_escape_sent_at_;
  ros::Time last_progress_time_;
  ros::Time last_failure_time_;
  ros::Time no_goal_since_;
  ros::Time last_motion_time_;
  double last_motion_x_ = 0.0;
  double last_motion_y_ = 0.0;
  double best_active_goal_distance_ = std::numeric_limits<double>::infinity();
  bool have_progress_measurement_ = false;
  std::vector<RobotPose2D> rejected_goals_;
};

}  // namespace
}  // namespace ros_fyoyi

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autonomous_explorer");
  ros_fyoyi::AutonomousExplorer explorer;
  ros::spin();
  return 0;
}
