#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct Waypoint
{
  std::string name;
  std::string frame_id;
  geometry_msgs::Pose pose;
};

class WaypointServer
{
public:
  WaypointServer()
    : nh_()
    , pnh_("~")
  {
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("waypoints_file", waypoints_file_, defaultWaypointsFile());
    pnh_.param<std::string>("add_pose_topic", add_pose_topic_, "/fyoyi/add_waypoint");
    pnh_.param<std::string>("clicked_point_topic", clicked_point_topic_, "/clicked_point");
    pnh_.param("enable_clicked_point", enable_clicked_point_, false);
    pnh_.param<std::string>("navi_waypoint_topic", navi_waypoint_topic_, "/fyoyi/navi_waypoint");
    pnh_.param<std::string>("patrol_waypoints_topic", patrol_waypoints_topic_, "/fyoyi/patrol_waypoints");
    pnh_.param<std::string>("cancel_navigation_topic", cancel_navigation_topic_, "/fyoyi/cancel_navigation");
    pnh_.param<std::string>("pause_patrol_topic", pause_patrol_topic_, "/fyoyi/pause_patrol");
    pnh_.param<std::string>("resume_patrol_topic", resume_patrol_topic_, "/fyoyi/resume_patrol");
    pnh_.param<std::string>("delete_waypoint_topic", delete_waypoint_topic_, "/fyoyi/delete_waypoint");
    pnh_.param<std::string>("rename_waypoint_topic", rename_waypoint_topic_, "/fyoyi/rename_waypoint");
    pnh_.param<std::string>("next_waypoint_name_topic", next_waypoint_name_topic_, "/fyoyi/next_waypoint_name");
    pnh_.param<std::string>("save_topic", save_topic_, "/fyoyi/save_waypoints_cmd");
    pnh_.param<std::string>("marker_topic", marker_topic_, "/fyoyi/waypoints_marker");
    pnh_.param<std::string>("list_topic", list_topic_, "/fyoyi/waypoint_list");
    pnh_.param<std::string>("result_topic", result_topic_, "/fyoyi/navi_result");
    pnh_.param<std::string>("move_base_action", move_base_action_, "move_base");
    pnh_.param<std::string>("auto_name_prefix", auto_name_prefix_, "wp");
    pnh_.param("default_yaw", default_yaw_, 0.0);
    pnh_.param("auto_save", auto_save_, true);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    list_pub_ = nh_.advertise<std_msgs::String>(list_topic_, 1, true);
    result_pub_ = nh_.advertise<std_msgs::String>(result_topic_, 10);

    interactive_server_.reset(new interactive_markers::InteractiveMarkerServer("fyoyi_waypoints_interactive"));
    menu_handler_.insert("删除这个航点", boost::bind(&WaypointServer::interactiveDeleteCallback, this, _1));

    add_pose_sub_ = nh_.subscribe(add_pose_topic_, 10, &WaypointServer::addPoseCallback, this);
    navi_waypoint_sub_ = nh_.subscribe(navi_waypoint_topic_, 10, &WaypointServer::naviWaypointCallback, this);
    patrol_waypoints_sub_ = nh_.subscribe(patrol_waypoints_topic_, 10, &WaypointServer::patrolWaypointsCallback, this);
    cancel_navigation_sub_ = nh_.subscribe(cancel_navigation_topic_, 10, &WaypointServer::cancelNavigationCallback, this);
    pause_patrol_sub_ = nh_.subscribe(pause_patrol_topic_, 10, &WaypointServer::pausePatrolCallback, this);
    resume_patrol_sub_ = nh_.subscribe(resume_patrol_topic_, 10, &WaypointServer::resumePatrolCallback, this);
    delete_waypoint_sub_ = nh_.subscribe(delete_waypoint_topic_, 10, &WaypointServer::deleteWaypointCallback, this);
    rename_waypoint_sub_ = nh_.subscribe(rename_waypoint_topic_, 10, &WaypointServer::renameWaypointCallback, this);
    next_waypoint_name_sub_ = nh_.subscribe(next_waypoint_name_topic_, 10, &WaypointServer::nextWaypointNameCallback, this);
    save_sub_ = nh_.subscribe(save_topic_, 10, &WaypointServer::saveTopicCallback, this);
    if (enable_clicked_point_)
    {
      clicked_point_sub_ = nh_.subscribe(clicked_point_topic_, 10, &WaypointServer::clickedPointCallback, this);
    }

    save_srv_ = nh_.advertiseService("/fyoyi/save_waypoints", &WaypointServer::saveServiceCallback, this);
    reload_srv_ = nh_.advertiseService("/fyoyi/reload_waypoints", &WaypointServer::reloadServiceCallback, this);
    clear_srv_ = nh_.advertiseService("/fyoyi/clear_waypoints", &WaypointServer::clearServiceCallback, this);
    list_srv_ = nh_.advertiseService("/fyoyi/list_waypoints", &WaypointServer::listServiceCallback, this);
    status_srv_ = nh_.advertiseService("/fyoyi/status", &WaypointServer::statusServiceCallback, this);

    loadWaypoints();
    publishAll();

    ROS_INFO("waypoint_server started, waypoints file: %s", waypoints_file_.c_str());
    ROS_INFO("add waypoint topic: %s, navigation topic: %s", add_pose_topic_.c_str(), navi_waypoint_topic_.c_str());
  }

private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  enum class NavOutcome
  {
    SUCCEEDED,
    FAILED,
    CANCELED
  };

  std::string defaultWaypointsFile() const
  {
    const std::string package_path = ros::package::getPath("ros_fyoyi");
    if (!package_path.empty())
    {
      return package_path + "/config/waypoints.yaml";
    }
    return std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/.ros/ros_fyoyi_waypoints.yaml";
  }

  bool isValidName(const std::string& name) const
  {
    static const std::regex pattern("^[A-Za-z0-9_-]+$");
    return std::regex_match(name, pattern);
  }

  bool nameExistsLocked(const std::string& name) const
  {
    for (const auto& waypoint : waypoints_)
    {
      if (waypoint.name == name)
      {
        return true;
      }
    }
    return false;
  }

  std::string nextWaypointName()
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    for (int index = 1;; ++index)
    {
      std::ostringstream name;
      name << auto_name_prefix_ << "_" << index;
      if (!nameExistsLocked(name.str()))
      {
        return name.str();
      }
    }
  }

  std::string consumeNextWaypointName()
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (!next_custom_name_.empty())
    {
      const std::string name = next_custom_name_;
      next_custom_name_.clear();
      return name;
    }
    return nextWaypointName();
  }

  geometry_msgs::Quaternion yawToQuaternion(double yaw) const
  {
    geometry_msgs::Quaternion quat;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = std::sin(yaw * 0.5);
    quat.w = std::cos(yaw * 0.5);
    return quat;
  }

  double quaternionToYawDeg(const geometry_msgs::Quaternion& quat) const
  {
    const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
  }

  bool findWaypoint(const std::string& name, Waypoint& result)
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    for (const auto& waypoint : waypoints_)
    {
      if (waypoint.name == name)
      {
        result = waypoint;
        return true;
      }
    }
    return false;
  }

  std::vector<std::string> splitWords(const std::string& text) const
  {
    std::vector<std::string> words;
    std::istringstream input(text);
    std::string word;
    while (input >> word)
    {
      words.push_back(word);
    }
    return words;
  }

  Waypoint poseMsgToWaypoint(const std::string& name, const geometry_msgs::PoseStamped& msg) const
  {
    Waypoint waypoint;
    waypoint.name = name;
    waypoint.frame_id = msg.header.frame_id.empty() ? map_frame_ : msg.header.frame_id;
    waypoint.pose = msg.pose;
    return waypoint;
  }

  bool parseWaypoint(const YAML::Node& node, Waypoint& waypoint)
  {
    try
    {
      waypoint.name = node["name"].as<std::string>();
      waypoint.frame_id = node["frame_id"] ? node["frame_id"].as<std::string>() : map_frame_;

      const YAML::Node position = node["position"];
      waypoint.pose.position.x = position["x"].as<double>();
      waypoint.pose.position.y = position["y"].as<double>();
      waypoint.pose.position.z = position["z"] ? position["z"].as<double>() : 0.0;

      const YAML::Node orientation = node["orientation"];
      waypoint.pose.orientation.x = orientation["x"] ? orientation["x"].as<double>() : 0.0;
      waypoint.pose.orientation.y = orientation["y"] ? orientation["y"].as<double>() : 0.0;
      waypoint.pose.orientation.z = orientation["z"] ? orientation["z"].as<double>() : 0.0;
      waypoint.pose.orientation.w = orientation["w"] ? orientation["w"].as<double>() : 1.0;
      return true;
    }
    catch (const std::exception& err)
    {
      ROS_WARN("ignore invalid waypoint yaml item: %s", err.what());
      return false;
    }
  }

  void loadWaypoints()
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    waypoints_.clear();

    std::ifstream input(waypoints_file_);
    if (!input.good())
    {
      ROS_WARN("waypoints file does not exist, start with empty list: %s", waypoints_file_.c_str());
      return;
    }

    try
    {
      const YAML::Node root = YAML::LoadFile(waypoints_file_);
      const YAML::Node items = root["waypoints"];
      if (items && items.IsSequence())
      {
        for (const auto& item : items)
        {
          Waypoint waypoint;
          if (parseWaypoint(item, waypoint))
          {
            waypoints_.push_back(waypoint);
          }
        }
      }
      ROS_INFO("loaded %zu waypoints", waypoints_.size());
    }
    catch (const std::exception& err)
    {
      ROS_WARN("failed to load waypoints file %s: %s", waypoints_file_.c_str(), err.what());
    }
  }

  void saveWaypoints()
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
    for (const auto& waypoint : waypoints_)
    {
      out << YAML::BeginMap;
      out << YAML::Key << "name" << YAML::Value << waypoint.name;
      out << YAML::Key << "frame_id" << YAML::Value << waypoint.frame_id;
      out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << waypoint.pose.position.x;
      out << YAML::Key << "y" << YAML::Value << waypoint.pose.position.y;
      out << YAML::Key << "z" << YAML::Value << waypoint.pose.position.z;
      out << YAML::EndMap;
      out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << waypoint.pose.orientation.x;
      out << YAML::Key << "y" << YAML::Value << waypoint.pose.orientation.y;
      out << YAML::Key << "z" << YAML::Value << waypoint.pose.orientation.z;
      out << YAML::Key << "w" << YAML::Value << waypoint.pose.orientation.w;
      out << YAML::EndMap;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream output(waypoints_file_);
    output << out.c_str() << "\n";
    ROS_INFO("saved %zu waypoints to: %s", waypoints_.size(), waypoints_file_.c_str());
  }

  void saveIfNeeded()
  {
    if (auto_save_)
    {
      saveWaypoints();
    }
  }

  void addPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    const std::string name = consumeNextWaypointName();
    const Waypoint waypoint = poseMsgToWaypoint(name, *msg);
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      waypoints_.push_back(waypoint);
    }
    publishAll();
    saveIfNeeded();
    ROS_INFO("added waypoint %s: x=%.3f y=%.3f", name.c_str(), msg->pose.position.x, msg->pose.position.y);
  }

  void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position.x = msg->point.x;
    pose.pose.position.y = msg->point.y;
    pose.pose.position.z = msg->point.z;
    pose.pose.orientation = yawToQuaternion(default_yaw_);
    addPoseCallback(boost::make_shared<geometry_msgs::PoseStamped const>(pose));
  }

  void deleteWaypointCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string name = msg->data;
    if (name.empty())
    {
      return;
    }

    bool deleted = false;
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      auto it = waypoints_.begin();
      while (it != waypoints_.end())
      {
        if (it->name == name)
        {
          it = waypoints_.erase(it);
          deleted = true;
        }
        else
        {
          ++it;
        }
      }
    }

    publishAll();
    if (deleted)
    {
      saveIfNeeded();
      ROS_INFO("deleted waypoint: %s", name.c_str());
    }
    else
    {
      ROS_WARN("waypoint not found: %s", name.c_str());
    }
  }

  void renameWaypointCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::istringstream input(msg->data);
    std::string old_name;
    std::string new_name;
    input >> old_name >> new_name;
    if (old_name.empty() || new_name.empty())
    {
      ROS_WARN("rename format error, expected: old_name new_name");
      return;
    }
    if (!isValidName(new_name))
    {
      ROS_WARN("invalid waypoint name, only letters, digits, _ and - are allowed: %s", new_name.c_str());
      return;
    }

    bool renamed = false;
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      if (nameExistsLocked(new_name))
      {
        ROS_WARN("waypoint name already exists: %s", new_name.c_str());
        return;
      }
      for (auto& waypoint : waypoints_)
      {
        if (waypoint.name == old_name)
        {
          waypoint.name = new_name;
          renamed = true;
          break;
        }
      }
    }

    if (renamed)
    {
      publishAll();
      saveIfNeeded();
      ROS_INFO("renamed waypoint: %s -> %s", old_name.c_str(), new_name.c_str());
    }
    else
    {
      ROS_WARN("waypoint not found: %s", old_name.c_str());
    }
  }

  void nextWaypointNameCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string name = msg->data;
    if (!isValidName(name))
    {
      ROS_WARN("invalid waypoint name, only letters, digits, _ and - are allowed: %s", name.c_str());
      return;
    }
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      if (nameExistsLocked(name))
      {
        ROS_WARN("waypoint name already exists: %s", name.c_str());
        return;
      }
      next_custom_name_ = name;
    }
    ROS_INFO("next RViz waypoint name: %s", name.c_str());
  }

  void interactiveDeleteCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    const std::string prefix = "wp::";
    if (feedback->marker_name.find(prefix) != 0)
    {
      return;
    }
    std_msgs::String msg;
    msg.data = feedback->marker_name.substr(prefix.size());
    deleteWaypointCallback(boost::make_shared<std_msgs::String const>(msg));
  }

  void saveTopicCallback(const std_msgs::Empty::ConstPtr&)
  {
    saveWaypoints();
  }

  bool saveServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    saveWaypoints();
    res.success = true;
    res.message = "航点已保存到 " + waypoints_file_;
    return true;
  }

  bool reloadServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    loadWaypoints();
    publishAll();
    res.success = true;
    res.message = "航点已重新加载";
    return true;
  }

  bool clearServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      waypoints_.clear();
    }
    publishAll();
    saveIfNeeded();
    res.success = true;
    res.message = "航点已清空";
    return true;
  }

  bool listServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    res.success = true;
    res.message = buildWaypointListText();
    return true;
  }

  bool statusServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::lock_guard<std::mutex> guard(task_mutex_);
    res.success = true;
    res.message = buildStatusTextLocked();
    return true;
  }

  void naviWaypointCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string name = msg->data;
    if (name.empty())
    {
      ROS_WARN("empty waypoint name");
      return;
    }
    const uint64_t task_id = startTask("单点导航", name, std::vector<std::string>{ name }, false);
    std::thread(&WaypointServer::singleNavigationThread, this, name, task_id).detach();
  }

  void patrolWaypointsCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::vector<std::string> words = splitWords(msg->data);
    bool loop = false;
    if (!words.empty() && words.front() == "--loop")
    {
      loop = true;
      words.erase(words.begin());
    }

    if (words.empty())
    {
      publishResult("失败:巡逻:航点队列为空");
      ROS_WARN("empty patrol waypoint list");
      return;
    }

    for (const auto& name : words)
    {
      Waypoint waypoint;
      if (!findWaypoint(name, waypoint))
      {
        publishResult("失败:巡逻:航点不存在:" + name);
        ROS_WARN("patrol waypoint not found: %s", name.c_str());
        return;
      }
    }

    const uint64_t task_id = startTask(loop ? "循环巡逻" : "队列导航", words.front(), words, loop);
    std::thread(&WaypointServer::patrolThread, this, words, loop, task_id).detach();
  }

  void cancelNavigationCallback(const std_msgs::String::ConstPtr&)
  {
    requestCancel("用户取消导航");
  }

  void pausePatrolCallback(const std_msgs::String::ConstPtr&)
  {
    std::lock_guard<std::mutex> guard(task_mutex_);
    if (task_mode_ == "队列导航" || task_mode_ == "循环巡逻" || task_mode_ == "暂停")
    {
      paused_ = true;
      task_mode_ = "暂停";
      last_result_ = "巡逻已暂停";
      publishResult(last_result_);
      ROS_INFO("patrol paused");
    }
    else
    {
      publishResult("失败:暂停:当前没有巡逻任务");
      ROS_WARN("no patrol task to pause");
    }
  }

  void resumePatrolCallback(const std_msgs::String::ConstPtr&)
  {
    std::lock_guard<std::mutex> guard(task_mutex_);
    if (paused_)
    {
      paused_ = false;
      task_mode_ = patrol_loop_ ? "循环巡逻" : "队列导航";
      last_result_ = "巡逻已继续";
      publishResult(last_result_);
      ROS_INFO("patrol resumed");
    }
    else
    {
      publishResult("失败:继续:当前没有暂停的巡逻任务");
      ROS_WARN("no paused patrol task to resume");
    }
  }

  uint64_t startTask(const std::string& mode, const std::string& current_goal,
                     const std::vector<std::string>& queue, bool loop)
  {
    cancelMoveBaseGoals();
    std::lock_guard<std::mutex> guard(task_mutex_);
    ++task_generation_;
    cancel_requested_ = false;
    paused_ = false;
    task_mode_ = mode;
    current_goal_ = current_goal;
    patrol_queue_ = queue;
    patrol_index_ = 0;
    patrol_loop_ = loop;
    last_result_ = "任务已启动";
    return task_generation_;
  }

  void requestCancel(const std::string& reason)
  {
    {
      std::lock_guard<std::mutex> guard(task_mutex_);
      cancel_requested_ = true;
      paused_ = false;
      task_mode_ = "空闲";
      current_goal_.clear();
      patrol_queue_.clear();
      patrol_index_ = 0;
      patrol_loop_ = false;
      last_result_ = reason;
    }
    cancelMoveBaseGoals();
    publishResult(reason);
    ROS_INFO("navigation canceled");
  }

  void singleNavigationThread(const std::string& name, uint64_t task_id)
  {
    const NavOutcome outcome = navigateToWaypoint(name, task_id, false);
    std::lock_guard<std::mutex> guard(task_mutex_);
    if (task_id == task_generation_ && outcome != NavOutcome::CANCELED)
    {
      task_mode_ = "空闲";
      current_goal_.clear();
    }
  }

  void patrolThread(const std::vector<std::string>& names, bool loop, uint64_t task_id)
  {
    std::size_t index = 0;
    do
    {
      for (; index < names.size(); ++index)
      {
        if (isTaskStopped(task_id))
        {
          return;
        }

        {
          std::lock_guard<std::mutex> guard(task_mutex_);
          current_goal_ = names[index];
          patrol_index_ = index;
        }

        const NavOutcome outcome = navigateToWaypoint(names[index], task_id, true);
        if (outcome == NavOutcome::CANCELED)
        {
          return;
        }
        if (outcome == NavOutcome::FAILED)
        {
          std::lock_guard<std::mutex> guard(task_mutex_);
          if (task_id == task_generation_)
          {
            task_mode_ = "空闲";
            current_goal_.clear();
            patrol_queue_.clear();
            patrol_index_ = 0;
            patrol_loop_ = false;
          }
          return;
        }
      }
      index = 0;
    } while (loop && ros::ok() && !isTaskStopped(task_id));

    std::lock_guard<std::mutex> guard(task_mutex_);
    if (task_id == task_generation_)
    {
      task_mode_ = "空闲";
      current_goal_.clear();
      patrol_queue_.clear();
      patrol_index_ = 0;
      patrol_loop_ = false;
      last_result_ = loop ? "循环巡逻已结束" : "队列导航已完成";
      publishResult(last_result_);
    }
  }

  NavOutcome navigateToWaypoint(const std::string& name, uint64_t task_id, bool allow_pause)
  {
    Waypoint waypoint;
    if (!findWaypoint(name, waypoint))
    {
      publishResult("失败:" + name + ":航点不存在");
      ROS_WARN("waypoint not found: %s", name.c_str());
      return NavOutcome::FAILED;
    }

    MoveBaseClient move_base_client(move_base_action_, true);

    ROS_INFO("waiting for move_base action server: %s", move_base_action_.c_str());
    if (!move_base_client.waitForServer(ros::Duration(10.0)))
    {
      publishResult("失败:" + name + ":move_base未启动");
      ROS_ERROR("move_base action server is not ready");
      setLastResult("失败:" + name + ":move_base未启动");
      return NavOutcome::FAILED;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = waypoint.frame_id;
    goal.target_pose.pose = waypoint.pose;

    while (ros::ok())
    {
      bool restart_current_goal = false;

      if (isTaskStopped(task_id))
      {
        move_base_client.cancelGoal();
        setLastResult("已取消:" + name);
        return NavOutcome::CANCELED;
      }

      ROS_INFO("navigating to waypoint: %s", name.c_str());
      move_base_client.sendGoal(goal);
      while (ros::ok() && !move_base_client.waitForResult(ros::Duration(0.2)))
      {
        if (isTaskStopped(task_id))
        {
          move_base_client.cancelGoal();
          setLastResult("已取消:" + name);
          return NavOutcome::CANCELED;
        }
        if (allow_pause && isTaskPaused(task_id))
        {
          move_base_client.cancelGoal();
          publishResult("暂停:" + name);
          ROS_INFO("paused while navigating to waypoint: %s", name.c_str());
          while (ros::ok() && isTaskPaused(task_id) && !isTaskStopped(task_id))
          {
            ros::WallDuration(0.1).sleep();
          }
          if (isTaskStopped(task_id))
          {
            setLastResult("已取消:" + name);
            return NavOutcome::CANCELED;
          }
          ROS_INFO("resume waypoint navigation: %s", name.c_str());
          restart_current_goal = true;
          break;
        }
      }

      if (restart_current_goal || (allow_pause && isTaskPaused(task_id)))
      {
        continue;
      }

      const actionlib::SimpleClientGoalState state = move_base_client.getState();
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        const std::string result = "完成:" + name;
        publishResult(result);
        setLastResult(result);
        ROS_INFO("arrived waypoint: %s", name.c_str());
        return NavOutcome::SUCCEEDED;
      }

      std::ostringstream result;
      result << "失败:" << name << ":state=" << state.toString();
      publishResult(result.str());
      setLastResult(result.str());
      ROS_WARN("navigation failed: %s, state: %s", name.c_str(), state.toString().c_str());
      return NavOutcome::FAILED;
    }

    return NavOutcome::CANCELED;
  }

  bool isTaskStopped(uint64_t task_id)
  {
    std::lock_guard<std::mutex> guard(task_mutex_);
    return cancel_requested_ || task_id != task_generation_;
  }

  bool isTaskPaused(uint64_t task_id)
  {
    std::lock_guard<std::mutex> guard(task_mutex_);
    return paused_ && task_id == task_generation_;
  }

  void setLastResult(const std::string& result)
  {
    std::lock_guard<std::mutex> guard(task_mutex_);
    last_result_ = result;
  }

  void cancelMoveBaseGoals()
  {
    actionlib_msgs::GoalID cancel_msg;
    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>(move_base_action_ + "/cancel", 1);
    ros::WallDuration(0.1).sleep();
    cancel_pub_.publish(cancel_msg);
  }

  std::string buildStatusTextLocked() const
  {
    std::ostringstream output;
    output << "当前状态：" << task_mode_;
    if (!current_goal_.empty())
    {
      output << "\n当前目标：" << current_goal_;
    }
    if (!patrol_queue_.empty())
    {
      output << "\n巡逻队列：";
      for (std::size_t i = 0; i < patrol_queue_.size(); ++i)
      {
        if (i > 0)
        {
          output << " -> ";
        }
        if (i == patrol_index_)
        {
          output << "[" << patrol_queue_[i] << "]";
        }
        else
        {
          output << patrol_queue_[i];
        }
      }
      output << "\n循环巡逻：" << (patrol_loop_ ? "是" : "否");
    }
    output << "\n上次结果：" << (last_result_.empty() ? "无" : last_result_);
    return output.str();
  }

  void publishResult(const std::string& text)
  {
    std_msgs::String msg;
    msg.data = text;
    result_pub_.publish(msg);
  }

  void publishAll()
  {
    publishMarkers();
    publishInteractiveMarkers();
    std_msgs::String list_msg;
    list_msg.data = buildWaypointListText();
    list_pub_.publish(list_msg);
  }

  void publishMarkers()
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    std::vector<Waypoint> waypoints;
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      waypoints = waypoints_;
    }

    for (std::size_t index = 0; index < waypoints.size(); ++index)
    {
      const Waypoint& waypoint = waypoints[index];

      visualization_msgs::Marker arrow;
      arrow.header.frame_id = waypoint.frame_id;
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "fyoyi_waypoint_arrow";
      arrow.id = static_cast<int>(index);
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.pose = waypoint.pose;
      arrow.scale.x = 0.45;
      arrow.scale.y = 0.08;
      arrow.scale.z = 0.08;
      arrow.color.r = 0.1;
      arrow.color.g = 0.7;
      arrow.color.b = 1.0;
      arrow.color.a = 0.95;
      marker_array.markers.push_back(arrow);

      visualization_msgs::Marker text;
      text.header.frame_id = waypoint.frame_id;
      text.header.stamp = ros::Time::now();
      text.ns = "fyoyi_waypoint_text";
      text.id = static_cast<int>(index + 10000);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position.x = waypoint.pose.position.x;
      text.pose.position.y = waypoint.pose.position.y;
      text.pose.position.z = waypoint.pose.position.z + 0.45;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.28;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      text.text = waypoint.name;
      marker_array.markers.push_back(text);
    }

    marker_pub_.publish(marker_array);
  }

  void publishInteractiveMarkers()
  {
    interactive_server_->clear();

    std::vector<Waypoint> waypoints;
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      waypoints = waypoints_;
    }

    for (const auto& waypoint : waypoints)
    {
      visualization_msgs::InteractiveMarker marker;
      marker.header.frame_id = waypoint.frame_id;
      marker.header.stamp = ros::Time::now();
      marker.name = "wp::" + waypoint.name;
      marker.description = waypoint.name;
      marker.pose = waypoint.pose;
      marker.scale = 0.45;

      visualization_msgs::Marker visual;
      visual.type = visualization_msgs::Marker::ARROW;
      visual.scale.x = 0.45;
      visual.scale.y = 0.08;
      visual.scale.z = 0.08;
      visual.color.r = 0.1;
      visual.color.g = 0.7;
      visual.color.b = 1.0;
      visual.color.a = 0.65;

      visualization_msgs::InteractiveMarkerControl control;
      control.always_visible = true;
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
      control.markers.push_back(visual);
      marker.controls.push_back(control);

      interactive_server_->insert(marker);
      menu_handler_.apply(*interactive_server_, marker.name);
    }

    interactive_server_->applyChanges();
  }

  std::string buildWaypointListText()
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (waypoints_.empty())
    {
      return "当前没有航点";
    }

    std::ostringstream output;
    for (std::size_t index = 0; index < waypoints_.size(); ++index)
    {
      const Waypoint& waypoint = waypoints_[index];
      output << waypoint.name << ": x=" << fixed3(waypoint.pose.position.x) << ", y=" << fixed3(waypoint.pose.position.y)
             << ", yaw=" << fixed1(quaternionToYawDeg(waypoint.pose.orientation)) << " deg, frame=" << waypoint.frame_id;
      if (index + 1 < waypoints_.size())
      {
        output << "\n";
      }
    }
    return output.str();
  }

  std::string fixed3(double value) const
  {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);
    out << value;
    return out.str();
  }

  std::string fixed1(double value) const
  {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(1);
    out << value;
    return out.str();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber add_pose_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber navi_waypoint_sub_;
  ros::Subscriber patrol_waypoints_sub_;
  ros::Subscriber cancel_navigation_sub_;
  ros::Subscriber pause_patrol_sub_;
  ros::Subscriber resume_patrol_sub_;
  ros::Subscriber delete_waypoint_sub_;
  ros::Subscriber rename_waypoint_sub_;
  ros::Subscriber next_waypoint_name_sub_;
  ros::Subscriber save_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher list_pub_;
  ros::Publisher result_pub_;
  ros::Publisher cancel_pub_;
  ros::ServiceServer save_srv_;
  ros::ServiceServer reload_srv_;
  ros::ServiceServer clear_srv_;
  ros::ServiceServer list_srv_;
  ros::ServiceServer status_srv_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  interactive_markers::MenuHandler menu_handler_;

  std::recursive_mutex mutex_;
  std::mutex task_mutex_;
  std::vector<Waypoint> waypoints_;
  std::string next_custom_name_;
  uint64_t task_generation_ = 0;
  bool cancel_requested_ = false;
  bool paused_ = false;
  std::string task_mode_ = "空闲";
  std::string current_goal_;
  std::vector<std::string> patrol_queue_;
  std::size_t patrol_index_ = 0;
  bool patrol_loop_ = false;
  std::string last_result_ = "无";

  std::string map_frame_;
  std::string waypoints_file_;
  std::string add_pose_topic_;
  std::string clicked_point_topic_;
  bool enable_clicked_point_;
  std::string navi_waypoint_topic_;
  std::string patrol_waypoints_topic_;
  std::string cancel_navigation_topic_;
  std::string pause_patrol_topic_;
  std::string resume_patrol_topic_;
  std::string delete_waypoint_topic_;
  std::string rename_waypoint_topic_;
  std::string next_waypoint_name_topic_;
  std::string save_topic_;
  std::string marker_topic_;
  std::string list_topic_;
  std::string result_topic_;
  std::string move_base_action_;
  std::string auto_name_prefix_;
  double default_yaw_;
  bool auto_save_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_server");
  WaypointServer server;
  ros::spin();
  return 0;
}
