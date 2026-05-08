#include <actionlib/client/simple_action_client.h>
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

    loadWaypoints();
    publishAll();

    ROS_INFO("航点服务已启动，航点文件：%s", waypoints_file_.c_str());
    ROS_INFO("添加航点话题：%s，导航航点话题：%s", add_pose_topic_.c_str(), navi_waypoint_topic_.c_str());
  }

private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
      ROS_WARN("忽略无效的航点配置项：%s", err.what());
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
      ROS_WARN("航点文件不存在，将使用空航点列表：%s", waypoints_file_.c_str());
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
      ROS_INFO("已加载 %zu 个航点", waypoints_.size());
    }
    catch (const std::exception& err)
    {
      ROS_WARN("加载航点文件失败 %s：%s", waypoints_file_.c_str(), err.what());
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
    ROS_INFO("已保存 %zu 个航点到：%s", waypoints_.size(), waypoints_file_.c_str());
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
    ROS_INFO("已添加航点 %s：x=%.3f y=%.3f", name.c_str(), msg->pose.position.x, msg->pose.position.y);
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
      ROS_INFO("已删除航点：%s", name.c_str());
    }
    else
    {
      ROS_WARN("航点不存在：%s", name.c_str());
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
      ROS_WARN("重命名格式错误，应为：旧名字 新名字");
      return;
    }
    if (!isValidName(new_name))
    {
      ROS_WARN("航点名无效，只允许英文、数字、下划线和横线：%s", new_name.c_str());
      return;
    }

    bool renamed = false;
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      if (nameExistsLocked(new_name))
      {
        ROS_WARN("航点名已存在：%s", new_name.c_str());
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
      ROS_INFO("已重命名航点：%s -> %s", old_name.c_str(), new_name.c_str());
    }
    else
    {
      ROS_WARN("航点不存在：%s", old_name.c_str());
    }
  }

  void nextWaypointNameCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string name = msg->data;
    if (!isValidName(name))
    {
      ROS_WARN("航点名无效，只允许英文、数字、下划线和横线：%s", name.c_str());
      return;
    }
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      if (nameExistsLocked(name))
      {
        ROS_WARN("航点名已存在：%s", name.c_str());
        return;
      }
      next_custom_name_ = name;
    }
    ROS_INFO("下一个通过 RViz 添加的航点将命名为：%s", name.c_str());
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

  void naviWaypointCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string name = msg->data;
    if (name.empty())
    {
      ROS_WARN("航点名为空");
      return;
    }
    std::thread(&WaypointServer::navigateToWaypoint, this, name).detach();
  }

  void navigateToWaypoint(const std::string& name)
  {
    Waypoint waypoint;
    if (!findWaypoint(name, waypoint))
    {
      publishResult("失败:" + name + ":航点不存在");
      ROS_WARN("航点不存在：%s", name.c_str());
      return;
    }

    MoveBaseClient move_base_client(move_base_action_, true);

    ROS_INFO("等待 move_base action 服务：%s", move_base_action_.c_str());
    if (!move_base_client.waitForServer(ros::Duration(10.0)))
    {
      publishResult("失败:" + name + ":move_base未启动");
      ROS_ERROR("move_base action 服务未启动");
      return;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = waypoint.frame_id;
    goal.target_pose.pose = waypoint.pose;

    ROS_INFO("开始导航到航点：%s", name.c_str());
    move_base_client.sendGoal(goal);
    move_base_client.waitForResult();

    const actionlib::SimpleClientGoalState state = move_base_client.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      publishResult("完成:" + name);
      ROS_INFO("已到达航点：%s", name.c_str());
    }
    else
    {
      std::ostringstream result;
      result << "失败:" << name << ":state=" << state.toString();
      publishResult(result.str());
      ROS_WARN("导航失败：%s，状态：%s", name.c_str(), state.toString().c_str());
    }
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
  ros::Subscriber delete_waypoint_sub_;
  ros::Subscriber rename_waypoint_sub_;
  ros::Subscriber next_waypoint_name_sub_;
  ros::Subscriber save_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher list_pub_;
  ros::Publisher result_pub_;
  ros::ServiceServer save_srv_;
  ros::ServiceServer reload_srv_;
  ros::ServiceServer clear_srv_;
  ros::ServiceServer list_srv_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  interactive_markers::MenuHandler menu_handler_;

  std::recursive_mutex mutex_;
  std::vector<Waypoint> waypoints_;
  std::string next_custom_name_;

  std::string map_frame_;
  std::string waypoints_file_;
  std::string add_pose_topic_;
  std::string clicked_point_topic_;
  bool enable_clicked_point_;
  std::string navi_waypoint_topic_;
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
