#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace
{

const char* COLOR_RED = "\033[1;31m";
const char* COLOR_GREEN = "\033[1;32m";
const char* COLOR_YELLOW = "\033[1;33m";
const char* COLOR_CYAN = "\033[1;36m";
const char* COLOR_RESET = "\033[0m";

enum class PromptAction
{
  Save,
  ContinueMapping
};

struct PromptResult
{
  PromptAction action = PromptAction::Save;
  std::string map_path;
  double escape_x = 0.0;
  double escape_y = 0.0;
  double escape_yaw = 0.0;
};

std::string expandHome(const std::string& path)
{
  if (path.empty() || path[0] != '~')
  {
    return path;
  }
  const char* home = std::getenv("HOME");
  if (!home)
  {
    return path;
  }
  if (path.size() == 1)
  {
    return std::string(home);
  }
  if (path[1] == '/')
  {
    return std::string(home) + path.substr(1);
  }
  return path;
}

std::string currentTimestamp()
{
  std::time_t now = std::time(nullptr);
  std::tm local_time;
  localtime_r(&now, &local_time);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
  return buffer;
}

std::string sanitizeFolderName(const std::string& raw)
{
  std::string clean;
  for (char ch : raw)
  {
    const unsigned char uch = static_cast<unsigned char>(ch);
    if (std::isalnum(uch) || ch == '-' || ch == '_')
    {
      clean.push_back(ch);
    }
    else
    {
      clean.push_back('_');
    }
  }
  return clean.empty() ? currentTimestamp() : clean;
}

std::string dirnameOf(const std::string& path)
{
  const std::string::size_type pos = path.find_last_of('/');
  if (pos == std::string::npos)
  {
    return "";
  }
  if (pos == 0)
  {
    return "/";
  }
  return path.substr(0, pos);
}

std::string normalizeMapPath(const std::string& raw)
{
  std::string path = expandHome(raw);
  if (path.size() >= 5 && path.substr(path.size() - 5) == ".yaml")
  {
    return path.substr(0, path.size() - 5);
  }
  if (path.size() >= 4 && path.substr(path.size() - 4) == ".pgm")
  {
    return path.substr(0, path.size() - 4);
  }
  const std::string::size_type pos = path.find_last_of('/');
  if (pos != std::string::npos && path.substr(pos + 1) == "map")
  {
    return path;
  }
  return path + "/map";
}

bool makeDirectory(const std::string& path)
{
  if (path.empty())
  {
    return true;
  }
  const pid_t pid = fork();
  if (pid == 0)
  {
    execlp("mkdir", "mkdir", "-p", path.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  if (pid < 0 || waitpid(pid, &status, 0) < 0)
  {
    return false;
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

bool runMapSaver(const std::string& map_path)
{
  const pid_t pid = fork();
  if (pid == 0)
  {
    execlp("rosrun", "rosrun", "map_server", "map_saver", "-f", map_path.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  if (pid < 0 || waitpid(pid, &status, 0) < 0)
  {
    return false;
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

class AutoMapSaver
{
public:
  AutoMapSaver()
    : private_nh_("~")
  {
    private_nh_.param("default_root", default_root_, std::string("/home/fyoyi/catkin_ws/src/ros_fyoyi/maps/default"));
    private_nh_.param("folder_name", folder_name_, std::string(""));
    private_nh_.param("map_path", map_path_override_, std::string(""));
    private_nh_.param("prompt_on_complete", prompt_on_complete_, true);
    private_nh_.param("frontier_threshold", frontier_threshold_, 30);
    private_nh_.param("stable_duration", stable_duration_, 12.0);
    private_nh_.param("no_goal_stable_duration", no_goal_stable_duration_, 6.0);
    private_nh_.param("min_runtime", min_runtime_, 45.0);
    private_nh_.param("min_free_cells", min_free_cells_, 200);
    private_nh_.param("stop_robot", stop_robot_, true);
    double check_period = 2.0;
    private_nh_.param("check_period", check_period, 2.0);

    default_root_ = expandHome(default_root_);
    map_path_override_ = expandHome(map_path_override_);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    explorer_command_pub_ = nh_.advertise<std_msgs::String>("/ros_fyoyi/explorer_command", 1);
    map_sub_ = nh_.subscribe("/map", 1, &AutoMapSaver::mapCallback, this);
    explorer_state_sub_ = nh_.subscribe("/ros_fyoyi/explorer_state", 1, &AutoMapSaver::explorerStateCallback, this);
    command_sub_ = nh_.subscribe("/ros_fyoyi/map_saver_command", 1, &AutoMapSaver::commandCallback, this);
    timer_ = nh_.createTimer(ros::Duration(check_period), &AutoMapSaver::checkMap, this);

    ROS_INFO("MapSaver: C++ node started, default_root=%s", default_root_.c_str());
  }

private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    latest_map_ = *msg;
    have_map_ = true;
  }

  void explorerStateCallback(const std_msgs::String::ConstPtr& msg)
  {
    latest_explorer_state_ = msg->data;
    if (!saved_ && !prompting_ && latest_explorer_state_ == "stopped_over_10s")
    {
      ROS_WARN("MapSaver: robot stopped for more than 10s; entering user decision prompt");
      finishAndSave();
      return;
    }
    if (!saved_ && !prompting_ && latest_explorer_state_ == "exploration_done")
    {
      ROS_WARN("MapSaver: explorer reported no reachable frontier; entering user decision prompt");
      finishAndSave();
      return;
    }

    no_goal_complete_since_ = ros::Time();
  }

  void commandCallback(const std_msgs::String::ConstPtr& msg)
  {
    if (saved_ || prompting_)
    {
      return;
    }

    const std::string command = msg->data;
    if (command == "prompt" || command == "save" || command == "ask")
    {
      ROS_WARN("MapSaver: user requested save prompt via /ros_fyoyi/map_saver_command");
      finishAndSave();
      return;
    }

    ROS_WARN("MapSaver: unknown command '%s' on /ros_fyoyi/map_saver_command", command.c_str());
  }

  void checkMap(const ros::TimerEvent&)
  {
    if (saved_ || prompting_)
    {
      return;
    }
    if (!have_map_)
    {
      ROS_INFO_THROTTLE(10.0, "MapSaver: waiting for /map");
      return;
    }

    int frontier_count = 0;
    int free_count = 0;
    countFrontiers(latest_map_, &frontier_count, &free_count);
    const double runtime = (ros::Time::now() - started_at_).toSec();

    ROS_INFO_THROTTLE(10.0,
                      "MapSaver: monitoring, frontier_cells=%d free_cells=%d runtime=%.1fs explorer=%s",
                      frontier_count,
                      free_count,
                      runtime,
                      latest_explorer_state_.c_str());

    clear_since_ = ros::Time();
  }

  void countFrontiers(const nav_msgs::OccupancyGrid& map, int* frontier_count, int* free_count) const
  {
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    *frontier_count = 0;
    *free_count = 0;
    if (width < 3 || height < 3)
    {
      return;
    }

    for (int y = 1; y < height - 1; ++y)
    {
      for (int x = 1; x < width - 1; ++x)
      {
        const int idx = y * width + x;
        if (map.data[idx] != 0)
        {
          continue;
        }
        ++(*free_count);
        if (map.data[idx - 1] == -1 || map.data[idx + 1] == -1 || map.data[idx - width] == -1 ||
            map.data[idx + width] == -1)
        {
          ++(*frontier_count);
        }
      }
    }
  }

  void finishAndSave()
  {
    prompting_ = true;
    if (stop_robot_)
    {
      cmd_pub_.publish(geometry_msgs::Twist());
    }
    publishExplorerCommand("pause");
    ros::Duration(0.2).sleep();
    ROS_INFO("MapSaver: exploration looks complete; stop command sent");

    PromptResult prompt = promptUser();
    if (prompt.action == PromptAction::ContinueMapping)
    {
      ROS_WARN("MapSaver: user chose to continue mapping without saving");
      resetCompletionState();
      publishExplorerCommand("resume");
      prompting_ = false;
      return;
    }

    const std::string map_path = prompt.map_path;
    const std::string output_dir = dirnameOf(map_path);
    if (!makeDirectory(output_dir))
    {
      ROS_ERROR("MapSaver: failed to create directory: %s", output_dir.c_str());
      publishExplorerCommand("resume");
      prompting_ = false;
      return;
    }

    ROS_INFO("MapSaver: saving map to %s.pgm and %s.yaml", map_path.c_str(), map_path.c_str());
    if (!runMapSaver(map_path))
    {
      ROS_ERROR("MapSaver: map_saver failed");
      publishExplorerCommand("resume");
      prompting_ = false;
      return;
    }

    ROS_INFO("MapSaver: map saved: %s.pgm and %s.yaml", map_path.c_str(), map_path.c_str());
    saved_ = true;
  }

  PromptResult promptUser()
  {
    PromptResult result;
    if (!map_path_override_.empty())
    {
      ROS_INFO("MapSaver: using map_path from launch: %s", map_path_override_.c_str());
      result.map_path = normalizeMapPath(map_path_override_);
      return result;
    }

    const std::string default_folder = folder_name_.empty() ? currentTimestamp() : sanitizeFolderName(folder_name_);
    const std::string default_path = default_root_ + "/" + default_folder + "/map";
    if (!prompt_on_complete_)
    {
      result.map_path = default_path;
      return result;
    }

    std::cout << std::endl;
    std::cout << COLOR_CYAN << "========== ros_fyoyi 自主建图状态确认 ==========" << COLOR_RESET << std::endl;
    std::cout << COLOR_YELLOW << "机器人已暂停，等待你的判断。" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN << "直接回车 / y / s：保存地图" << COLOR_RESET << std::endl;
    std::cout << COLOR_YELLOW << "c：暂不保存，继续自动建图" << COLOR_RESET << std::endl;
    std::cout << COLOR_CYAN << "请选择: " << COLOR_RESET << std::flush;

    std::string confirm;
    if (!std::getline(std::cin, confirm))
    {
      ROS_WARN("MapSaver: terminal is not interactive; using default save path");
      result.map_path = default_path;
      return result;
    }
    std::transform(confirm.begin(), confirm.end(), confirm.begin(), [](unsigned char c) { return std::tolower(c); });
    if (confirm == "c" || confirm == "continue" || confirm == "n" || confirm == "no")
    {
      result.action = PromptAction::ContinueMapping;
      return result;
    }
    std::cout << COLOR_GREEN << "请输入保存地址，直接回车使用默认地址 " << default_root_ << ": " << COLOR_RESET << std::flush;
    std::string root;
    std::getline(std::cin, root);
    std::cout << COLOR_GREEN << "请输入地图文件夹名称，直接回车使用当前时间 " << default_folder << ": " << COLOR_RESET << std::flush;
    std::string folder;
    std::getline(std::cin, folder);

    const std::string output_root = root.empty() ? default_root_ : expandHome(root);
    const std::string output_folder = folder.empty() ? default_folder : sanitizeFolderName(folder);
    result.map_path = output_root + "/" + output_folder + "/map";
    return result;
  }

  void publishExplorerCommand(const std::string& command)
  {
    std_msgs::String msg;
    msg.data = command;
    explorer_command_pub_.publish(msg);
  }

  void resetCompletionState()
  {
    clear_since_ = ros::Time();
    no_goal_complete_since_ = ros::Time();
    latest_explorer_state_ = "user_continue";
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber explorer_state_sub_;
  ros::Subscriber command_sub_;
  ros::Publisher explorer_command_pub_;
  ros::Publisher cmd_pub_;
  ros::Timer timer_;

  nav_msgs::OccupancyGrid latest_map_;
  bool have_map_ = false;
  bool saved_ = false;
  bool prompting_ = false;
  bool prompt_on_complete_ = true;
  bool stop_robot_ = true;
  int frontier_threshold_ = 30;
  int min_free_cells_ = 200;
  double stable_duration_ = 12.0;
  double no_goal_stable_duration_ = 6.0;
  double min_runtime_ = 45.0;
  ros::Time started_at_ = ros::Time::now();
  ros::Time clear_since_;
  ros::Time no_goal_complete_since_;
  std::string latest_explorer_state_ = "unknown";
  std::string default_root_;
  std::string folder_name_;
  std::string map_path_override_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "auto_map_saver");
  AutoMapSaver saver;
  ros::spin();
  return 0;
}
