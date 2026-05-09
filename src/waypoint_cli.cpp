#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <iostream>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace
{

void printUsage()
{
  std::cout << R"(ros_fyoyi 航点工具

用法:
  rosrun ros_fyoyi waypoint_cli help
      显示本帮助信息。

  rosrun ros_fyoyi waypoint_cli list
      查看当前保存的所有航点，包含 x、y、yaw 角度和坐标系。

  rosrun ros_fyoyi waypoint_cli save
      手动保存航点到 yaml 文件。默认已自动保存，这条命令用于确认写入。

  rosrun ros_fyoyi waypoint_cli reload
      从 yaml 文件重新加载航点。手动修改 waypoints.yaml 后使用。

  rosrun ros_fyoyi waypoint_cli clear
      清空当前航点列表，并按当前配置自动保存。这个操作会删除已加载航点。

  rosrun ros_fyoyi waypoint_cli nav <waypoint_name>
      导航到指定航点。发送前会检查航点是否存在。

  rosrun ros_fyoyi waypoint_cli patrol <waypoint_1> <waypoint_2> ...
      队列导航，按顺序依次前往多个航点。

  rosrun ros_fyoyi waypoint_cli patrol --loop <waypoint_1> <waypoint_2> ...
      循环巡逻，按顺序到达最后一个航点后，再从第一个航点重新开始。

  rosrun ros_fyoyi waypoint_cli cancel
      取消当前导航、队列导航或循环巡逻。

  rosrun ros_fyoyi waypoint_cli pause
      暂停当前巡逻任务。暂停时会取消当前 move_base 目标。

  rosrun ros_fyoyi waypoint_cli resume
      继续暂停中的巡逻任务。继续后会从当前航点重新发送导航目标。

  rosrun ros_fyoyi waypoint_cli status
      查看当前状态，例如空闲、单点导航、队列导航、循环巡逻、暂停。

  rosrun ros_fyoyi waypoint_cli watch [刷新秒数]
      实时查看状态和导航结果日志。默认每 1 秒刷新一次，按 Ctrl+C 退出。

  rosrun ros_fyoyi waypoint_cli delete <waypoint_name>
      删除指定航点。发送前会检查航点是否存在。

  rosrun ros_fyoyi waypoint_cli rename <old_name> <new_name>
      重命名航点。旧名字必须存在，新名字不能重复。

  rosrun ros_fyoyi waypoint_cli name <next_waypoint_name>
      设置下一个通过 RViz 添加的航点名字。只对下一次添加生效。

示例:
  rosrun ros_fyoyi waypoint_cli list
  rosrun ros_fyoyi waypoint_cli name room
  rosrun ros_fyoyi waypoint_cli nav room
  rosrun ros_fyoyi waypoint_cli patrol room ketin fangjian
  rosrun ros_fyoyi waypoint_cli patrol --loop room ketin fangjian
  rosrun ros_fyoyi waypoint_cli watch
  rosrun ros_fyoyi waypoint_cli cancel
  rosrun ros_fyoyi waypoint_cli rename wp_1 door
  rosrun ros_fyoyi waypoint_cli delete door
  rosrun ros_fyoyi waypoint_cli save
)";
}

bool callTriggerRaw(const std::string& service_name, std_srvs::Trigger::Response& response)
{
  if (!ros::service::waitForService(service_name, ros::Duration(5.0)))
  {
    std::cerr << "错误：服务不可用，请先启动 waypoint_server：" << service_name << std::endl;
    return false;
  }

  std_srvs::Trigger srv;
  if (!ros::service::call(service_name, srv))
  {
    std::cerr << "错误：调用服务失败：" << service_name << std::endl;
    return false;
  }

  response = srv.response;
  return true;
}

bool callTrigger(const std::string& service_name)
{
  std_srvs::Trigger::Response response;
  if (!callTriggerRaw(service_name, response))
  {
    return false;
  }

  std::cout << response.message << std::endl;
  return response.success;
}

bool isValidName(const std::string& name)
{
  static const std::regex pattern("^[A-Za-z0-9_-]+$");
  return std::regex_match(name, pattern);
}

std::set<std::string> parseWaypointNames(const std::string& list_text)
{
  std::set<std::string> names;
  std::istringstream input(list_text);
  std::string line;
  while (std::getline(input, line))
  {
    if (line.empty() || line == "no waypoint" || line == "当前没有航点")
    {
      continue;
    }

    const std::size_t split = line.find(':');
    if (split != std::string::npos && split > 0)
    {
      names.insert(line.substr(0, split));
    }
  }
  return names;
}

bool getWaypointNames(std::set<std::string>& names)
{
  std_srvs::Trigger::Response response;
  if (!callTriggerRaw("/fyoyi/list_waypoints", response))
  {
    return false;
  }
  if (!response.success)
  {
    std::cerr << "错误：读取航点列表失败：" << response.message << std::endl;
    return false;
  }
  names = parseWaypointNames(response.message);
  return true;
}

void printAvailableWaypoints(const std::set<std::string>& names)
{
  if (names.empty())
  {
    std::cerr << "当前没有可用航点。" << std::endl;
    return;
  }

  std::cerr << "可用航点：";
  bool first = true;
  for (const auto& name : names)
  {
    if (!first)
    {
      std::cerr << ", ";
    }
    std::cerr << name;
    first = false;
  }
  std::cerr << std::endl;
}

bool validateNameSyntax(const std::string& name)
{
  if (isValidName(name))
  {
    return true;
  }

  std::cerr << "错误：航点名只能包含英文、数字、下划线和横线，不能包含空格或中文：" << name << std::endl;
  return false;
}

bool requireWaypointExists(const std::set<std::string>& names, const std::string& name, const std::string& action)
{
  if (names.count(name) > 0)
  {
    return true;
  }

  std::cerr << "错误：航点不存在，无法" << action << "：" << name << std::endl;
  printAvailableWaypoints(names);
  return false;
}

bool requireWaypointMissing(const std::set<std::string>& names, const std::string& name, const std::string& action)
{
  if (names.count(name) == 0)
  {
    return true;
  }

  std::cerr << "错误：航点名已存在，无法" << action << "：" << name << std::endl;
  printAvailableWaypoints(names);
  return false;
}

std::string joinWords(const std::vector<std::string>& words, std::size_t begin_index)
{
  std::ostringstream output;
  for (std::size_t i = begin_index; i < words.size(); ++i)
  {
    if (i > begin_index)
    {
      output << " ";
    }
    output << words[i];
  }
  return output.str();
}

bool publishOnce(ros::NodeHandle& nh, const std::string& topic, const std::string& text)
{
  ros::Publisher pub = nh.advertise<std_msgs::String>(topic, 1);
  ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(3.0);
  ros::WallRate rate(20);
  while (ros::ok() && pub.getNumSubscribers() == 0 && ros::WallTime::now() < deadline)
  {
    ros::spinOnce();
    rate.sleep();
  }

  if (pub.getNumSubscribers() == 0)
  {
    std::cerr << "错误：没有节点订阅话题，请确认 waypoint_server 已启动：" << topic << std::endl;
    return false;
  }

  std_msgs::String msg;
  msg.data = text;
  pub.publish(msg);
  ros::WallDuration(0.2).sleep();
  return true;
}

void navResultCallback(const std_msgs::String::ConstPtr& msg)
{
  std::cout << "\n[导航结果] " << msg->data << std::endl;
}

bool parsePositiveDouble(const std::string& text, double& value)
{
  std::istringstream input(text);
  input >> value;
  return !input.fail() && input.eof() && value > 0.0;
}

int watchStatus(ros::NodeHandle& nh, double period_seconds)
{
  if (!ros::service::waitForService("/fyoyi/status", ros::Duration(5.0)))
  {
    std::cerr << "错误：服务不可用，请先启动 waypoint_server：/fyoyi/status" << std::endl;
    return 1;
  }

  ros::ServiceClient status_client = nh.serviceClient<std_srvs::Trigger>("/fyoyi/status");
  ros::Subscriber result_sub = nh.subscribe("/fyoyi/navi_result", 20, navResultCallback);
  ros::WallRate rate(1.0 / period_seconds);

  std::cout << "开始实时查看小车状态，按 Ctrl+C 退出。" << std::endl;
  while (ros::ok())
  {
    std_srvs::Trigger srv;
    if (status_client.call(srv) && srv.response.success)
    {
      std::cout << "\n========== 小车状态 ==========\n" << srv.response.message
                << "\n==============================" << std::endl;
    }
    else
    {
      std::cerr << "错误：读取状态失败。" << std::endl;
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i)
  {
    args.push_back(argv[i]);
  }

  if (args.empty() || args[0] == "help" || args[0] == "-h" || args[0] == "--help")
  {
    printUsage();
    return 0;
  }

  ros::init(argc, argv, "waypoint_cli", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  const std::string& command = args[0];
  if (command == "list" && args.size() == 1)
  {
    return callTrigger("/fyoyi/list_waypoints") ? 0 : 1;
  }
  if (command == "save" && args.size() == 1)
  {
    return callTrigger("/fyoyi/save_waypoints") ? 0 : 1;
  }
  if (command == "reload" && args.size() == 1)
  {
    return callTrigger("/fyoyi/reload_waypoints") ? 0 : 1;
  }
  if (command == "clear" && args.size() == 1)
  {
    return callTrigger("/fyoyi/clear_waypoints") ? 0 : 1;
  }
  if (command == "status" && args.size() == 1)
  {
    return callTrigger("/fyoyi/status") ? 0 : 1;
  }
  if (command == "watch" && args.size() <= 2)
  {
    double period_seconds = 1.0;
    if (args.size() == 2 && !parsePositiveDouble(args[1], period_seconds))
    {
      std::cerr << "错误：刷新秒数必须是大于 0 的数字。" << std::endl;
      return 2;
    }
    return watchStatus(nh, period_seconds);
  }
  if (command == "nav" && args.size() == 2)
  {
    std::set<std::string> names;
    if (!validateNameSyntax(args[1]) || !getWaypointNames(names) ||
        !requireWaypointExists(names, args[1], "导航"))
    {
      return 1;
    }
    if (!publishOnce(nh, "/fyoyi/navi_waypoint", args[1]))
    {
      return 1;
    }
    std::cout << "已发送导航航点：" << args[1] << std::endl;
    return 0;
  }
  if (command == "patrol" && args.size() >= 2)
  {
    bool loop = false;
    std::size_t first_name_index = 1;
    if (args[1] == "--loop")
    {
      loop = true;
      first_name_index = 2;
    }
    if (first_name_index >= args.size())
    {
      std::cerr << "错误：巡逻命令至少需要一个航点。" << std::endl;
      return 2;
    }

    std::set<std::string> names;
    if (!getWaypointNames(names))
    {
      return 1;
    }
    for (std::size_t i = first_name_index; i < args.size(); ++i)
    {
      if (!validateNameSyntax(args[i]) || !requireWaypointExists(names, args[i], "加入巡逻队列"))
      {
        return 1;
      }
    }

    std::string command_text = joinWords(args, first_name_index);
    if (loop)
    {
      command_text = "--loop " + command_text;
    }
    if (!publishOnce(nh, "/fyoyi/patrol_waypoints", command_text))
    {
      return 1;
    }
    std::cout << (loop ? "已发送循环巡逻队列：" : "已发送导航队列：") << joinWords(args, first_name_index)
              << std::endl;
    return 0;
  }
  if (command == "cancel" && args.size() == 1)
  {
    if (!publishOnce(nh, "/fyoyi/cancel_navigation", "cancel"))
    {
      return 1;
    }
    std::cout << "已发送取消导航命令。" << std::endl;
    return 0;
  }
  if (command == "pause" && args.size() == 1)
  {
    if (!publishOnce(nh, "/fyoyi/pause_patrol", "pause"))
    {
      return 1;
    }
    std::cout << "已发送暂停巡逻命令。" << std::endl;
    return 0;
  }
  if (command == "resume" && args.size() == 1)
  {
    if (!publishOnce(nh, "/fyoyi/resume_patrol", "resume"))
    {
      return 1;
    }
    std::cout << "已发送继续巡逻命令。" << std::endl;
    return 0;
  }
  if (command == "delete" && args.size() == 2)
  {
    std::set<std::string> names;
    if (!validateNameSyntax(args[1]) || !getWaypointNames(names) ||
        !requireWaypointExists(names, args[1], "删除"))
    {
      return 1;
    }
    if (!publishOnce(nh, "/fyoyi/delete_waypoint", args[1]))
    {
      return 1;
    }
    std::cout << "已发送删除航点：" << args[1] << std::endl;
    return 0;
  }
  if (command == "rename" && args.size() == 3)
  {
    std::set<std::string> names;
    if (!validateNameSyntax(args[1]) || !validateNameSyntax(args[2]) || !getWaypointNames(names) ||
        !requireWaypointExists(names, args[1], "重命名") ||
        !requireWaypointMissing(names, args[2], "重命名"))
    {
      return 1;
    }
    if (!publishOnce(nh, "/fyoyi/rename_waypoint", args[1] + " " + args[2]))
    {
      return 1;
    }
    std::cout << "已发送重命名航点：" << args[1] << " -> " << args[2] << std::endl;
    return 0;
  }
  if ((command == "name" || command == "next-name") && args.size() == 2)
  {
    std::set<std::string> names;
    if (!validateNameSyntax(args[1]) || !getWaypointNames(names) ||
        !requireWaypointMissing(names, args[1], "设置为下一个航点名"))
    {
      return 1;
    }
    if (!publishOnce(nh, "/fyoyi/next_waypoint_name", args[1]))
    {
      return 1;
    }
    std::cout << "下一个通过 RViz 添加的航点将命名为：" << args[1] << std::endl;
    return 0;
  }

  std::cerr << "错误：命令格式不正确。" << std::endl;
  printUsage();
  return 2;
}
