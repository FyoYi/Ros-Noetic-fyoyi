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
  rosrun ros_fyoyi waypoint_cli list
  rosrun ros_fyoyi waypoint_cli save
  rosrun ros_fyoyi waypoint_cli reload
  rosrun ros_fyoyi waypoint_cli clear
  rosrun ros_fyoyi waypoint_cli nav <waypoint_name>
  rosrun ros_fyoyi waypoint_cli delete <waypoint_name>
  rosrun ros_fyoyi waypoint_cli rename <old_name> <new_name>
  rosrun ros_fyoyi waypoint_cli name <next_waypoint_name>

示例:
  rosrun ros_fyoyi waypoint_cli list
  rosrun ros_fyoyi waypoint_cli name room
  rosrun ros_fyoyi waypoint_cli nav room
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

  ros::init(argc, argv, "waypoint_cli");
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
