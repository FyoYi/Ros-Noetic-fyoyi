/*
 * 航点命令行工具。
 *
 * 这个节点是 waypoint_server 的终端客户端，用于从命令行发送航点管理和
 * 导航命令，例如 list、save、reload、nav、patrol、cancel、pause、
 * resume、delete、rename、status 等。它本身不保存航点，也不直接控制
 * move_base，而是通过 ros_fyoyi 的 topic/service 接口调用 waypoint_server。
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
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

  std::cout << R"(

███████╗██╗   ██╗ ██████╗ ██╗   ██╗██╗
██╔════╝╚██╗ ██╔╝██╔═══██╗╚██╗ ██╔╝██║
█████╗   ╚████╔╝ ██║   ██║ ╚████╔╝ ██║
██╔══╝    ╚██╔╝  ██║   ██║  ╚██╔╝  ██║
██║        ██║   ╚██████╔╝   ██║   ██║
╚═╝        ╚═╝    ╚═════╝    ╚═╝   ╚═╝

)";

  // 标题
  std::cout << "\033[1;96m"
            << "ros_fyoyi 航点工具\n"
            << "\033[0m\n";

  // 用法标题
  std::cout << "\033[1;93m"
            << "用法:\n"
            << "\033[0m";

  std::cout << "\033[97m  rosrun ros_fyoyi waypoint_cli := ~\033[0m\n\n";

  // 指令
  std::cout << "\033[1;92m";

  std::cout << "  ~ help";
  std::cout << "\033[97m    显示帮助信息\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ watch";
  std::cout << "\033[97m   实时监控小车状态\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ status";
  std::cout << "\033[97m  查看当前状态\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ list";
  std::cout << "\033[97m    显示所有航点\033[0m\n\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ nav <waypoint>";
  std::cout << "\033[97m    导航到指定航点\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ patrol (-l (-n <loop_num>)) <waypoint_1> ...";
  std::cout << "\033[97m  巡逻多个航点\033[0m\n\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ pause";
  std::cout << "\033[97m   暂停当前任务\033[0m\n";
  
  std::cout << "\033[1;92m";
  std::cout << "  ~ resume";
  std::cout << "\033[97m  恢复当前任务\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ stop";
  std::cout << "\033[97m    停止当前任务\033[0m\n\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ set (-n <next_waypoint_name>)";
  std::cout << "\033[97m    设置当前位置为航点\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ rename <old_name> <new_name>";
  std::cout << "\033[97m     重命名航点\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ delete <waypoint_name>";
  std::cout << "\033[97m           删除指定航点\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ clear";
  std::cout << "\033[97m   清空所有航点\033[0m\n\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ save";
  std::cout << "\033[97m    保存航点到文件\033[0m\n";

  std::cout << "\033[1;92m";
  std::cout << "  ~ reload";
  std::cout << "\033[97m  重新加载航点文件\033[0m\n";

  std::cout << "\033[0m" << std::endl;
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

void display_banner2()
{
    std::cout << "███████╗██╗   ██╗ ██████╗ ██╗   ██╗██╗" << std::endl;
    std::cout << "██╔════╝╚██╗ ██╔╝██╔═══██╗╚██╗ ██╔╝██║" << std::endl;
    std::cout << "█████╗   ╚████╔╝ ██║   ██║ ╚████╔╝ ██║" << std::endl;
    std::cout << "██╔══╝    ╚██╔╝  ██║   ██║  ╚██╔╝  ██║" << std::endl;
    std::cout << "██║        ██║   ╚██████╔╝   ██║   ██║" << std::endl;
    std::cout << "╚═╝        ╚═╝    ╚═════╝    ╚═╝   ╚═╝" << std::endl;
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

bool parsePositiveInt(const std::string& text, int& value)
{
  std::istringstream input(text);
  input >> value;
  return !input.fail() && input.eof() && value > 0;
}

std::vector<std::string> splitByDelimiter(const std::string& text, const std::string& delimiter)
{
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (start <= text.size())
  {
    const std::size_t found = text.find(delimiter, start);
    if (found == std::string::npos)
    {
      parts.push_back(text.substr(start));
      break;
    }
    parts.push_back(text.substr(start, found - start));
    start = found + delimiter.size();
  }
  return parts;
}

std::string trimText(const std::string& text)
{
  const std::size_t begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos)
  {
    return "";
  }
  const std::size_t end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

// std::string compactPatrolQueueText(const std::string& queue_text)
// {
//   std::vector<std::string> items = splitByDelimiter(queue_text, "->");
//   std::size_t current_index = 0;
//   bool found_current = false;

//   for (std::size_t i = 0; i < items.size(); ++i)
//   {
//     std::string item = trimText(items[i]);
//     if (item.size() >= 2 && item.front() == '[' && item.back() == ']')
//     {
//       item = item.substr(1, item.size() - 2);
//       current_index = i;
//       found_current = true;
//     }
//     items[i] = trimText(item);
//   }

//   if (items.empty() || !found_current)
//   {
//     return queue_text;
//   }

//   const std::size_t first_visible = current_index > 2 ? current_index - 2 : 0;
//   const std::size_t last_visible = std::min(current_index + 2, items.size() - 1);

//   std::ostringstream output;
//   bool has_output = false;
//   const int hidden_before = static_cast<int>(first_visible);
//   if (hidden_before > 0)
//   {
//     output << "[" << hidden_before << "waypoint]";
//     has_output = true;
//   }

//   for (std::size_t i = first_visible; i <= last_visible; ++i)
//   {
//     if (i == current_index)
//     {
//       output << (has_output ? " " : "") << "->[" << items[i] << "]";
//     }
//     else
//     {
//       if (has_output)
//       {
//         output << " -> ";
//       }
//       output << items[i];
//     }
//     has_output = true;
//   }

//   return output.str();
// }

std::string compactPatrolQueueText(const std::string& queue_text)
{
    std::vector<std::string> items =
        splitByDelimiter(queue_text, "->");

    std::size_t current_index = 0;
    bool found_current = false;

    // =========================
    // 找当前目标点 [xxx]
    // =========================
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        std::string item = trimText(items[i]);

        if (item.size() >= 2 &&
            item.front() == '[' &&
            item.back() == ']')
        {
            item = item.substr(1, item.size() - 2);

            current_index = i;
            found_current = true;
        }

        items[i] = trimText(item);
    }

    if (items.empty() || !found_current)
    {
        return queue_text;
    }

    // =========================
    // 红色箭头所在边
    // =========================
    int arrow_left = static_cast<int>(current_index) - 1;

    // 左边最多显示2个
    int visible_left =
        std::max(0, arrow_left - 1);

    // 右边最多显示2个
    int visible_right =
        std::min(
            static_cast<int>(items.size()) - 1,
            static_cast<int>(current_index) + 1);

    std::ostringstream output;

    // =========================
    // 左侧压缩
    // =========================
    if (visible_left > 0)
    {
        output << "["
               << items.front()
               << "->{"
               << (visible_left - 1)
               << "}]->";
    }

    // =========================
    // 开始输出
    // =========================
    for (int i = visible_left;
         i <= visible_right;
         ++i)
    {
        // 起始情况
        if (i == 0 && current_index == 0)
        {
            output << "\033[1;92m->\033[0m"
                   << items[i];
        }
        else
        {
            output << items[i];
        }

        // 后面还有
        if (i < visible_right)
        {
            // 当前运动边
            if (i == arrow_left)
            {
                output << "\033[1;92m->\033[0m";
            }
            else
            {
                output << "->";
            }
        }
    }

    // =========================
    // 右侧压缩
    // =========================
    int hidden_right =
        static_cast<int>(items.size()) - visible_right - 1;

    if (hidden_right > 0)
    {
        output << "->[{"
               << (hidden_right - 1)
               << "}->"
               << items.back()
               << "]";
    }

    return output.str();
}

std::string formatStatusForWatch(const std::string& status_text)
{
  std::istringstream input(status_text);
  std::ostringstream output;
  std::string line;
  bool first_line = true;
  const std::string prefix = "巡逻队列：";

  while (std::getline(input, line))
  {
    if (!first_line)
    {
      output << "\n";
    }
    first_line = false;

    if (line.find(prefix) == 0)
    {
      output << prefix << compactPatrolQueueText(line.substr(prefix.size()));
    }
    else
    {
      output << line;
    }
  }

  return output.str();
}

int watchStatus(ros::NodeHandle& nh)
{
  if (!ros::service::waitForService("/fyoyi/status", ros::Duration(5.0)))
  {
    std::cerr << "错误：服务不可用，请先启动 waypoint_server：/fyoyi/status" << std::endl;
    return 1;
  }

  ros::ServiceClient status_client = nh.serviceClient<std_srvs::Trigger>("/fyoyi/status");
  ros::Subscriber result_sub = nh.subscribe("/fyoyi/navi_result", 20, navResultCallback);
  ros::WallRate rate(1.0);

  std::string last_status;
  int no_change_count = 0;

  std::cout << "开始实时查看小车状态，按 Ctrl+C 退出。" << std::endl;
  while (ros::ok())
  {
    std_srvs::Trigger srv;
    if (status_client.call(srv) && srv.response.success)
    {
      std::string current_status = formatStatusForWatch(srv.response.message);
      bool changed = (current_status != last_status);
      bool timeout_refresh = (no_change_count >= 10);

      if (changed || timeout_refresh)
      {
        std::cout << "\033[2J\033[1;1H" << std::endl;
        display_banner2();
        std::cout << "\n=============== 小车状态 ===============\n" 
                  << current_status
                  << "\n========================================" << std::endl;

        last_status = current_status;

        no_change_count = 0;
      }
      else
      {
        no_change_count++;
      }
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
  if (command == "list" && args.size() == 1){
    return callTrigger("/fyoyi/list_waypoints") ? 0 : 1;
  }

  if (command == "save" && args.size() == 1){
    return callTrigger("/fyoyi/save_waypoints") ? 0 : 1;
  }

  if (command == "reload" && args.size() == 1){
    return callTrigger("/fyoyi/reload_waypoints") ? 0 : 1;
  }

  if (command == "clear" && args.size() == 1){
    return callTrigger("/fyoyi/clear_waypoints") ? 0 : 1;
  }

  if (command == "status" && args.size() == 1){
    return callTrigger("/fyoyi/status") ? 0 : 1;
  }

  if (command == "watch" && args.size() == 1){
    return watchStatus(nh);
  }
  
  if (command == "nav" && args.size() == 2){

    std::set<std::string> names;

    if (!validateNameSyntax(args[1]) || !getWaypointNames(names) || !requireWaypointExists(names, args[1], "导航"))
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
/* ***************************************************************************************** */
  if (command == "patrol" && args.size() >= 2)
  {
    bool loop = false;
    int loop_num = 0;
    std::size_t first_name_index = 1;
    if (args[1] == "-l")
    {
      loop = true;
      first_name_index = 2;
      if (args.size() >= 4 && args[2] == "-n")
      {
        if (!parsePositiveInt(args[3], loop_num))
        {
          std::cerr << "错误：-n 后面的循环次数必须是大于 0 的整数。" << std::endl;
          return 2;
        }
        std::cout << loop_num << std::endl;

        /*------------------------------------------------*/
        first_name_index = 4;
        /*------------------------------------------------*/
      }
      else if (args.size() >= 3 && args[2] == "-n")
      {
        std::cerr << "错误：-n 后面必须跟循环次数。" << std::endl;
        return 2;
      }
    }
    else if (args[1] == "-n")
    {
      std::cerr << "错误：-n 必须配合 -l 使用，格式：patrol -l -n <loop_num> <waypoint_1> ..." << std::endl;
      return 2;
    }
    else if (!args[1].empty() && args[1][0] == '-')
    {
      std::cerr << "错误：patrol 只支持格式：patrol (-l (-n <loop_num>)) <waypoint_1> ..." << std::endl;
      return 2;
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
      if (!args[i].empty() && args[i][0] == '-')
      {
        std::cerr << "错误：patrol 只支持格式：patrol (-l (-n <loop_num>)) <waypoint_1> ..." << std::endl;
        return 2;
      }
      if (!validateNameSyntax(args[i]) || !requireWaypointExists(names, args[i], "加入巡逻队列"))
      {
        return 1;
      }
    }

    std::string command_text = joinWords(args, first_name_index);
    if (loop){
      command_text = "--loop " + command_text;
      if (loop_num > 0)
      {
        command_text = "--loop --loop-count= " + std::to_string(loop_num) + " " + joinWords(args, first_name_index);
      }
    }
    if (!publishOnce(nh, "/fyoyi/patrol_waypoints", command_text))
    {
      return 1;
    }
    if (loop && loop_num > 0)
    {
      std::cout << "已发送循环巡逻队列：" << joinWords(args, first_name_index) << "，循环次数：" << loop_num << std::endl;
    }
    else
    {
      std::cout << (loop ? "已发送循环巡逻队列：" : "已发送导航队列：") << joinWords(args, first_name_index)
                << std::endl;
    }
    return 0;
  }

/* ***************************************************************************************** */

  if (command == "stop" && args.size() == 1)
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

  if (command == "delete" && args.size() >= 2)
  {
    std::set<std::string> names;
    for(std::size_t i = 0; i < args.size()-1; i++){
      if (!validateNameSyntax(args[i+1]) || !getWaypointNames(names) ||
            !requireWaypointExists(names, args[i+1], "删除"))
      {
        return 1;
      }
      if (!publishOnce(nh, "/fyoyi/delete_waypoint", args[i+1]))
      {
        return 1;
      }
    std::cout << "已发送删除航点：" << args[i+1] << std::endl;
  }
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
  
  if (command == "set")
  {
    if (args.size() == 1)
    {
      if (!publishOnce(nh, "/fyoyi/next_waypoint_name", ""))
      {
        return 1;
      }
      std::cout << "下一个通过 RViz 添加的航点将使用默认名字。" << std::endl;
      return 0;
    }

    if (args.size() == 3 && args[1] == "-n")
    {
      std::set<std::string> names;
      if (!validateNameSyntax(args[2]) || !getWaypointNames(names) ||
          !requireWaypointMissing(names, args[2], "设置为下一个航点名"))
      {
        return 1;
      }
      if (!publishOnce(nh, "/fyoyi/next_waypoint_name", args[2]))
      {
        return 1;
      }
      std::cout << "下一个通过 RViz 添加的航点将命名为：" << args[2] << std::endl;
      return 0;
    }

    std::cerr << "错误：set 命令格式应为：set 或 set -n <航点名>。" << std::endl;
    return 2;
  }

  std::cerr << "错误：命令格式不正确。" << std::endl;
  printUsage();
  return 2;
}
