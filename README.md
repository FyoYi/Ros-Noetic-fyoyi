# ros_fyoyi 导航包使用手册

`ros_fyoyi` 是一个 ROS1 导航包，定位是“只负责导航”。它会启动：

```text
map_server          加载已有地图，发布 /map
amcl                根据地图、雷达和里程计做定位
move_base           做路径规划、局部避障，并发布 /cmd_vel
rviz                可视化地图、机器人、路径、代价地图
waypoint_server 本包自带航点管理和航点导航
```

它不负责启动：

```text
机器人模型
Gazebo 仿真环境
雷达驱动
底盘驱动
里程计节点
robot_state_publisher
joint_state_publisher
```

也就是说，这个包默认假设：机器人或者仿真系统已经由其他 bringup 包启动好了。

## 整体工作流程

典型运行流程是：

```text
1. 外部 bringup 启动机器人或仿真
2. 机器人发布 /scan、/odom、/tf
3. ros_fyoyi 启动 map_server 加载地图
4. ros_fyoyi 启动 amcl 进行定位
5. ros_fyoyi 启动 move_base 进行导航
6. RViz 中使用 2D Pose Estimate 设置初始位姿
7. RViz 中使用 2D Nav Goal 发送导航目标
8. move_base 发布 /cmd_vel 控制机器人移动
```

核心数据链路：

```text
/scan + /odom + /tf + /map
        ↓
      amcl
        ↓
   map -> odom
        ↓
    move_base
        ↓
     /cmd_vel
```

## 运行前准备

启动本包前，外部机器人系统必须已经提供下面这些接口。

### 必须有的话题

```text
/scan    sensor_msgs/LaserScan    激光雷达数据
/odom    nav_msgs/Odometry         里程计数据
/tf      tf/tf2                    坐标变换
```

底盘控制器还必须订阅：

```text
/cmd_vel geometry_msgs/Twist       导航速度指令
```

### 必须有的 TF

默认坐标树假设是：

```text
map -> odom -> base_footprint -> laser
```

其中：

```text
map -> odom
```

由 `amcl` 发布。

```text
odom -> base_footprint
```

由机器人底盘里程计或仿真插件发布。

```text
base_footprint -> laser
```

由机器人模型、`robot_state_publisher` 或静态 TF 发布。

如果你的机器人使用 `base_link` 而不是 `base_footprint`，运行时需要传入：

```bash
base_frame:=base_link
```

### 检查命令

启动外部机器人 bringup 后，先检查雷达：

```bash
rostopic hz /scan
```

检查里程计：

```bash
rostopic echo /odom
```

检查 TF：

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_footprint laser
```

如果你的底盘坐标叫 `base_link`，就检查：

```bash
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link laser
```

检查底盘是否能接收速度：

```bash
rostopic info /cmd_vel
```

如果这些都不正常，先不要启动导航，应该先修机器人 bringup。

## 安装依赖

本包依赖 ROS 导航相关组件。常见依赖包括：

```text
move_base
map_server
amcl
rviz
teb_local_planner
global_planner
clear_costmap_recovery
rotate_recovery
```

如果某个包缺失，`roslaunch` 会提示类似：

```text
cannot launch node of type [...]
```

系统包可用下面方式安装一部分：

```bash
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-teb-local-planner
sudo apt install ros-noetic-global-planner
```

`waterplus_map_tools` 现在不是本包必需依赖。只有你主动设置 `use_waterplus:=true` 兼容老教程时，才需要额外安装它。

## 编译

在工作区根目录编译：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

确认 ROS 能找到本包：

```bash
rospack find ros_fyoyi
```

应该输出类似：

```text
/home/fyoyi/catkin_ws/src/ros_fyoyi
```

## 默认启动

如果机器人 bringup 已经启动，并且话题和坐标系都符合默认约定，可以直接运行：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch ros_fyoyi waypoint_nav.launch
```

默认会启动这些节点：

```text
/map_server
/amcl
/move_base
/rviz
/waypoint_server
```

可以用下面命令只检查会启动哪些节点，不真正启动：

```bash
roslaunch ros_fyoyi waypoint_nav.launch --nodes
```

可以用下面命令查看最终加载的参数：

```bash
roslaunch ros_fyoyi waypoint_nav.launch --dump-params
```

## RViz 中如何导航

启动后，RViz 中通常需要两步：

1. 点击 `2D Pose Estimate`
2. 在地图上点机器人当前大概位置，并拖动箭头设置朝向
3. 点击 `2D Nav Goal`
4. 在地图上点目标位置，并拖动箭头设置目标朝向

注意：

```text
2D Pose Estimate 是设置初始定位，不会让机器人移动。
2D Nav Goal 才是发送导航目标。
```

如果机器人不动，检查：

```bash
rostopic echo /move_base/status
rostopic echo /cmd_vel
```

如果 `/cmd_vel` 有速度但机器人不动，问题通常在底盘驱动或仿真底盘插件。

如果 `/cmd_vel` 没速度，问题通常在定位、地图、TF、costmap 或目标点不可达。

## 自带航点功能

本包现在带了一个轻量版航点节点：

```text
waypoint_server
```

它提供类似 `waterplus_map_tools` 的基础功能：

```text
1. 启动时从 yaml 文件加载航点
2. 在 RViz 中标记航点
3. 在 RViz 中显示航点图标和名字
4. 保存航点到 yaml 文件
5. 按航点名字调用 move_base 导航
6. 发布导航结果
7. 在 RViz 中右键删除航点
8. 自定义下一个航点名字，或者重命名已有航点
```

默认航点文件是：

```text
$(find ros_fyoyi)/config/waypoints.yaml
```

也就是源码里的：

```text
src/ros_fyoyi/config/waypoints.yaml
```

### 标记航点

启动：

```bash
roslaunch ros_fyoyi waypoint_nav.launch
```

RViz 工具栏里会有两个 `2D Nav Goal` 类型的工具：

```text
/move_base_simple/goal   直接导航
/fyoyi/add_waypoint      添加航点
```

添加航点时，选择 topic 为：

```text
/fyoyi/add_waypoint
```

然后在地图上点击并拖动箭头设置朝向。节点会自动生成名字：

```text
wp_1
wp_2
wp_3
```

如果你希望下一个航点使用自定义名字，例如 `aa`，先运行：

```bash
rosrun ros_fyoyi waypoint_cli name aa
```

然后再去 RViz 里用 `/fyoyi/add_waypoint` 点航点。这个新航点就会叫：

```text
aa
```

航点会显示在 RViz 的：

```text
/fyoyi/waypoints_marker
```

现在航点显示为蓝色箭头和白色名字。

注意：

```text
name 只是设置“下一个航点的名字”。
只有你在 RViz 里真正点下 /fyoyi/add_waypoint 之后，航点才会加入列表。
航点名只支持英文、数字、下划线和横线，例如 aa、wp_1、room-a。
```

### 保存航点

默认情况下，添加、删除、重命名航点后会自动保存到文件。也就是说，你在 RViz 里点完航点后，下次重新启动也会加载出来。

如果你想手动再保存一次，可以调用：

```bash
rosservice call /fyoyi/save_waypoints "{}"
```

保存成功后，航点会写入：

```text
config/waypoints.yaml
```

### 查看航点

推荐使用本包自带命令：

```bash
rosrun ros_fyoyi waypoint_cli list
```

它会按行显示航点，例如：

```text
wp_1: x=-2.953, y=2.268, yaw=0.0 deg, frame=map
wp_2: x=-1.570, y=2.560, yaw=-75.8 deg, frame=map
```

也可以使用服务：

```bash
rosservice call /fyoyi/list_waypoints "{}"
```

但是 `rosservice call` 打印字符串时，会把换行显示成 `\n`：

```text
message: "wp_1: ...\nwp_2: ..."
```

这不是节点没有换行，而是 `rosservice` 对字符串的显示方式。想看真正换行，用：

```bash
rosrun ros_fyoyi waypoint_cli list
```

也可以看话题：

```bash
rostopic echo /fyoyi/waypoint_list
```

### 按名字导航

例如导航到 `wp_1`：

```bash
rostopic pub /fyoyi/navi_waypoint std_msgs/String "data: 'wp_1'" -1
```

也可以使用更短的命令：

```bash
rosrun ros_fyoyi waypoint_cli nav wp_1
```

查看导航结果：

```bash
rostopic echo /fyoyi/navi_result
```

成功时会看到类似：

```text
完成:wp_1
```

失败时会看到类似：

```text
失败:wp_1:state=ABORTED
```

如果用命令行导航到不存在的航点，`waypoint_cli` 会先报错，不会继续发布导航指令：

```bash
rosrun ros_fyoyi waypoint_cli nav not_exist
```

输出类似：

```text
错误：航点不存在，无法导航：not_exist
可用航点：wp_1, room
```

### 队列导航和循环巡逻

队列导航就是一次给多个航点，小车会按顺序导航。

例如依次去 `room`、`ketin`、`fangjian`：

```bash
rosrun ros_fyoyi waypoint_cli patrol room ketin fangjian
```

如果想一直循环巡逻，在航点前面加 `--loop`：

```bash
rosrun ros_fyoyi waypoint_cli patrol --loop room ketin fangjian
```

也可以直接发布话题：

```bash
rostopic pub /fyoyi/patrol_waypoints std_msgs/String "data: 'room ketin fangjian'" -1
rostopic pub /fyoyi/patrol_waypoints std_msgs/String "data: '--loop room ketin fangjian'" -1
```

注意：

```text
队列里的每个航点都必须已经存在。
如果某个航点不存在，waypoint_cli 会先报错，不会发送巡逻任务。
```

### 取消、暂停和继续

取消当前导航或巡逻：

```bash
rosrun ros_fyoyi waypoint_cli cancel
```

暂停巡逻：

```bash
rosrun ros_fyoyi waypoint_cli pause
```

继续巡逻：

```bash
rosrun ros_fyoyi waypoint_cli resume
```

暂停时会取消当前 `move_base` 目标。继续后，会从当前航点重新发送导航目标。

### 查看当前状态

查看 waypoint_server 当前在做什么：

```bash
rosrun ros_fyoyi waypoint_cli status
```

输出类似：

```text
当前状态：循环巡逻
当前目标：room
巡逻队列：[room] -> ketin -> fangjian
循环巡逻：是
上次结果：任务已启动
```

也可以直接调用服务：

```bash
rosservice call /fyoyi/status "{}"
```

### 实时查看状态和导航日志

如果希望像日志窗口一样持续查看小车当前状态，运行：

```bash
rosrun ros_fyoyi waypoint_cli watch
```

默认每 1 秒刷新一次状态，同时监听 `/fyoyi/navi_result`，有导航完成、失败、暂停、取消等结果时会立即打印。

也可以指定刷新间隔，例如每 0.5 秒刷新一次：

```bash
rosrun ros_fyoyi waypoint_cli watch 0.5
```

退出实时查看：

```text
Ctrl+C
```

### 删除航点

方式一：在 RViz 里删除。

在 Displays 里打开：

```text
Fyoyi Editable Waypoints
```

然后右键点击要删除的航点，选择：

```text
删除这个航点
```

方式二：命令行删除。

例如删除 `wp_1`：

```bash
rostopic pub /fyoyi/delete_waypoint std_msgs/String "data: 'wp_1'" -1
rosservice call /fyoyi/save_waypoints "{}"
```

也可以写成：

```bash
rosrun ros_fyoyi waypoint_cli delete wp_1
rosrun ros_fyoyi waypoint_cli save
```

默认会自动保存。第二条 `save` 命令是手动确认保存一次。

### 重命名航点

例如把 `wp_1` 改成 `door`：

```bash
rosrun ros_fyoyi waypoint_cli rename wp_1 door
rosrun ros_fyoyi waypoint_cli save
```

也可以直接发布话题：

```bash
rostopic pub /fyoyi/rename_waypoint std_msgs/String "data: 'wp_1 door'" -1
rosservice call /fyoyi/save_waypoints "{}"
```

注意：

```text
航点名字只能包含英文、数字、下划线和横线，不能包含空格或中文。
默认会自动保存。为了确认写入成功，也可以再手动运行一次 `save`。
```

### 航点工具帮助命令

查看航点工具支持哪些命令：

```bash
rosrun ros_fyoyi waypoint_cli help
```

会显示：

```text
ros_fyoyi 航点工具

用法:
  rosrun ros_fyoyi waypoint_cli help
  rosrun ros_fyoyi waypoint_cli list
  rosrun ros_fyoyi waypoint_cli save
  rosrun ros_fyoyi waypoint_cli reload
  rosrun ros_fyoyi waypoint_cli clear
  rosrun ros_fyoyi waypoint_cli nav <waypoint_name>
  rosrun ros_fyoyi waypoint_cli patrol [--loop] <waypoint_1> <waypoint_2> ...
  rosrun ros_fyoyi waypoint_cli cancel
  rosrun ros_fyoyi waypoint_cli pause
  rosrun ros_fyoyi waypoint_cli resume
  rosrun ros_fyoyi waypoint_cli status
  rosrun ros_fyoyi waypoint_cli watch [刷新秒数]
  rosrun ros_fyoyi waypoint_cli delete <waypoint_name>
  rosrun ros_fyoyi waypoint_cli rename <old_name> <new_name>
  rosrun ros_fyoyi waypoint_cli name <next_waypoint_name>
```

### 重新加载航点

如果你手动改了 `config/waypoints.yaml`，可以运行：

```bash
rosservice call /fyoyi/reload_waypoints "{}"
```

### 单独启动航点工具

如果 `map_server`、`amcl`、`move_base` 已经由其他 launch 启动，你只想启动航点工具和 RViz：

```bash
roslaunch ros_fyoyi waypoint_tools.launch
```

如果不想启动 RViz：

```bash
roslaunch ros_fyoyi waypoint_tools.launch use_rviz:=false
```

### 换航点文件

例如你希望每个地图有自己的航点文件：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  waypoints_file:=/home/fyoyi/maps/lab_waypoints.yaml
```

### 和 WaterPlus 的区别

本包自带航点功能保留了最常用部分：

```text
添加航点
加载航点
保存航点
显示航点
按名字导航
返回导航结果
```

暂时没有做 WaterPlus 的这些扩展：

```text
RViz 自定义插件按钮
交互拖动修改航点
右键删除航点
充电桩 Charger 管理
UDP 远程控制
```

这样做的原因是第一版更容易理解，也更适合你之后自己扩展。

## 默认配置

当前默认值在 `launch/waypoint_nav.launch` 中定义：

```text
地图文件:       $(find ros_fyoyi)/maps/home/map.yaml
RViz 配置:      $(find ros_fyoyi)/rviz/Local_plan.rviz
地图坐标系:     map
里程计坐标系:   odom
底盘坐标系:     base_footprint
雷达话题:       /scan
里程计话题:     odom
机器人半径:     0.25
TEB 半径:       0.17
膨胀半径:       0.5
障碍物检测距离: 1.0
光线清除距离:   6.0
全局规划器:     global_planner/GlobalPlanner
局部规划器:     teb_local_planner/TebLocalPlannerROS
控制频率:       10.0
启动 RViz:      true
启动 WaterPlus: true
```

## 常用运行示例

### 1. 使用默认地图导航

```bash
roslaunch ros_fyoyi waypoint_nav.launch
```

### 2. 换一张地图

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  map_file:=/home/user/maps/lab.yaml
```

地图文件必须是 `.yaml`，并且 yaml 中的 `image:` 能找到对应 `.pgm` 或 `.png`。

### 3. 换雷达话题

如果机器人雷达发布的是 `/scan_filtered`：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  scan_topic:=/scan_filtered
```

如果机器人雷达发布的是 `/rplidar/scan`：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  scan_topic:=/rplidar/scan
```

### 4. 换底盘坐标系

如果你的机器人没有 `base_footprint`，只有 `base_link`：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  base_frame:=base_link
```

### 5. 换里程计话题

如果 TEB 要用的里程计话题是 `/wheel/odom`：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  odom_topic:=/wheel/odom
```

注意：`odom_topic` 是 TEB 使用的里程计话题，`odom_frame` 是 TF 坐标系名字，它们不是同一个概念。

### 6. 同时适配一台新机器人

例如新机器人：

```text
雷达话题:   /front/scan
里程计话题: /wheel/odom
底盘坐标:   base_link
机器人半径: 0.32
地图文件:   /home/user/maps/office.yaml
```

可以这样启动：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  map_file:=/home/user/maps/office.yaml \
  scan_topic:=/front/scan \
  odom_topic:=/wheel/odom \
  base_frame:=base_link \
  robot_radius:=0.32 \
  teb_footprint_radius:=0.32
```

### 7. 不启动 RViz

适合远程机器人或无显示环境：

```bash
roslaunch ros_fyoyi waypoint_nav.launch use_rviz:=false
```

### 8. 不启动 WaterPlus 航点工具

只使用 RViz 的 `2D Nav Goal` 导航：

```bash
roslaunch ros_fyoyi waypoint_nav.launch use_waterplus:=false
```

### 9. 不启动 RViz，也不启动 WaterPlus

只启动最核心导航：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  use_rviz:=false \
  use_waterplus:=false
```

## 参数文件说明

`launch/waypoint_nav.launch` 负责启动节点和定义入口参数。

`config/costmap_common_params.yaml` 同时加载到全局 costmap 和局部 costmap，主要控制：

```text
robot_radius      机器人半径
inflation_radius  障碍物膨胀半径
obstacle_range    多远以内的雷达障碍物会标记进 costmap
raytrace_range    多远以内的雷达空闲区域会清除障碍物
scan_topic        雷达话题
```

`config/global_costmap_params.yaml` 控制全局代价地图：

```text
global_frame      默认 map
robot_base_frame  默认 base_footprint
static_map        true，表示使用 map_server 发布的静态地图
recovery_behaviors 恢复行为
```

`config/local_costmap_params.yaml` 控制局部代价地图：

```text
global_frame      默认 odom
rolling_window    true，局部地图跟着机器人移动
width/height      局部地图尺寸
update_frequency  更新频率
```

`config/teb_local_planner_params.yaml` 控制 TEB 局部规划器：

```text
odom_topic               里程计话题
max_vel_x                最大前进速度
max_vel_x_backwards      最大后退速度
max_vel_theta            最大角速度
acc_lim_x                最大线加速度
acc_lim_theta            最大角加速度
min_obstacle_dist        与障碍物最小距离
footprint_model.radius   TEB 使用的机器人半径
xy_goal_tolerance        到达目标的平移容差
yaw_goal_tolerance       到达目标的角度容差
```

## 如何修改参数

### 临时修改

临时修改推荐用 launch 参数：

```bash
roslaunch ros_fyoyi waypoint_nav.launch robot_radius:=0.30
```

这种方式不会改文件，适合测试。

### 永久修改默认值

如果你想以后默认就是某个值，修改：

```text
launch/waypoint_nav.launch
```

例如把：

```xml
<arg name="base_frame" default="base_footprint"/>
```

改成：

```xml
<arg name="base_frame" default="base_link"/>
```

### 修改算法细节

如果要调规划效果，修改 `config/*.yaml`。

例如机器人离障碍物太近，可以调大：

```yaml
min_obstacle_dist: 0.2
inflation_dist: 0.5
```

如果机器人速度太快，可以调小：

```yaml
max_vel_x: 0.25
max_vel_theta: 0.6
```

如果局部避障范围太小，可以调大：

```yaml
local_costmap:
  width: 4.0
  height: 4.0
```

## 如何换机器人

换机器人时，先不要改代码，按下面顺序排查：

1. 雷达话题是否是 `/scan`
2. 里程计话题是否是 `/odom`
3. 底盘坐标是否是 `base_footprint`
4. 机器人真实半径是多少
5. 底盘是否订阅 `/cmd_vel`
6. TF 是否完整

如果话题和坐标不同，通过 launch 参数覆盖。

如果机器人尺寸不同，至少修改：

```bash
robot_radius:=实际半径
teb_footprint_radius:=实际半径
```

如果机器人不是圆形，后续可以把 TEB 的 footprint 从 circular 改成 polygon。

## 如何换地图

把地图文件放到任意目录都可以，只要启动时传入 yaml：

```bash
roslaunch ros_fyoyi waypoint_nav.launch map_file:=/home/user/maps/new_map.yaml
```

如果希望地图跟随本包上传到 GitHub，可以放到：

```text
maps/地图名/map.yaml
maps/地图名/map.pgm
```

然后启动：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  map_file:=$(rospack find ros_fyoyi)/maps/地图名/map.yaml
```

在 launch 文件里不能直接使用 `$(rospack find ...)`，这里只是 shell 命令示例。更常见做法是把默认值写到 launch：

```xml
<arg name="map_file" default="$(find ros_fyoyi)/maps/地图名/map.yaml"/>
```

## 如何扩展

### 增加一个新地图

新增目录：

```text
maps/lab/
  map.yaml
  map.pgm
```

运行：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  map_file:=$(rospack find ros_fyoyi)/maps/lab/map.yaml
```

### 增加一套机器人配置

当前包通过 launch 参数适配机器人。后续如果机器人很多，可以增加多个 launch：

```text
launch/wpb_home_nav.launch
launch/my_robot_nav.launch
launch/lab_robot_nav.launch
```

这些 launch 内部 include `waypoint_nav.launch`，并传入不同参数。

示例结构：

```xml
<launch>
  <include file="$(find ros_fyoyi)/launch/waypoint_nav.launch">
    <arg name="base_frame" value="base_link"/>
    <arg name="scan_topic" value="/front/scan"/>
    <arg name="robot_radius" value="0.32"/>
  </include>
</launch>
```

这样 `waypoint_nav.launch` 保持通用，具体机器人用单独 launch 包装。

### 替换局部规划器

当前默认局部规划器：

```text
teb_local_planner/TebLocalPlannerROS
```

如果以后想换成 DWA，可以传参：

```bash
roslaunch ros_fyoyi waypoint_nav.launch \
  local_planner:=dwa_local_planner/DWAPlannerROS
```

但仅传这个还不够，DWA 需要自己的参数文件。更规范的做法是新增：

```text
config/dwa_local_planner_params.yaml
launch/dwa_waypoint_nav.launch
```

### 增加自己的任务节点

如果你要写“按顺序去多个航点”的节点，可以让它发布：

```text
/move_base_simple/goal
```

或者使用 WaterPlus 的航点接口。

你的任务节点不应该直接控制底盘，除非是在手动控制模式。导航模式下建议让 `move_base` 统一发布 `/cmd_vel`。

## 常见问题

### RViz 里没有地图

检查：

```bash
rostopic echo /map_metadata
```

如果没有输出，说明 `map_server` 没有成功加载地图。检查 `map_file` 路径和 yaml 中的 `image:`。

### RViz 里机器人位置不对

检查 TF：

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_footprint
```

如果 `map -> odom` 没有，通常是 AMCL 没定位成功。

如果 `odom -> base_footprint` 没有，通常是机器人 bringup 没有发布里程计 TF。

### 点了 2D Pose Estimate 后机器人不动

这是正常的。`2D Pose Estimate` 只是设置初始定位。

要让机器人移动，需要使用：

```text
2D Nav Goal
```

### 点了 2D Nav Goal 后机器人不动

检查：

```bash
rostopic echo /move_base/status
rostopic echo /cmd_vel
```

如果 `/cmd_vel` 没速度，检查目标点是否可达、costmap 是否被障碍物堵死、TF 是否正常。

如果 `/cmd_vel` 有速度但机器人不动，检查底盘驱动是否订阅 `/cmd_vel`。

### 机器人靠墙太近

尝试调大：

```text
robot_radius
inflation_radius
TebLocalPlannerROS/min_obstacle_dist
TebLocalPlannerROS/inflation_dist
```

### 机器人速度太快

修改：

```text
config/teb_local_planner_params.yaml
```

重点参数：

```yaml
max_vel_x
max_vel_x_backwards
max_vel_theta
acc_lim_x
acc_lim_theta
```

### 不想用 WaterPlus

启动时关闭：

```bash
roslaunch ros_fyoyi waypoint_nav.launch use_waterplus:=false
```

## 推荐提交到 GitHub 的内容

建议保留：

```text
launch/
config/
maps/
rviz/
README.md
package.xml
CMakeLists.txt
```

不要提交：

```text
build/
devel/
install/
log/
.catkin_tools/
.vscode/
*.bag
```

地图文件如果不大，可以提交；如果地图很大，再考虑是否单独管理。
