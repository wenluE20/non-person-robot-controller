# 一、gmapping 建图和 navigation 自主导航 

  分别打开建图、rviz、键盘控制节点，使用键盘控制机器人移
动，建图，打开三个终端，依次分别输入以下三个命令：

**roslaunch dashgo_nav gmapping_imu.launch** 

**roslaunch dashgo_rviz view_navigation.launch**

**rosrun teleop_twist_keyboard teleop_twist_keyboard.py**

其中键盘控制节点启动成功后，键盘“i”建表示前进，“，”
表示后退，“j”表示左转，“l”表示右转，“k”表示停止。通过
键盘控制机器人在环境中移动，rviz 中可以观察到实时的建图情
况，待地图建立完成后，打开一个新终端，输入以下命令保存地
图：

**roscd dashgo_nav/maps**

**rosrun map_server map_saver -f K321_map**

保存成功，会获得以下信息：

[ INFO] [1591867676.882396678]: Waiting for the map
map_intensity: 100

[ INFO] [1591867679.148215442]: Received a 544 X 832 map @ 
0.050 m/pix

[ INFO] [1591867679.148239665]: Writing map occupancy data 
to K321_map.pgm

[ INFO] [1591867679.180059395]: Writing map occupancy data 
to K321_map.yaml

[ INFO] [1591867679.180220954]: Done

启动导航，打开两个终端分别输入以下命令：

**roslaunch dashgo_nav navigation_imu.launch map_file:**

**=”~/dashgo_ws/src/dashgo/dashgo_nav/maps/K321_map.yaml”**

**roslaunch dashgo_rviz view_navigation.launch**

其中，map_file:= 后的双引号内为建立地图后生成的.yaml 文
件路径。

注意：rviz 打开后显示机器人默认所在的位置是栅格的中心点，不
一定是机器人实际所在的位置，因此需要检查并设置起点位置，当
激光数据与地图重合时则起点位置正确。

在 rviz 上设置机器人起点位置，如图 5.2 所示，点击 rviz 上
的 2D Pose Estimate 按钮，然后根据机器人实际位置，在地图相
应位置上点击(保持按下鼠标)，并拖动鼠标设置好机器人正确方
向，其中绿色箭头方向即表示机器人方向，当设置好起点位置时，
激光数据与实际环境地图是相匹配的。

在 rviz 上设置机器人目标点位置，点击2D Nav 
Goal 按钮，然后在地图上点击目标点位置，正常情况，机器人会规
划好到目标点的路径，并移动机器人到目标点，在行走过程中，如
果在规划好的路径上出现障碍物，机器人会重新规划新的路径，绕
开障碍物，继续行走。

# 二、移动机器人多点导航 

单点导航仅能满足基础的位置移动需求，而实际应用场景往往
需要机器人按特定顺序访问多个关键位置点。例如在仓储物流中，
机器人需要依次经过多个货架位置并停留，下面我们模拟这个过
程。

在 ROS 的导航功能包集 navigation 中提供了 amcl 功能包，用
于实现导航中的机器人定位。导航包 navigation_imu.launch 中包
含了 AMCL 节点，启动导航功能包后先不要设置目标点，打开一个新
终端，输入：

**rostopic echo /amcl_pose**

即可观察到该话题的消息

其中，(position.x, position.y, orientation.z, orientation.w)这一组参
数描述了机器人的位姿，前两个参数描述了机器人在地图中的 x、y
坐标，后两个参数描述了机器人的姿态，即机器人朝向。

# 实验内容：

在 rviz 上设置机器人起点位置后，通过查看/amcl_pose
话题记录下此时机器人的位姿，作为多点导航的终点，然后推动机
器人至地图中两个不同的点位，分别记录下/amcl_pose 话题输出的
位姿，作为多点导航的中间点，编写代码实现移动机器人的多点导
航，即机器人从起点位置出发，依次经过两个中间点，并返回至起
点，其中在每个中间点停留时间不少于 3s。

**提示：**

将 exp5 功能包拷贝至之前的工作空间下，补充 src 文件夹下
multi_point_nav.cpp 代码的注释部分即可，需补充的代码共三个
部分：

1、第一部分定义导航的目标点集合，用一个 vector 来存放多个目
标点，每个目标点是一个 NavigationPoint 结构体（代码第十行处
定义），将需要导航的目标点姿态填入该 vector 内即可。

2、第二部分设置目标点的位置和姿态，通过 rosmsg show 命令可以
查看 MoveBaseGoal 消息的定义。

其中坐标系 target_pose.header.frame_id 和时间戳 target_pose.header. 
stamp 已经设置好，需要补充位置和姿态共四个分量。

3、第三部分设置一个延时，让机器人导航至目标点后停留几秒钟，
可以使用 ros::Duration 类实现。

