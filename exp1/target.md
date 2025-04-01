**实验一：ROS工程的创建和移动机器人的使用**

一、基础
1.Linux基本命令
*打开/添加终端：ctrl + alt + t / ctrl + shift + t。
*进入某路径：cd + [路径]

备注说明：ubuntu 文件系统根目录挂载在“\”下，home 目录挂载在“~”下，
home 目录是用户文件存放的地方，修改不需要系统权限；根目录是系统配置文
件存放的地方，修改需要系统权限。我们常操作的是 home 目录下的文件，例：
cd ~/sdv_ws。

*显示某路径下的所有文件：ls + [路径]
*删除某文件/文件夹：rm -rf + [文件/文件夹路径]

备注说明：若该文件在系统路径下，我们则需在前面加上 sudo，即 sudo rm 
-rf + [文件/文件夹路径]。

*复制某文件：cp + [源文件路径] + [目标文件路径] 
*新建文件夹：mkdir -p + [创建文件夹路径]，其中 -p 为递归创建文件夹
的作用。
*终止进程：ctrl + c 
*在命令行中复制或粘贴语句：ctrl + shift + c , ctrl + shift +v 
*新建某文件：touch + [文件路径]
*修改某文件：gedit/vim + [文件路径]
其中 gedit 为可视化界面，操作简单；vim 为命令行窗口，操作较为复杂。

2.ROS 话题消息常用命令
rostopic -h （查看帮助）
rostopic list （查看已经发布或订阅的话题）
rosrun rqt_graph rqt_graph （查看各话题间的发布订阅关系）
rostopic echo + 话题名称 (查看发布内容)
rostopic type + 话题名称 (查看消息类型)
rostopic pub [topic] [msg_type] [args] （在命令行发布话题内容，可以使用 tab
查看联想内容）
rosmsg show + 消息类型 （查看消息类型的细节，可以使用 tab 查看系统提供
的各种消息类型）

二、实验1：创建ROS工程
鼠标右键或使用快捷键 ctrl+alt+T 打开一个终端
# 启动 ros Master:
roscore

再打开一个新的终端，创建 sdv_ws 工作空间并建立 my_robot 功能包
# 创建工作空间 sdv_ws 的文件夹
mkdir -p ~/sdv_ws/src
cd ~/sdv_ws/
catkin_make

# 创建功能包 my_robot
cd ~/sdv_ws/src
catkin_create_pkg my_robot std_msgs rospy roscpp
cd ~/sdv_ws
catkin_make

# 添加环境变量
source ~/sdv_ws/devel/setup.bash

编写一个节点 robot_moving 发布速度话题，控制移动机器人运动，具体步骤
如下：
# 创建 cpp 文件
cd my_robot 
(注意：打开 my_robot 的完整路径是~/sdv_ws/src/my_robot，如果要退出至
上一目录，使用命令 cd ..)
gedit src/robot_moving.cpp

添加代码(见同目录下code_1)

编写完成后，保存并退出
# 修改 my_robot 功能包下的 CMakeLists.txt
gedit CMakeLists.txt 
(注意：此时的完整路径是~/sdv_ws/src/my_robot)
内容(见同目录下CMake_1)

# 保存程序文件，编译并运行节点：
cd ~/sdv_ws
catkin_make
source devel/setup.bash
rosrun my_robot robot_moving
（注意：此时如果只打开了一个 rosrun 的终端，需要再打开一个 roscore 的
终端，不然会报错）

# 再打开一个新的终端，输入以下命令
rostopic list #列出所有的 ros 话题

可以观察到除了 roscore 自带的 /rosout 和 /rosout_agg 话题外 ， 多 了
robot_moving 节点发布的/cmd_vel 话题。

# 使用 rostopic echo 查看该话题上发布的消息
rostopic echo /cmd_vel #打印出该话题发布的消息内容

可以观察到 x 方向线速度数据和 z 轴角速度。

将笔记本连接到移动机器人（注意：务必将机器人上的白色 USB 线接到笔
记本右侧的接口上，否则无法正常启动机器人）。
使用 ctrl+c 关闭运行中的 roscore 节点

# 执行以下命令启动移动机器人底盘
roslaunch dashgo_driver driver_imu.launch

再次启动 robot_moving 节点，可以观察到移动机器人运动，如果要停止机器
人运动，使用 ctrl+c 关闭 robot_moving 节点，或者，按下机器人上的红色急停按
钮后，再使用 ctrl+c 关闭 robot_moving 节点。
（注意：程序停止后，要复位机器人上的红色急停按钮，不然下次运行程序机
器人处于暂停状态，将不会运动）

自行了解/cmd_vel 话题中参数的意义，改变 robot_moving 节点代码中的参
数，以熟悉如何控制移动机器人进行简单的移动。

三、实验2：摄像头数据读取与机器人运动
在实验 1 创建的功能包 my_robot 中编写第二个节点 robot_camera 来调用机
器人摄像头，并订阅 robot_moving 节点发布的速度话题。
# 进入 my_robot 功能包，创建 cpp 文件：
cd ~/sdv_ws/src/my_robot 
gedit src/robot_camera.cpp

添加代码(见同目录下code_2)

# 新节点的加入需要修改 my_robot 功能包下的 CMakeLists.txt。
内容(见同目录下CMake_2)

# CMakeList 修改完成后，需要重新编译工作空间
cd ~/sdv_ws
catkin_make
source ~ /devel/setup.bash

根据启动摄像头的不同，自行选择执行以下程序。
①启动笔记本摄像头
robot_camera 代码第 8 行处定义了枚举变量 CameraState，若希望打开笔记本
摄像头，修改代码第 14 行处的 state 变量修改为 COMPUTER，保存程序并编译，
执行以下命令：

# 打开一个终端，执行以下命令：
roscore

# 再打开一个新终端，执行以下命令：
rosrun my_robot robot_camera

②启动 REALSENSE 摄像头
修改 robot_camera 代码的第 14 行处的 state 变量修改为 REALSENSE

# 执行以下命令：
roslaunch realsense2_camera rs_rgbd.launch

# 再打开一个新终端，执行以下命令：
rosrun my_robot robot_camera

③启动 ZED 摄像头
修改 robot_camera 代码的第 14 行处的 state 变量修改为 ZED，并修改 else 
if(state==ZED)下的 capture.open()括号中的数字为 1 或 2 或 3 或 4，每修改一次，
记得编译

# 再执行以下命令：
roscore

# 再打开一个新终端，执行以下命令：
rosrun my_robot robot_camera

编写 launch 文件启动多个节点
ROS 工程一般包含多个节点，ROS 提供了 roslaunch 工具以同时启动多个节
点，下面编写 launch 文件同时启动 robot_moving 和 robot_camera 节点。

# 在功能包 my_robot 路径下新建 launch/move_and_camera.launch 文件
source ~/sdv_ws/devel/setup.bash && roscd my_robot
mkdir launch
gedit launch/move_and_camera.launch

move_and_camera.launch 文件文件内容(见同目录文件launch_2)

# 执行 move_and_camera.launch 以同时启动两个节点，会打开摄像头并让机器人运动。
source ~/sdv_ws/devel/setup.bash
roslaunch my_robot move_and_camera.launch

四、实验3：激光雷达数据读取与机器人运动（拓展）激光雷达数据的读取方法
新旧两个版本机器人搭载不同的摄像头和激光雷达，其中旧版本机器人搭载
ZED 摄像头和力策激光雷达，而新机器人搭载 Realsense 摄像头和 G4 雷达，两
种雷达对应不同的启动方式，其中：
# 力策激光雷达启动方式：
roslaunch ltme01_driver ltme01.launch

# G4 激光雷达启动方式：
roslaunch ydlidar_ros_driver ydlidar1_up.launch

# 成功启动激光雷达后，打开一个新终端，输入命令：
rostopic echo /scan

可以看到大量帧的激光雷达点云数据，点云数据含义解释如下：
Seq #扫描顺序增加的 id，可以理解为扫描的次数
Stamp # ros 中表示时间的一种结构，时间辍包含了开始扫
描的时间和与开始扫描的时间差
frame_id #扫描的参考系名称.注意扫描是逆时针从正前方开始
的
float32 angle_min # scan 的开始扫描角度(弧度) 
float32 angle_max # scan 的结束扫描角度(弧度) 
float32 angle_increment # 测量的角度间的距离(弧度) 
float32 time_increment # 测量的时间间隔(s) 
float32 scan_time #扫描的时间间隔(s) 
float32 range_min #最小的测量距离(m) 
float32 range_max #最大的测量距离(m) 
float32[] ranges #测量的距离数据(注意: 值 < range_min 
或 > range_max 应当被丢弃) 
float32[] intensities #强度数据

数据信息：逆时针 0~360°，每隔 angle_increment 存储一次，最终数据在 ranges 
中。由于距离激光雷达过近的数据不可靠，因此输出为 0.0，也有部分雷达默
认是 inf，输出 0 还是 inf 可以通过 launch 文件进行修改。

