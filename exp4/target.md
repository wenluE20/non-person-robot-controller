**网络设置（动捕网络、机器人网络）**

在 ROS 端中，主要是利用 Vicon_bridge 功能包，读取局域网中已定义好的刚体信
息，并发布到 ROS 的话题广场中。具体操作如下：

步骤 1：连接到无线局域网
将电脑连接到和动捕系统同一个 WiFi，使用局域网名称：vicon。

步骤 2：下载并编译 vicon_bridge
主目录下打开终端，创建一个工作空间，下载并编译 vicon_bridge
mkdir -p vicon_ws/src
cd vicon_ws/src
git clone https://github.com/ethz-asl/vicon_bridge.git
cd ..
catkin_make

步骤 3：修改 launch 文件配置
修改功能包内 src/launch 文件夹下 vicon.launch 文件中“datastream_hostport”的值为
“192.168.10.1:801”，保存后退出。

步骤 3：运行
执行命令
source ~/vicon_ws/devel/setup.bash
roslaunch vicon_bridge vicon.launch

步骤 4：查看 vicon 的话题消息
执行命令
rostopic echo /vicon/ge/ge
备注说明：ge 是在动捕系统上建立的刚体的名字
可以查看到机器人位姿的动态数据消息。

**实验任务**
使用动作捕捉系统获取移动机器人实时位置信息、锥桶的位置信息，编写程序规划路
径，令机器人由当前位置走到锥桶附近停下。

提示：将 exp4 功能包添加至之前的工作空间下，补充 src 文件夹下 motionCapture.cpp 代码
的注释部分即可，需补充的代码共四个部分：

1、第一部分需定义处理机器人位置信息的回调函数，该函数接收 TransformStamped 类型
的消息，该消息下 transform.translation 的 x、y 值即为机器人的 x、y 坐标，将其赋给先
前定义的全局变量即可在主函数中调用机器人位置信息。
2、锥桶位置信息的回调函数，同第一部分。
3、第三部分订阅 vicon 发布的机器人与锥桶的位置话题（注意话题名称，在 rostopic list 下
可以查看），将前面定义的回调函数名作为参数传入。
4、第四部分根据机器人与锥桶之间实时的位置关系，发布速度话题控制机器人朝着锥桶
移动，距离小于某个阈值时停下即可，可参考之前实验的代码。

