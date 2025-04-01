**实验三 激光雷达+聚类实验**
1.运行并查看雷达话题

步骤 1：启动雷达
# 如果是力策雷达，运行以下命令启动雷达 source ./devel/setup.bash
roslaunch ltme01_driver ltme01.launch

# 如果是 G4 雷达，运行以下命令启动雷达
roslaunch ydlidar_ros_driver ydlidar1_up.launch

注意：启动后请不要按刹车按钮，避免对雷达造成损坏。

步骤 2：查看雷达话题
# 新建终端，输入
rostopic echo /scan
运行几秒钟后使用 ctrl+c停止查看消息，上滑可以看到雷达的话
题

数据含义解释如下：
Seq #扫描顺序增加的 id，可以理解为扫描的次数
Stamp # ros 中表示时间的一种结构，时间辍包含了开始扫描的时间和
与开始扫描的时间差
frame_id #扫描的参考系名称.注意扫描是逆时针从正前方开始的
float32 angle_min # scan 的开始扫描角度(弧度)
float32 angle_max # scan 的结束扫描角度(弧度)
float32 angle_increment # 测量的角度间的距离(弧度)
float32 time_increment # 测量的时间间隔(s)
float32 scan_time # 扫描的时间间隔(s)
float32 range_min #最小的测量距离(m)
float32 range_max # 最大的测量距离(m)
float32[] ranges #测量的距离数据(注意: 值 < range_min 或 > 
range_max 应当被丢弃)
float32[] intensities #强度数据

数据信息：逆时针0~360°，每隔angle_increment存储一次，最
终数据在 ranges 中。由于距离激光雷达过近的数据不可靠，因此输
出为 0.0，也有部分雷达默认是 inf，输出 0 还是 inf 可以通过 launch
文件进行修改。

步骤 3：可视化查看
# 新建终端，输入
rviz

设置左侧的 Global Options 下的 fixed frame，选择 base_footprint。

单击左下角 add 可以增加模块，在该实验中需要加入两个模块，
分别是 LaserScan 和 Axes，Axes 为坐标轴，可以显示固定坐标系的
位置和朝向，而 LaserScan 可以将点云数据可视化。
 
添加 LaserScan 后，选择 topic 为/scan。

补充操作小知识：Rviz 画布调整方法
按住“shift”+鼠标左键可以调整整个画布的位置；
滚动轮可以调整画布的放大和缩小；
按住左键可旋转画布；

2.使用聚类 Kmeans 算法识别锥桶
雷达的点云数据包括了角度和距离，如果要实现避障可以保证
每一个点与机器人距离大于一个值，要精确计算出某一个障碍物与
雷达的距离，可以使用聚类等方法将雷达的点云数据进行分类，提
取出属于同一个物体的点，再计算这些点与雷达的平均距离。

将 Kmeans 聚类算法程序包 detect_lidar 添加到 dashgo_ws 工作空
间的 src 下，删除 dashgo_ws 工作空间中的 build 和 devel 文件，编译。

detect_lidar 功能包中包含以下几个子文件夹：
include：存储 C++中的.h 文件，此文件夹为空。
launch：其中已经包含了力策雷达的启动，如果需要更换为
G4 雷达，则需要将程序中 2-19 行的内容替换为 G4 雷达 launch
文件的内容，根据文件内的注释可以对聚类的相关参数进行调
整。
msg：自定义的话题类型，可以发送锥桶的坐标话题，本实
验中无需订阅该话题。
src：含有.cpp 文件，为本功能包处理雷达消息的主体程序，
通过订阅雷达点云话题，根据 launch 中设置的参数对点云进行取
舍后，利用 Kmeans++算法进行聚类。
CMakeLists.txt：对 CMake 编译进行设置。
package：对依赖包进行管理。

# 步骤 1：调用 Kmeans++聚类算法，打开终端输入以下命令
roslaunch detect_lidar detect_lidar.launch

上述命令是调用了雷达处理功能包中的 detect_lidar.launch 文件，
该文件启动雷达，同时，启动了对雷达数据的处理。

注意：如果前面的雷达程序没有停止，雷达会因被占用而无法
调用，导致程序无法正常运行。

当聚类程序被成功启动后，将会出现一个窗口显示机器人与锥
桶等障碍物信息的图片

launch 设定的范围进行绘制（具体可参考程序 422-456 行内容），其
中白色点为点云数据，红色圈为聚类的结果，黄色为处理点云数据
范围，当周围环境空旷且仅有锥桶时，识别效果较好。

力策雷达的 launch 文件内容见同目录下launch_1：

G4 雷达的 launch 文件内容见同目录下launch_2：

调整雷达半径、角度可以避免过远的点带来的数据精度问题，
但不会影响程序响应速度（雷达转速固定），调整角度可以避免车身
的干扰以及改善数据处理速度。

提高 Kmeans 循环次数可以提高聚类的稳定性，但会牺牲运行速
度，过多的循环次数会出现饱和情况。

参数——Proportion_XY，通过缩放每一个点的坐标以实现点云
数据的取舍。
假设机器人右前方只有一个锥桶，当该
值为 1 时，锥桶表示为绿点，当该值为 0.5 时，锥桶表示为红点，如
果蓝线为聚类处理的范围（在本例程中，该范围为固定值），那么该
值为 1 时锥桶的数据便不会被处理，而是被丢弃。

步骤 2：在机器人前放置两个锥桶，在 launch 文件中修改参数
Proportion_XY 的值，以验证 Proportion XY 参数的功能，对于下
面情景，分别取 Proportion_XY 为 0.25、0.5、0.75 可以得到如下
几种聚类结果。
 
Proportion_XY 为 0.25
 
Proportion_XY 为 0.5 Proportion_XY 为 0.75

当该参数为 0.75 时只能看见一个锥桶，而
当该参数为 0.25 时，又有太多多余信息，因此当“最需要关注的
锥桶”距离与图中接近时，参数设置为 0.5 最合适。

步骤 3：查看聚类的结果
# 在上面程序运行的过程中，新建终端，输入以下命令，获取聚类结果发出的话题
rostopic echo /cones

数据可在 msg/LidarDetect.msg 中看到，解释如下：

Seq #聚类顺序增加的 id，可以理解为聚类的次数
Stamp # ros 中表示时间的一种结构，时间辍包含了开始扫描的
时间和与开始扫描的时间差
frame_id #扫描的参考系名称，注意扫描是逆时针从正前方开始的
x #锥桶组相对雷达的 x 坐标
y #锥桶组相对雷达的 y 坐标
z #锥桶组相对雷达的 z 坐标，由于该实验激光雷达为二维雷
达，因此设置为固定值 1。

3。实验任务
3.1 锥桶检测与避障
使用聚类算法检测到一个锥桶，计算锥桶与机器人之间的距离，
在满足一定距离时（自己设定），让机器人做一个转弯的动作，或者
绕锥桶走一周。

提示：让机器人做直线运动并订阅/cones话题，提取距离最近锥桶相
对于机器人的坐标并计算两者之间的距离，当距离小于某个阈值执
行转向等动作。

4.附件（1）——雷达参数
# 力策激光雷达
LITRA ME01
测距原理是脉冲 ToF
角分辨率为 0.18 度
探测最大距离为 30m
测距精度为±2 cm

# G4 雷达
![image](https://github.com/user-attachments/assets/5af8566b-895e-44d8-a73a-ada33ac3d6a3)

附件（2）启动雷达失败解决办法
# 1.使用以下命令查看串口
ls /dev
通过拔插 USB 线确定对应的是哪个 ttyUSB*,一般是 ttyUSB0

# 2.使用命令
udevadm info --attribute-walk --name=/dev/ttyUSB0 | grep devpath
查看对应的 devpath，查询结果是 ATTRS{devpath}="x.x"

# 3.更改串口绑定规则
sudo vim /etc/udev/rules.d/port2.rules

把文件中的 ATTRS{devpath}="x.x"修改成 ATTRS{devpath}="x.4"
（ 例 如 ： 如 果 查 到 的 结 果 是 ATTRS{devpath}="2.1"， 则 修 改
ATTRS{devpath}="2.4"）

补充：vim 文件修改和保存方法
按下“i”键进入插入模式，此时你可以开始编辑文件；
编辑完成后，按 Esc 键退出插入模式；
输入 :wq，保存文件并退出 vim。

# 4.执行以下命令
sudo service udev reload
sudo service udev restart

5.重新插拔 USB 线

6.以上操作后，依然没有雷达数据，则检查雷达硬件连接线（电源
线和数据线），或者 hub 连接线是否有故障。

附件（3）控制机器人运动程序示例见同目录文件code_2
