#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "vicon_bridge/Marker.h"
#include "geometry_msgs/TransformStamped.h"


ros::Publisher vel_pub;
float x_robot, y_robot; //机器人位置坐标
float x_cone,y_cone;	//锥桶位置坐标

/* 一、回调函数处理机器人vicon数据 */
// 该回调函数接收机器人的位置变换消息，提取x、y坐标并存储在全局变量中
void robotPositionCallback(const geometry_msgs::TransformStamped& msg)
{
    // 从消息的平移变换中提取机器人的x、y坐标
    x_robot = msg.transform.translation.x;
    y_robot = msg.transform.translation.y;
}


/* 二、回调函数处理锥桶vicon数据 */
// 该回调函数接收锥桶的位置变换消息，提取x、y坐标并存储在全局变量中
void conePositionCallback(const geometry_msgs::TransformStamped& msg)
{
    // 从消息的平移变换中提取锥桶的x、y坐标
    x_cone = msg.transform.translation.x;
    y_cone = msg.transform.translation.y;
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motionCapture");
    ros::NodeHandle n;

    /* 三、订阅vicon发布的机器人与锥桶位置话题 */
    // 订阅机器人和锥桶的位置话题，使用之前定义的回调函数
    ros::Subscriber robot_sub = n.subscribe("/vicon/robot/robot", 10, robotPositionCallback);
    ros::Subscriber cone_sub = n.subscribe("/vicon/cone/cone", 10, conePositionCallback);

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(5); 
    geometry_msgs::Twist vel_msg;

    while (ros::ok())
    {
    	ros::spinOnce();
    	/* 四、判断机器人与锥桶位置关系，发布速度话题控制机器人运动 */
        // 计算机器人与锥桶之间的距离
        float distance = sqrt(pow(x_robot - x_cone, 2) + pow(y_robot - y_cone, 2));
        
        // 设置距离阈值，小于此值时停止
        float distance_threshold = 0.5; // 单位：米

        if (distance > distance_threshold)
        {
            // 计算朝向锥桶的速度
            float angle = atan2(y_cone - y_robot, x_cone - x_robot);
            
            // 设置线速度和角速度
            vel_msg.linear.x = 0.5; // 前进速度
            vel_msg.angular.z = angle; // 转向角速度
        }
        else
        {
            // 达到阈值，停止
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
        }

        // 发布速度消息
        vel_pub.publish(vel_msg);

        loop_rate.sleep();
    }

    return 0;
}
