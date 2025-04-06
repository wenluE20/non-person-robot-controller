#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 定义目标点结构
struct NavigationPoint {
    double x;  //position.x
    double y;  //position.y
    double z;  //orientation.z
    double w;  //orientation.w
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_point_nav");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);	    // 创建MoveBase客户端
    
    ROS_INFO("Wait for move_base server...");
    ac.waitForServer();	// 等待move_base动作服务器启动
    ROS_INFO("Move_base server connected!");
    
    /*** 一、定义导航目标点集合 ***/

    std::vector<NavigationPoint> points = {
    {1.0, 1.0, 0.0, 1.0},  // 中间点1
    {2.0, 0.0, 0.7071, 0.7071},  // 中间点2
    {0.0, 0.0, 0.0, 1.0}   // 返回起点
};

    //遍历目标点集合，依次到达每个目标点
    for(int i = 0; i < points.size(); ++i) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";// 设置目标点的坐标系
        goal.target_pose.header.stamp = ros::Time::now();// 设置目标点的时间戳

        /*** 二、设置目标点的位置和姿态 ***/
        
        goal.target_pose.pose.position.x = points[i].x;
        goal.target_pose.pose.position.y = points[i].y;
        goal.target_pose.pose.orientation.z = points[i].z;
        goal.target_pose.pose.orientation.w = points[i].w;

        ac.sendGoal(goal);        // 发送目标点
    	ac.waitForResult();       // 等待动作完成
    	
    	/*** 三、延迟一段时间 ***/
        
        ROS_INFO("Reached waypoint %d, waiting for 3 seconds...", i+1);
        ros::Duration(3.0).sleep();  // 停留3秒钟
    }
    
    ROS_INFO("Navigation End!");
    return 0;
}
