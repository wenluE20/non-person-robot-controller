#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <detect_lidar/LidarDetect.h>
#include <cmath>
#include <vector>
#include <algorithm>

// 全局变量
ros::Publisher vel_pub;
const float SAFE_DISTANCE = 1.0;  // 通道宽度安全阈值
const float angle_gain = -0.3;  // 转向增益系数
float target_angle = 0;           //转向修正角度 
geometry_msgs::Twist twist;
//====== 基础运动独立可调参数 ======//
const int MAX_CYCLES = 5;          // 蛇形循环次数
const double TURN_DURATION = 6.8;  // 单次转弯时间(s)
const double BASE_SPEED = 0.35;     // 基础线速度(m/s)
const double TURN_ANGLE = 0.47;     // 基础转向速度(rad/s)

// 锥桶数据结构体
struct Cone {
    float x;
    float y;
    float distance;
};

// 回调函数处理锥桶数据，得到机器人前方的最近的锥桶的之间的中点，并据此得出一个修正角度，只有中点在机器人前方才进行路径修正
void conesCallback(const detect_lidar::LidarDetect::ConstPtr& msg)
{
    std::vector<Cone> valid_cones;
    
    // 收集有效锥桶数据
    for(int i = 0; i < 8; ++i) {
        float x = msg->x[i];
        float y = msg->y[i];
        
        if(fabs(x) > 0.01 || fabs(y) > 0.01) {  // 过滤无效数据
            float dist = sqrt(x*x + y*y);
            valid_cones.push_back({x, y, dist});
        }
    }

    // 需要至少两个锥桶才能形成通道
    if(valid_cones.size() < 2) {
        ROS_WARN("Not enough cones detected (%zu), keep straight", valid_cones.size());
        return;
    }

    // 按距离排序找到最近的两个锥桶
    std::sort(valid_cones.begin(), valid_cones.end(), 
        [](const Cone& a, const Cone& b){ return a.distance < b.distance; });

    Cone& cone1 = valid_cones[0];
    Cone& cone2 = valid_cones[1];

    // 计算中间点坐标
    float mid_x = (cone1.x + cone2.x) / 2;
    float mid_y = (cone1.y + cone2.y) / 2;
    
    if(mid_y > 0){//最近锥桶中点在机器人前方才进行修正
    // 计算转向角度（使用反正切函数）
    target_angle = atan2(mid_x, mid_y);  // 注意坐标系方向
    
    // 计算通道宽度
    float distance_between = sqrt(pow(cone1.x-cone2.x, 2) + pow(cone1.y-cone2.y, 2));
        
    if(distance_between < SAFE_DISTANCE) {
        ROS_WARN("Channel too narrow (%.2fm), stopping!", distance_between);
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    } 
    // else {  //锥桶通道足够宽
    //     ROS_INFO("Navigating to cones: Midpoint(%.2f, %.2f) Angle: %.2f rad", 
    //             mid_x, mid_y, target_angle);
    // }
    }else{//最近锥桶中点在机器人后方则不修正
        target_angle = 0;
    }
    //ROS_INFO(" target_angle: %.2f",  target_angle);
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_lidar");
    ros::NodeHandle nh;
 
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("cones", 10, conesCallback);

    //发布速度：基础+修正

    // 运动控制
    geometry_msgs::Twist cmd;
    // 运动参数
    bool is_turning_right = true;   
    int cycle_count = 0;
    ros::Time turn_start = ros::Time::now();
    ROS_INFO("Starting snake motion control");
    ROS_INFO("MAX_CYCLES: %d, TURN_DURATION: %.2f, BASE_SPEED: %.2f, TURN_ANGLE: %.2f", 
             MAX_CYCLES, TURN_DURATION, BASE_SPEED, TURN_ANGLE);
    while(ros::ok() && cycle_count < MAX_CYCLES) {
        // 基础蛇形节奏控制
        double elapsed = (ros::Time::now() - turn_start).toSec();
        
        ROS_DEBUG("Cycle: %d/%d, Turning: %s, Elapsed: %.2f", 
                 cycle_count, MAX_CYCLES, 
                 is_turning_right ? "RIGHT" : "LEFT", 
                 elapsed);

        if(elapsed > TURN_DURATION) {
            is_turning_right = !is_turning_right;
            turn_start = ros::Time::now();
            cycle_count++;
            
            ROS_INFO("Direction changed to: %s, Cycle count: %d", 
                    is_turning_right ? "RIGHT" : "LEFT", 
                    cycle_count);
        }

        // 生成运动指令
        cmd.linear.x = BASE_SPEED;
        cmd.angular.z = is_turning_right ? -TURN_ANGLE : TURN_ANGLE + angle_gain * target_angle;
        
        // //另一种控制策略
        // //在最近锥桶中点在前的时候，只用转向修正角度
        // cmd.linear.x = BASE_SPEED;
        // if(abs(target_angle))>0.001){
        //     cmd.angular.z = angle_gain * target_angle;
        // }else{
        //     cmd.angular.z = is_turning_right ? -TURN_ANGLE : TURN_ANGLE;
        // }

        ROS_INFO("Publishing cmd_vel - linear.x: %.2f, angular.z: %.2f, target_angle: %.2f", 
                 cmd.linear.x, cmd.angular.z, target_angle);

        // 发布控制指令
        vel_pub.publish(cmd);
        ros::Rate(10).sleep();
        ros::spinOnce();
    }

    // 安全停止
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    vel_pub.publish(cmd);
    ROS_INFO("Motion completed. Stopping robot.");

    return 0;
}