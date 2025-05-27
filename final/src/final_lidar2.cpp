#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <detect_lidar/LidarDetect.h>
#include <cmath>
#include <vector>
#include <algorithm>

ros::Publisher vel_pub;
const float SAFE_DISTANCE = 1.0;
const float ANGLE_GAIN = -0.15;    // 只在DRIVE3阶段生效
float target_angle = 0;
geometry_msgs::Twist twist;
// 运动参数
const int MAX_CYCLES = 5;               // 总循环次数
const double DRIVE1_DURATION = 4.0;     // 第一段直行时间
const double DRIVE2_DURATION = 4.8;     // 第二段直行时间 
const double DRIVE3_DURATION = 4.0;     // 第三段直行时间
const double TURN_DURATION = 3.7;  // 90度转向时间(角度/角速度)
const double TURN_SPEED = 0.7;          // 转向角速度 
const double BASE_SPEED = 0.3;          // 基础线速度

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
        
    // if(distance_between < SAFE_DISTANCE) {
    //     ROS_WARN("Channel too narrow (%.2fm), stopping!", distance_between);
    //     twist.linear.x = 0.0;
    //     twist.angular.z = 0.0;
    // } 
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
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "enhanced_square_wave");
    ros::NodeHandle nh;
 
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("cones", 10, conesCallback);

    enum State { DRIVE1, TURN1, DRIVE2, TURN2, DRIVE3 };
    State current_state = DRIVE1;
    bool turn_left = false;  // 转向方向控制
    int cycle_count = 0;
    ros::Time state_start_time = ros::Time::now();
    geometry_msgs::Twist cmd;

    ROS_INFO("启动增强型方波导航");
    ROS_INFO("周期数:%d 转向速度:%.1frad/s", MAX_CYCLES, TURN_SPEED);

    while(ros::ok() && cycle_count < MAX_CYCLES) {
        ros::spinOnce();
        double elapsed = (ros::Time::now() - state_start_time).toSec();

        switch(current_state) {
            case DRIVE1: {
                // 第一阶段直行（无修正）
                cmd.linear.x = BASE_SPEED;
                cmd.angular.z = 0.0;
                vel_pub.publish(cmd);

                if(elapsed >= DRIVE1_DURATION) {
                    ROS_INFO("[周期%d] 直行1完成 转向方向:%s", 
                           cycle_count+1, turn_left ? "左" : "右");
                    current_state = TURN1;
                    state_start_time = ros::Time::now();
                }
                break;
            }

            case TURN1: {
                // 第一次转向（与周期方向一致）
                cmd.linear.x = 0.0;
                cmd.angular.z = turn_left ? TURN_SPEED : -TURN_SPEED;
                vel_pub.publish(cmd);

                if(elapsed >= TURN_DURATION) {
                    ROS_INFO("[周期%d] 转向1完成", cycle_count+1);
                    current_state = DRIVE2;
                    state_start_time = ros::Time::now();
                }
                break;
            }

            case DRIVE2: {
                // 第二阶段直行（无修正）
                cmd.linear.x = BASE_SPEED;
                cmd.angular.z = 0.0;
                vel_pub.publish(cmd);

                if(elapsed >= DRIVE2_DURATION) {
                    ROS_INFO("[周期%d] 直行2完成", cycle_count+1);
                    current_state = TURN2;
                    state_start_time = ros::Time::now();
                }
                break;
            }

            case TURN2: {
                // 第二次转向（与周期方向一致）
                cmd.linear.x = 0.0;
                cmd.angular.z = turn_left ? TURN_SPEED : -TURN_SPEED;
                vel_pub.publish(cmd);

                if(elapsed >= TURN_DURATION) {
                    ROS_INFO("[周期%d] 转向2完成", cycle_count+1);
                    current_state = DRIVE3;
                    state_start_time = ros::Time::now();
                }
                break;
            }

            case DRIVE3: {
                // 第三阶段直行（启用雷达修正）
                cmd.linear.x = BASE_SPEED;
                cmd.angular.z = ANGLE_GAIN * target_angle;
                vel_pub.publish(cmd);

                if(elapsed >= DRIVE3_DURATION) {
                    ROS_INFO("[周期%d] 直行3完成 累计修正:%.2frad", 
                           cycle_count+1, target_angle);
                    cycle_count++;
                    turn_left = !turn_left;  // 切换下一周期转向方向
                    current_state = DRIVE1;
                    state_start_time = ros::Time::now();
                }
                break;
            }
        }

        ros::Rate(20).sleep();
    }

    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    vel_pub.publish(cmd);
    ROS_INFO("导航完成 总执行周期:%d", cycle_count);

    return 0;
}