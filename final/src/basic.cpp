#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//====== 独立可调参数 ======//
const int MAX_CYCLES = 5;          // 蛇形循环次数
const double TURN_DURATION = 6.8;  // 单次转弯时间(s)
const double BASE_SPEED = 0.35;     // 基础线速度(m/s)
const double TURN_ANGLE = 0.4;     // 基础转向速度(rad/s)

//====== 主控制逻辑 ======//
int main(int argc, char** argv) {
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;
    
    // 运动控制
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
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

        // 生成基础运动指令
        cmd.linear.x = BASE_SPEED;
        cmd.angular.z = is_turning_right ? -TURN_ANGLE : TURN_ANGLE;
        
        ROS_DEBUG("Publishing cmd_vel - linear.x: %.2f, angular.z: %.2f", 
                 cmd.linear.x, cmd.angular.z);

        // 发布控制指令
        vel_pub.publish(cmd);
        ros::spinOnce();
        ros::Rate(15).sleep();
    }

    // 安全停止
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    vel_pub.publish(cmd);
    
    ROS_INFO("Motion completed. Stopping robot.");
    
    return 0;
}