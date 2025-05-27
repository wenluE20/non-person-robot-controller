#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

using namespace cv;  // OpenCV命名空间

//====== 独立可调参数 ======//
const int MAX_CYCLES = 5;          // 蛇形循环次数
const double TURN_DURATION = 3.0;  // 单次转弯时间(s)
const double BASE_SPEED = 0.2;     // 基础线速度(m/s)
const double TURN_ANGLE = 1.0;     // 基础转向速度(rad/s)
const int CONE_THRESH[6] = {0,255,100,255,26,255}; // H,S,V阈值

//====== 完整视觉模块 ======//
class ConeVision {
private:
    cv::Mat current_frame;
    bool new_frame = false;

    // HSI转换函数
    void BGR2HSI(const cv::Mat &src, cv::Mat &hsi) {
        hsi.create(src.size(), src.type());
        
        for(int i=0; i<src.rows; i++) {
            for(int j=0; j<src.cols; j++) {
                Vec3b pixel = src.at<Vec3b>(i, j);
                float b = pixel[0]/255.0f;
                float g = pixel[1]/255.0f;
                float r = pixel[2]/255.0f;

                float numerator = 0.5f * ((r-g) + (r-b));
                float denominator = sqrt((r-g)*(r-g) + (r-b)*(g-b)) + 1e-6f;
                float theta = acos(numerator / denominator);

                float H = (b <= g) ? theta : (2*CV_PI - theta);
                float sum = r + g + b;
                float S = (sum < 1e-6f) ? 0 : 1 - 3*std::min({r, g, b})/sum;
                float I = sum / 3.0f;

                hsi.at<Vec3b>(i, j) = Vec3b(
                    static_cast<uchar>(H * 255 / (2*CV_PI)),
                    static_cast<uchar>(S * 255),
                    static_cast<uchar>(I * 255)
                );
            }
        }
    }

public:
    // 图像回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            current_frame = cv_ptr->image.clone();
            new_frame = true;
        } catch (...) {
            ROS_WARN("图像接收异常");
        }
    }

    // 获取锥桶对中点偏移量（单位：像素）
    bool getMidOffset(float& offset) {
        if(!new_frame || current_frame.empty()) return false;
        
        cv::Mat resized_frame, hsi, mask;
        cv::resize(current_frame, resized_frame, cv::Size(), 0.5, 0.5);
        
        // 颜色空间转换
        BGR2HSI(resized_frame, hsi);
        
        // 颜色阈值处理
        cv::inRange(hsi, 
                  cv::Scalar(CONE_THRESH[0], CONE_THRESH[2], CONE_THRESH[4]),
                  cv::Scalar(CONE_THRESH[1], CONE_THRESH[3], CONE_THRESH[5]),
                  mask);

        // 形态学操作
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, 
                        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 筛选有效锥桶对
        if(contours.size() >= 2) {
            // 按面积排序
            std::sort(contours.begin(), contours.end(),
                    [](auto& a, auto& b){return cv::contourArea(a) > cv::contourArea(b);});

            // 计算中点
            cv::Rect rect1 = cv::boundingRect(contours[0]);
            cv::Rect rect2 = cv::boundingRect(contours[1]);
            cv::Point center1(rect1.x + rect1.width/2, rect1.y + rect1.height/2);
            cv::Point center2(rect2.x + rect2.width/2, rect2.y + rect2.height/2);
            
            // 计算横向偏移量（归一化到[-1,1]）
            offset = ((center1.x + center2.x)/2.0 - mask.cols/2.0) / (mask.cols/2.0);
            return true;
        }
        return false;
    }
};

//====== 主控制逻辑 ======//
int main(int argc, char** argv) {
    ros::init(class ConeVision {
private:
    cv::Mat current_frame;
    bool new_frame = false;

    // HSI转换函数
    void BGR2HSI(const cv::Mat &src, cv::Mat &hsi) {
        hsi.create(src.size(), src.type());
        
        for(int i=0; i<src.rows; i++) {
            for(int j=0; j<src.cols; j++) {
                Vec3b pixel = src.at<Vec3b>(i, j);
                float b = pixel[0]/255.0f;
                float g = pixel[1]/255.0f;
                float r = pixel[2]/255.0f;

                float numerator = 0.5f * ((r-g) + (r-b));
                float denominator = sqrt((r-g)*(r-g) + (r-b)*(g-b)) + 1e-6f;
                float theta = acos(numerator / denominator);

                float H = (b <= g) ? theta : (2*CV_PI - theta);
                float sum = r + g + b;
                float S = (sum < 1e-6f) ? 0 : 1 - 3*std::min({r, g, b})/sum;
                float I = sum / 3.0f;

                hsi.at<Vec3b>(i, j) = Vec3b(
                    static_cast<uchar>(H * 255 / (2*CV_PI)),
                    static_cast<uchar>(S * 255),
                    static_cast<uchar>(I * 255)
                );
            }
        }
    }

public:
    // 图像回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            current_frame = cv_ptr->image.clone();
            new_frame = true;
        } catch (...) {
            ROS_WARN("图像接收异常");
        }
    }

    // 获取锥桶对中点偏移量（单位：像素）
    bool getMidOffset(float& offset) {
        if(!new_frame || current_frame.empty()) return false;
        
        cv::Mat resized_frame, hsi, mask;
        cv::resize(current_frame, resized_frame, cv::Size(), 0.5, 0.5);
        
        // 颜色空间转换
        BGR2HSI(resized_frame, hsi);
        
        // 颜色阈值处理
        cv::inRange(hsi, 
                  cv::Scalar(CONE_THRESH[0], CONE_THRESH[2], CONE_THRESH[4]),
                  cv::Scalar(CONE_THRESH[1], CONE_THRESH[3], CONE_THRESH[5]),
                  mask);

        // 形态学操作
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, 
                        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        cv::finrgc, argv, "vision_snake");
    ros::NodeHandle nh;
    
    // 初始化视觉模块
    ConeVision vision;
    ros::Subscriber img_sub = nh.subscribe("/camera/color/image_raw", 1, &ConeVision::imageCallback, &vision);
    
    // 运动控制
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist cmd;
    
    // 运动参数
    bool is_turning_right = true;
    int cycle_count = 0;
    ros::Time turn_start = ros::Time::now();

    while(ros::ok() && cycle_count < MAX_CYCLES) {
        // 基础蛇形节奏控制
        double elapsed = (ros::Time::now() - turn_start).toSec();
        if(elapsed > TURN_DURATION) {
            is_turning_right = !is_turning_right;
            turn_start = ros::Time::now();
            cycle_count++;
        }

        // 生成基础运动指令
        cmd.linear.x = BASE_SPEED;
        cmd.angular.z = is_turning_right ? -TURN_ANGLE : TURN_ANGLE;

        // 视觉修正
        float vision_offset = 0.0f;
        if(vision.getMidOffset(vision_offset)) {
            cmd.angular.z += 0 * vision_offset; 
            ROS_INFO_STREAM("视觉修正量: " << vision_offset);
        }

        vel_pub.publish(cmd);
        ros::spinOnce();
        ros::Rate(15).sleep();
    }

    // 安全停止
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    vel_pub.publish(cmd);
    
    return 0;
}
