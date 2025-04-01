#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#include <math.h>
#include <cv_bridge/cv_bridge.h>

enum CameraState
{
    COMPUTER = 0,
    ZED,
    REALSENSE
};
CameraState state = REALSENSE;

using namespace cv;
using namespace std;
ros::Publisher vel_pub;

void BGR2HSI(const Mat &src, Mat &hsi)
{
    if (src.rows != hsi.rows ||
        src.cols != hsi.cols ||
        src.channels() != 3 ||
        hsi.channels() != 3)
        return;
        /*** RGB转HSI ***/

  float r, g, b, H, S, I, num, den, theta, sum, min_RGB;

  for(int i=0; i<src.rows; i++)
    {
      for(int j=0; j<src.cols; j++)
        {
          b = src.at<Vec3b>(i, j)[0];
          g = src.at<Vec3b>(i, j)[1];
          r = src.at<Vec3b>(i, j)[2];

          // 归一化
          b = b/255.0;
          g = g/255.0;
          r = r/255.0;

          num = 0.5 * ((r-g)+(r-b));
          den = sqrt((r-g)*(r-g)+(r-b)*(g-b));
          theta = acos(num/den);

          if(den == 0){
                H = 0; 
          }
          else{
                if(b <= g){
                        H = theta;
                }
                else{
                        H = (2*3.14169265 - theta);
                }
                }

          min_RGB = min(min(b,g),r); // min(R,G,B)
          sum = b+g+r;
          if(sum == 0)
            {
                S = 0;
            }else{
                S = 1 - 3*min_RGB/sum;
            }

          I = sum/3.0;
          H = H/(2*3.14159265);
          hsi.at<Vec3b>(i, j)[0] = H*255;
          hsi.at<Vec3b>(i, j)[1] = S*255;
          hsi.at<Vec3b>(i, j)[2] = I*255;
        }
        }
}

Mat frame_msg;
void rcvCameraCallBack(const sensor_msgs::Image::ConstPtr& img)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}
/*** 一、在此处填充阈值分割代码 ***/
int ColorSplitAuto(const Mat &hsi_input, Mat &grey_output, int color_thresh[6])
{
    if (hsi_input.empty() || grey_output.empty())
	return 0;
    int count = 0;
    int h_min = color_thresh[0];
    int h_max = color_thresh[1];
    int s_min = color_thresh[2];
    int s_max = color_thresh[3];
    int i_min = color_thresh[4];
    int i_max = color_thresh[5];

    // 初始化输出图像为全黑（0）
    grey_output = Mat::zeros(hsi_input.size(), CV_8UC1);

    // 遍历图像的每个像素
    for (int y = 0; y < hsi_input.rows; y++)
    {
        for (int x = 0; x < hsi_input.cols; x++)
        {
            int h = hsi_input.at<Vec3b>(y, x)[0];
            int s = hsi_input.at<Vec3b>(y, x)[1];
            int i = hsi_input.at<Vec3b>(y, x)[2];

            // 检查像素值是否在指定的HSI范围内
            bool in_range;
            if (h_min <= h_max)
            {
                in_range = (h >= h_min && h <= h_max);
            }
            else  // 处理H值环绕的情况（例如红色：350-10度）
            {
                in_range = (h >= h_min || h <= h_max);
            }

            in_range = in_range && (s >= s_min && s <= s_max) && (i >= i_min && i <= i_max);

            // 如果在范围内，将输出图像相应像素设为255（白色），并增加计数
            if (in_range)
            {
                grey_output.at<uchar>(y, x) = 255;
                count++;
            }
        }
    }
    return count;
}
/*** 二、在此处定义四种颜色的H、S、I阈值 ***/

// 红色HSI阈值 (H接近0或接近1)
int red_thresh[6] = {170, 10, 70, 255, 60, 255};  // H_min, H_max, S_min, S_max, I_min, I_max

// 绿色HSI阈值 (H约为120度，归一化后约为1/3)
int green_thresh[6] = {70, 100, 70, 255, 60, 255};

// 蓝色HSI阈值 (H约为240度，归一化后约为2/3)
int blue_thresh[6] = {150, 190, 70, 255, 60, 255};

// 黄色HSI阈值 (H约为60度，归一化后约为1/6)
int yellow_thresh[6] = {30, 50, 70, 255, 60, 255};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exp2_2"); // 初始化 ROS 节点
    ros::NodeHandle n;
    ros::Rate loop_rate(10); 
/*** 三、在此处声明机器人的速度话题 ***/
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber camera_sub;
    VideoCapture capture;
    if(state == COMPUTER)
    {
        capture.open(0);     
        if (!capture.isOpened())
        {
            printf("电脑摄像头没有正常打开\n");
            return 0;
        }
        waitKey(1000);
    }
    else if(state == ZED)
    {
        capture.open(4);     
        if (!capture.isOpened())
        {
            printf("ZED摄像头没有正常打开\n");
            return 0;
        }
        waitKey(1000);
    }
    else if(state == REALSENSE)
    {
        camera_sub = n.subscribe("/camera/color/image_raw",1,rcvCameraCallBack);
    }
    

    Mat frIn;
    while (ros::ok())
    {
        if(state == COMPUTER)
        {
            capture.read(frIn);
            if (frIn.empty())
            {
                printf("没有获取到电脑图像\n");
                continue;
            }
        }
        else if(state == ZED)
        {
            capture.read(frIn);
            if (frIn.empty())
            {
                printf("没有获取到ZED图像\n");
                continue;
            }
            frIn = frIn(cv::Rect(0,0,frIn.cols/2,frIn.rows));//截取zed的左目图片
        }
        else if(state == REALSENSE)
        {
            if(frame_msg.cols == 0)
            {
                printf("没有获取到realsense图像\n");
                ros::spinOnce();
                continue;
            }
            frIn = frame_msg;
        }

        float factor = 0.25;
        resize(frIn, frIn, Size((int) (factor * frIn.cols), (int) (factor * frIn.rows)));
        Mat hsi(frIn.size(), CV_8UC3);
        BGR2HSI(frIn, hsi);
        imshow("hsi",hsi);

/*** 四、在此处填充颜色识别与机器人控制代码 ***/
        Mat red_mask(frIn.size(), CV_8UC1);
        Mat green_mask(frIn.size(), CV_8UC1);
        Mat blue_mask(frIn.size(), CV_8UC1);
        Mat yellow_mask(frIn.size(), CV_8UC1);
        
        // 对四种颜色进行阈值分割
        int red_count = ColorSplitAuto(hsi, red_mask, red_thresh);
        int green_count = ColorSplitAuto(hsi, green_mask, green_thresh);
        int blue_count = ColorSplitAuto(hsi, blue_mask, blue_thresh);
        int yellow_count = ColorSplitAuto(hsi, yellow_mask, yellow_thresh);
        
        // 显示分割结果
        imshow("Red Mask", red_mask);
        imshow("Green Mask", green_mask);
        imshow("Blue Mask", blue_mask);
        imshow("Yellow Mask", yellow_mask);
        
        // 创建速度消息
        geometry_msgs::Twist vel_msg;
        
        // 设置阈值来决定是否检测到足够多的颜色像素点
        int pixel_threshold = 500;  // 可以根据实际情况调整
        
        // 默认速度为0
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 0;
        
        // 找出像素点最多的颜色
        int max_count = max(max(red_count, green_count), max(blue_count, yellow_count));
        
        if (max_count > pixel_threshold) {
            if (max_count == red_count) {
                // 红色：前进
                vel_msg.linear.x = 0.2;  // 正值表示前进
                printf("检测到红色，机器人前进\n");
            } else if (max_count == green_count) {
                // 绿色：后退
                vel_msg.linear.x = -0.2;  // 负值表示后退
                printf("检测到绿色，机器人后退\n");
            } else if (max_count == blue_count) {
                // 蓝色：左转
                vel_msg.angular.z = 0.5;  // 正值表示左转
                printf("检测到蓝色，机器人左转\n");
            } else if (max_count == yellow_count) {
                // 黄色：右转
                vel_msg.angular.z = -0.5;  // 负值表示右转
                printf("检测到黄色，机器人右转\n");
            }
        } else {
            printf("未检测到明确的颜色，机器人停止\n");
        }
        
        // 发布速度命令
        vel_pub.publish(vel_msg);


        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
