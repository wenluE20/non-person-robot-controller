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

enum CameraState {
    COMPUTER = 0,
    ZED,
    REALSENSE
};
CameraState state = REALSENSE;

using namespace cv;
using namespace std;

// HSI转换函数
void BGR2HSI(const Mat &src, Mat &hsi) {
    if (src.rows != hsi.rows ||
        src.cols != hsi.cols ||
        src.channels() != 3 ||
        hsi.channels() != 3)
        return;

    float r, g, b, H, S, I, num, den, theta, sum, min_RGB;

    for(int i=0; i<src.rows; i++) {
        for(int j=0; j<src.cols; j++) {
            b = src.at<Vec3b>(i, j)[0];
            g = src.at<Vec3b>(i, j)[1];
            r = src.at<Vec3b>(i, j)[2];

            b /= 255.0; g /= 255.0; r /= 255.0;

            num = 0.5 * ((r-g)+(r-b));
            den = sqrt((r-g)*(r-g)+(r-b)*(g-b));
            theta = acos(num/(den + 1e-6));  // 防止除零

            if(b <= g) H = theta;
            else       H = 2*CV_PI - theta;

            min_RGB = min(min(b,g),r);
            sum = b+g+r;
            S = (sum < 1e-6) ? 0 : (1 - 3*min_RGB/sum);
            I = sum/3.0;

            hsi.at<Vec3b>(i, j)[0] = H*255/(2*CV_PI);
            hsi.at<Vec3b>(i, j)[1] = S*255;
            hsi.at<Vec3b>(i, j)[2] = I*255;
        }
    }
}

Mat frame_msg;
void rcvCameraCallBack(const sensor_msgs::Image::ConstPtr& img) {
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}

// 颜色分割函数
int ColorSplitAuto(const Mat &hsv_input, Mat &grey_output, const int threshold[6]) {
    int hmin = threshold[0], hmax = threshold[1];
    int smin = threshold[2], smax = threshold[3];
    int vmin = threshold[4], vmax = threshold[5];

    for (int i=0; i<hsv_input.rows; i++) {
        for (int j=0; j<hsv_input.cols; j++) {
            Vec3b pixel = hsv_input.at<Vec3b>(i, j);
            bool inRange = (pixel[0] >= hmin) && (pixel[0] <= hmax) &&
                           (pixel[1] >= smin) && (pixel[1] <= smax) &&
                           (pixel[2] >= vmin) && (pixel[2] <= vmax);
            grey_output.at<uchar>(i, j) = inRange ? 255 : 0;
        }
    }
    return countNonZero(grey_output);
}

// 锥桶颜色阈值（根据实际环境调整）
int cone_threshold[6] = {0, 10, 100, 255, 50, 255}; 

int main(int argc, char **argv) {
    ros::init(argc, argv, "exp3_8shape_navigation");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber camera_sub; 
    geometry_msgs::Twist vel_msg;

    // 正弦波路径参数
    double t = 0.0;
    const double FREQ = 0.15;    // 控制8字形状频率
    const double AMP = 0.8;       // 控制转向幅度
    const double LINEAR_SPEED = 0.4;

    VideoCapture capture;
    if(state == COMPUTER) {
        capture.open(0);     
        if (!capture.isOpened()) return 0;
        waitKey(1000);
    } else if(state == ZED) {
        capture.open(4);     
        if (!capture.isOpened()) return 0;
        waitKey(1000);
    } else if(state == REALSENSE) {
        camera_sub = n.subscribe("/camera/color/image_raw",1,rcvCameraCallBack);
    }

    Mat frIn;
    while (ros::ok()) {
        // 获取图像（与原代码相同）
        if(state == COMPUTER || state == ZED) {
            if(!capture.read(frIn)) continue;
            if(state == ZED) frIn = frIn(Rect(0,0,frIn.cols/2,frIn.rows));
        } else {
            if(frame_msg.empty()) {
                ros::spinOnce();
                continue;
            }
            frIn = frame_msg;
        }

        // 图像预处理
        resize(frIn, frIn, Size(), 0.25, 0.25);
        Mat hsi;
        BGR2HSI(frIn, hsi);

        // 锥桶检测
        Mat cone_mask(hsi.size(), CV_8UC1);
        ColorSplitAuto(hsi, cone_mask, cone_threshold);

        // 形态学处理
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(7,7));
        morphologyEx(cone_mask, cone_mask, MORPH_CLOSE, kernel);

        // 轮廓检测
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(cone_mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 筛选有效轮廓
        vector<Rect> cone_boxes;
        for (size_t i=0; i<contours.size(); i++) {
            if(contourArea(contours[i]) > 300)
                cone_boxes.push_back(boundingRect(contours[i]));
        }

        // 计算视觉修正量
        float correction = 0;
        const int img_center_x = frIn.cols/2;
        const int img_height = frIn.rows;
        int valid_cones = 0;

        for (const auto& box : cone_boxes) {
            Point center(box.x + box.width/2, box.y + box.height/2);
            // 只处理图像下半部分的锥桶
            if(center.y > img_height*0.6) { 
                float offset = center.x - img_center_x;
                correction += offset;
                valid_cones++;
                circle(frIn, center, 8, Scalar(0,255,255), 2);
            }
        }

        // 计算平均偏移量
        if(valid_cones > 0) {
            correction = -0.004 * (correction / valid_cones); // 比例系数调整
        }

        // 生成正弦波基准角速度
        double base_angular = AMP * sin(2*CV_PI*FREQ*t);
        t += 0.1; // 时间递增（10Hz循环）

        // 综合控制指令
        vel_msg.linear.x = LINEAR_SPEED;
        vel_msg.angular.z = base_angular + correction;

        // 角速度限幅
        vel_msg.angular.z = max(min(vel_msg.angular.z, 1.5), -1.5);

        // 可视化显示
        putText(frIn, "Base Ang: " + to_string(base_angular), Point(10,30), 
               FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200,0,0), 2);
        putText(frIn, "Final Ang: " + to_string(vel_msg.angular.z), Point(10,60), 
               FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,200), 2);
        imshow("Cone Detection", frIn);
        imshow("Binary Mask", cone_mask);
        waitKey(5);

        // 发布控制指令
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}