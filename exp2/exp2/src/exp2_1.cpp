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
                H = 0; // 分母不能为0
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

          // 将S分量和H分量都扩充到[0,255]区间以便于显示;一般H分量在[0,2pi]之间，S在[0,1]之间
          hsi.at<Vec3b>(i, j)[0] = H*255;
          hsi.at<Vec3b>(i, j)[1] = S*255;
          hsi.at<Vec3b>(i, j)[2] = I*255;
        }
        

        }
    
}

void ColorSplitManual(const Mat &hsv_input, Mat &grey_output, const string window)
{
	static int hmin = 0;
	static int hmax = 360;
	static int smin = 0;
	static int smax = 255;
	static int vmin = 0;
	static int vmax = 255;
	createTrackbar("Hmin", window, &hmin, 255);
	createTrackbar("Hmax", window, &hmax, 255);
	createTrackbar("Smin", window, &smin, 255);
	createTrackbar("Smax", window, &smax, 255);
	createTrackbar("Vmin", window, &vmin, 255);
	createTrackbar("Vmax", window, &vmax, 255);

    /*** 阈值分割 ***/
	int rw = hsv_input.rows;
	int cl = hsv_input.cols;
	for(int i = 0; i < rw; i++)
	{
		for(int j = 0; j < cl; j++)
		{
			if(hmin <= hsv_input.at<Vec3b>(i, j)[0] && hsv_input.at<Vec3b>(i, j)[0] <= hmax &&
			   smin <= hsv_input.at<Vec3b>(i, j)[1] && hsv_input.at<Vec3b>(i, j)[1] <= smax &&
			   vmin <= hsv_input.at<Vec3b>(i, j)[2] && hsv_input.at<Vec3b>(i, j)[2] <= vmax)
			   {
			   	grey_output.at<uchar>(i, j) = 255;
			   }
				
			else grey_output.at<uchar>(i, j) = 0;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exp2_1"); // 初始化 ROS 节点
    ros::NodeHandle n;
    ros::Rate loop_rate(10); 
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
        
        // RGB转HSI
        Mat hsi(frIn.size(), CV_8UC3);
        BGR2HSI(frIn, hsi);
        imshow("hsi",hsi);

        // 手动颜色分割
        Mat grey(frIn.rows, frIn.cols, CV_8UC1);
        ColorSplitManual(hsi, grey, "hsi");
        imshow("split", grey);

        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
