#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include "detect_lidar/LidarDetect.h"
#include <cv_bridge/cv_bridge.h>

class NewScan
{
    public:
        NewScan()
        {
            sub = nh.subscribe("scan", 1000, &NewScan::callback, this);
            pub = nh.advertise<sensor_msgs::LaserScan>("scan_new", 10);
        }
        void callback(const sensor_msgs::LaserScan::ConstPtr &scan)  
        {
            // Get basic information of scan
            unsigned int size = scan->ranges.size();
            std::vector<float> data_r = std::vector<float>(size, 0);
            float angle_now = scan->angle_min;
            // Init scan_new
            scan_new.angle_increment = scan->angle_increment;
            scan_new.header = scan->header;
            scan_new.scan_time = scan->scan_time;
            scan_new.time_increment = scan->time_increment;
            scan_new.range_max = scan->range_max;
            scan_new.range_min = scan->range_min;
            scan_new.ranges.resize(size);
            scan_new.intensities.resize(size);
            // Read raw data
            for(int i=0; i<size; ++i)
            {
                data_r[i] = angle_now;
                angle_now += scan->angle_increment;
            }
            // Change angle x to 2pi-x
            for(int i=0; i<size; ++i)
            {
                data_r[i] = 2*3.14159265 - data_r[i];
            }
            // Change angle range [0, 2pi] to [-pi, pi]
            for(int i=0; i<size; ++i)
            {
                data_r[i] = (data_r[i]>3.14159265)?(data_r[i]-2*3.14159265):data_r[i];
            }
            // Get index of the min element
            unsigned int index_min = min_element(data_r.begin(), data_r.end()) - data_r.begin();
            unsigned int index_max = max_element(data_r.begin(), data_r.end()) - data_r.begin();
            scan_new.angle_min = data_r[index_min];
            scan_new.angle_max = data_r[index_max];
            // Fill data of scan_new
            for(int i=0; i<=index_min; ++i)
            {
                scan_new.ranges[i] = scan->ranges[index_min-i];
                // scan_new.intensities[i] = scan->intensities[index_min-i];
            }
            for(int i=index_min+1; i<size; ++i)
            {
                scan_new.ranges[i] = scan->ranges[size+index_min-i];
                // scan_new.intensities[i] = scan->intensities[size+index_min-i];
            }
            pub.publish(scan_new);
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        sensor_msgs::LaserScan scan_new;
};

class PointFloat
{
    public:
        PointFloat():x(0.0), y(0.0) {}
        PointFloat(float x, float y):x(x), y(y) {}
        float x;
        float y;
};

class KMeans
{
    public:
        std::vector<PointFloat> means;  // results
        std::vector<float> var = std::vector<float>(7, INFINITY);
        unsigned int k;
        bool no_data = false;
        KMeans(std::vector<cv::Point> &data_input,int _loop_times):
            auto_k(true),
            _loop_times(_loop_times),
            data(data_input),
            data_size(data_input.size())
        {
            means_array = std::vector<std::vector<PointFloat>>{
                std::vector<PointFloat>(2),
                std::vector<PointFloat>(3),
                std::vector<PointFloat>(4),
                std::vector<PointFloat>(5),
                std::vector<PointFloat>(6),
                std::vector<PointFloat>(7),
                std::vector<PointFloat>(8),
            };
        }

        KMeans(std::vector<cv::Point> &data_input, unsigned int k_input,int _loop_times):
            k(k_input),
            _loop_times(_loop_times),
            auto_k(false),
            data(data_input),
            data_size(data_input.size())
        {
        }
        void clustering()
        {
            unsigned int temp;  // use to get random index
            unsigned int is_same = 0;  // use to check if means changed
            const float eps = 10;
            float variance = 0;
            unsigned int k_start = 0;
            unsigned int k_end = 0;
            std::vector<std::vector<float>> dist;
            // check whether it has enough data
            if(auto_k == true)
            {
                if(data_size <= 8)
                {
                    no_data = true;
                }
                else if(data_size > 8)
                {
                    no_data = false;
                    k_start = 2;
                    k_end = 8;
                }
            }
            else
            {
                k_start = k;
                k_end = k;
            }
            // ROS_INFO("Test : data_size = %d, k_start = %d, k_end = %d", data_size, k_start, k_end);
            if(no_data == false)
            {
                // Step 0: k from k_start to k_end loop!
                for(k=k_start; k<=k_end; ++k)
                {
                    // ROS_INFO("k:%d", k);
                    // generate some variables
                    result = std::vector<unsigned int>(data_size, 0);
                    means_temp = std::vector<PointFloat>(k);
                    sums = std::vector<cv::Point>(k);
                    core = std::vector<unsigned int>(k, 0);
                    nums = std::vector<unsigned int>(k, 0);
                    dist = std::vector<std::vector<float>>(data_size, std::vector<float>(k, 0));
                    // calc 5 times for 5 sets of random points, so that the result is more reliable
                    for(int loop_times=0; loop_times<_loop_times; ++loop_times)
                    {
                        variance = 0;
                        
                        // // Step 1: Random select k points in raw data
                        // for(int i=0; i<k; ++i)
                        // {
                        //     do 
                        //     {
                        //         temp = rand() % data_size;
                        //         // ROS_INFO("Step 1 : i cores: %d|%d|%d|%d", i, temp, data[temp].x, data[temp].y);
                        //     }while(find(core.begin(), core.end(), temp) != core.end());
                        //     core[i] = temp;
                        // }

                        // KMeans++ :
                        // Step 1.1: random select one
                        core[0] = rand() % data_size;
                        unsigned int core_index;
                        std::vector<float> core_dist;
                        // Step 1.4: loop
                        for(core_index = 1; core_index<k; ++core_index)
                        {
                            core_dist = std::vector<float>(data_size, INFINITY);
                            // Step 1.2: calculate distance between each point and the centers, and record the shortest distance
                            float dist_temp = 0;
                            for(int i=0; i<data_size; i++)
                            {
                                for(int j=0; j<core_index; ++j)
                                {
                                    dist_temp = powf32(data[i].x-data[core[j]].x, 2) + powf32(data[i].y-data[core[j]].y, 2);
                                    core_dist[i] = (dist_temp<core_dist[i])?dist_temp:core_dist[i];
                                }
                            }
                            
                            // Step 1.3: choose the farest as the next
                            core[core_index] = max_element(core_dist.begin(), core_dist.end()) - core_dist.begin();
                        }
                        // Step 2: Generate first set of means
                        for(int i=0; i<k; ++i)
                        {
                            means_temp[i].x = (float)data[core[i]].x;
                            means_temp[i].y = (float)data[core[i]].y;
                            // ROS_INFO("Step 1 : i means:%d|%f|%f", i, means_temp[i].x, means_temp[i].y);
                        }
                        // Step 6: Loop until means do not change
                        do
                        {
                            is_same = 0;
                            // Step 3: Calculate the distance betwenn each point and each mean
                            for(int i=0; i<data_size; ++i)
                            {
                                for(int j=0; j<k; ++j)
                                {
                                    dist[i][j] = distance(data[i].x, data[i].y, means_temp[j].x, means_temp[j].y);
                                }
                            }
                            // Step 4: Assign each point to nearest mean;
                            for(int i=0; i<k; ++i)  // clear nums and sums
                            {
                                nums[i] = 0;
                                sums[i].x = 0;
                                sums[i].y = 0;
                            }
                            for(int i=0; i<data_size; ++i)
                            {
                                result[i] = min_element(dist[i].begin(), dist[i].end()) - dist[i].begin();
                                nums[result[i]]++;
                                // ROS_INFO("Step4 : %d, %f", result[i], *min_element(dist[i].begin(), dist[i].end()));
                            }
                            // Step 5: Calculate new means
                            for(int i=0; i<data_size; ++i)
                            {
                                sums[result[i]].x += data[i].x;
                                sums[result[i]].y += data[i].y;
                            }
                            for(int i=0; i<k; ++i)
                            {
                                if(nums[i] != 0)
                                {
                                    if(fabs(means_temp[i].x-(float)sums[i].x/nums[i])<eps && fabs(means_temp[i].y-(float)sums[i].y/nums[i])<eps)
                                    {
                                        is_same++;
                                    }
                                    means_temp[i].x = (float)sums[i].x/nums[i];
                                    means_temp[i].y = (float)sums[i].y/nums[i];
                                }
                                else
                                {
                                    is_same++;
                                    means_temp[i].x = 300.0;
                                    means_temp[i].y = 300.0;
                                    ROS_INFO("k error");
                                }
                                // ROS_INFO("Step 5: index:%d, size:%d, nums:%d", i, data_size, nums[i]);
                            }
                        }while(is_same!=k);
                        // Step 7: Calculate variance
                        for(int i=0; i<data_size; ++i)
                        {
                            variance += *min_element(dist[i].begin(), dist[i].end());
                            // ROS_INFO("Step 7 : var=%f", variance);
                        }
                        // Step 8: Compare with best solution
                        if(variance < var[k-k_start])
                        {
                            // ROS_INFO("Step 8 : %d", k-k_start);
                            if(auto_k == true)
                            {
                                means_array[k-k_start] = means_temp;
                            }
                            else
                            {
                                means = means_temp;
                            }
                            var[k-k_start] = variance;
                            // ROS_INFO("Step 8 End!");
                        }
                    }  // calc 5 times
                }  // k loop
                if(auto_k == true)
                {
                    // print variances
                    for(k=k_start; k<=k_end; ++k)
                    {
                        // ROS_INFO("Test : var=%f", var[k-k_start]);
                    }
                    // d(var)/d(k)
                    std::vector<float> var_d1 = std::vector<float>(6, INFINITY);
                    const float var_trd = 5000;  // trd in var_trd means threashold
                    for(k=k_start; k<k_end; ++k)  // notice: it's < here, not <=
                    {
                        var_d1[k-2] = var[k-1] - var[k-2];
                        // ROS_INFO("Test : var_d1=%f", var_d1[k-2]);
                    }
                    for(k=k_start; k<k_end; ++k)
                    {
                        if(fabs(var_d1[k-2]) < var_trd)
                        {
                            break;
                        }
                    }
                    means = means_array[k-k_start];
                    // ROS_INFO("Final : k=%d", k);  // show result
                    means.resize(k);
                }
            }
        }
    private:
        bool auto_k = false;
        int _loop_times=5;
        std::vector<cv::Point> data;
        unsigned int data_size;
        std::vector<cv::Point> sums;        // use for calc new means
        std::vector<unsigned int> nums;     // show each means have how much nums
        std::vector<unsigned int> result;   // means'(sums') index
        std::vector<unsigned int> core;     // data's index(just a temp variable)
        std::vector<PointFloat> means_temp; // for temprarily place means
        std::vector<std::vector<PointFloat>> means_array; // for temprarily place means
        float distance(float x1, float y1, float x2, float y2)
        {
            return powf32((x1-x2), 2) + powf32((y1-y2), 2);
        }
};

class LaserHandler
{
    public:
        void data_handler(const sensor_msgs::LaserScan::ConstPtr &data)
        {
            /***** Data Preprocess *****/
            // Get rid of INFINITES in data->ranges (convert them to data->range_max)
            unsigned int data_size = (unsigned int)data->ranges.size();
            std::vector<float> data_r(data_size, 0);  // r in data_r means raw
            // std::vector<float> data_r_i(data_size, 0);  // i in data_i means inverse
            for(int i=0; i<data_size; ++i)
            {
                if(data->ranges[i] == INFINITY || data->ranges[i] > data->range_max)
                {
                    data_r[i] = data->range_max;
                }
                else
                {
                    data_r[i] = data->ranges[i];
                }
            }
	    /*
            // Inverse
            unsigned int inverse_count;
            for(inverse_count=0; inverse_count<data_size; ++inverse_count)
            {
                data_r_i[inverse_count] = data_r[data_size - inverse_count -1];
            }
	    */

            // Get wanted points
            unsigned int data_w_size = 0;
            unsigned int data_nr_size = 0;
            std::vector<cv::Point> data_w(data_size);  // w in data_w means wanted, the vector will be resized when all processes are over
            std::vector<cv::Point> data_nr(data_size);  // np in data_nr means non-repeat
            float w_angle_min = n.param("/detect_lidar/Kmeans_Angle_Min",-120);  // w in w_angle_min means wanted
            float w_angle_max = n.param("/detect_lidar/Kmeans_Angle_Max",120);
            w_angle_min=w_angle_min/180*3.1415926535;
            w_angle_max=w_angle_max/180*3.1415926535;
            float proportion = n.param("/detect_lidar/Proportion_XY",0.5);
            float w_range_max = 1/proportion;
            
            double now_angle = data->angle_min - data->angle_increment;
	        // ROS_INFO("now_angle = %f, min = %f, incr = %f", now_angle, data->angle_min, data->angle_increment);
            // now_angle = data->angle_min - data->angle_increment;
            /*map parameters*/
            int mymap_x = n.param("/detect_lidar/Map_Size",600);
            int mymap_y = mymap_x;
            // std::cout<<mymap_x<<","<<mymap_y<<std::endl;
            // int mymap_x =600;
            // int mymap_y =600;
            /*--------------*/
            for(int i=0; i<data_size; ++i)
            {
                now_angle += data->angle_increment;
                // check if the point is front of ridar
                if(now_angle>=w_angle_min && now_angle<=w_angle_max)
                {
                    // check if the range is wanted
                    if(data_r[i]<w_range_max && data_r[i] != 0)
                    {
                        data_w[data_w_size].x = round(mymap_x/2*(1+data_r[i]*sin(now_angle)/w_range_max));  // it should be + (the ridar's fault)
                        data_w[data_w_size].y = round(mymap_y/2*(1-data_r[i]*cos(now_angle)/w_range_max));
                        data_w_size++;
                    }
                }
            }
            // get rid of those repeat points
            if(data_w_size != 0)
            {
                data_nr[0] = data_w[0];
                data_nr_size++;
                for(int i=0; i<data_w_size; ++i)
                {
                    for(int j=0; j<data_nr_size; ++j)
                    {
                        if(data_w[i].x == data_nr[j].x && data_w[i].y == data_nr[j].y)
                        {
                            break;
                        }
                        if(j == data_nr_size-1)
                        {
                            data_nr[data_nr_size].x = data_w[i].x;
                            data_nr[data_nr_size].y = data_w[i].y;
                            data_nr_size++;
                        }
                    } 
                }
            }
            data_nr.resize(data_nr_size);
            /***** Data Process *****/
            KMeans centers(data_nr,_loop_times);
            centers.clustering();
            
            /***** Draw Map *****/
            // Draw lines
            cv::Mat mymap = cv::Mat::zeros(mymap_x, mymap_y, CV_8UC3);
            cv::Point map_origin(mymap_x/2, mymap_y/2);
            cv::Point map_border_l(round(mymap_x/2*(1+w_range_max*sin(w_angle_min)/w_range_max)),\
                                    round(mymap_y/2*(1-w_range_max*cos(w_angle_min)/w_range_max)));
            cv::Point map_border_r(round(mymap_x/2*(1+w_range_max*sin(w_angle_max)/w_range_max)),\
                                    round(mymap_y/2*(1-w_range_max*cos(w_angle_max)/w_range_max)));
            line(mymap, cv::Point(0, mymap_y/2), cv::Point(mymap_x, mymap_y/2), cv::Scalar(255, 255, 255));
            line(mymap, cv::Point(mymap_x/2, 0), cv::Point(mymap_x/2, mymap_y), cv::Scalar(255, 255, 255));
            line(mymap, map_origin, map_border_l, cv::Scalar(0, 255, 255));
            line(mymap, map_origin, map_border_r, cv::Scalar(0, 255, 255));
            cv::circle(mymap,cv::Point(mymap_x/2,mymap_y/2),mymap_x/2, cv::Scalar(0, 255, 255));
            // Draw points
            for(int i=0; i<data_nr_size; i++)
            {
                circle(mymap, data_nr[i], 0, cv::Scalar(255, 255, 255), 1);
            }
            // Draw centers
            if(centers.no_data == false)
            {
                for(int i=0; i<centers.means.size(); i++)
                {
                    circle(mymap, cv::Point(centers.means[i].x, centers.means[i].y), 5, cv::Scalar(0, 0, 0xFF), 2);
                }
            }
            else
            {
                ROS_INFO("No Data!!!");
            }
            // Show image
            imshow("Cones Map", mymap);
            sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mymap).toImageMsg();
            rgb_pub.publish(msg_rgb);

            /***** Data Output *****/
            // variables
            if(centers.no_data == false)
            {
                // ROS_INFO("------");
                std::vector<PointFloat> points_temp(centers.k, PointFloat(0,0));
                std::vector<unsigned int> pos_index(centers.k, 0);
                std::vector<float> length(centers.k, 0);
                unsigned int index_temp = 0;
                float length_temp = 0;
                // calculate distance
                for(int i=0; i<centers.k; ++i)
                {
                    points_temp[i].x = (centers.means[i].x - (mymap_x/2))/(mymap_x/2) * w_range_max;
                    points_temp[i].y = (1 - centers.means[i].y/(mymap_y/2)) * w_range_max;
                    length[i] = powf32(points_temp[i].x, 2) + powf32(points_temp[i].y, 2);
                    pos_index[i] = i;
                }
                // sort
                for(int i=0; i<centers.k-1; ++i)
                {
                    for(int j=i+1; j<centers.k; ++j)
                    {
                        if(length[i] > length[j])
                        {
                            length_temp = length[j];
                            length[j] = length[i];
                            length[i] = length_temp;
                            index_temp = pos_index[j];
                            pos_index[j] = pos_index[i];
                            pos_index[i] = index_temp;
                        }
                    }
                }
                // generate result
                float result_x[8];
                float result_y[8];
                unsigned int result_index = 0;
                for(int i=0; i<centers.k; ++i)
                {
                    if(length[i] > 0.01)  // check if length is zero
                    {
                        result_x[i] = points_temp[pos_index[i]].x;
                        result_y[i] = points_temp[pos_index[i]].y;
                        result_index++;
                    }
                }
                for(result_index; result_index<8; ++result_index)
                {
                    result_x[result_index] = 0;
                    result_y[result_index] = 0;
                }
                // screen output
                /*
                ROS_INFO("---Output---");
                for(int i=0; i<4; i++)
                {
                    ROS_INFO("[Point%d:(%f, %f)]", result_x[i], result_y[i]);
                }
                */
                
                // fill pub
                pub_msg.header.stamp = ros::Time::now();
                pub_msg.header.frame_id = "lidar_link";
                for(int i=0; i<8; ++i)
                {
                    pub_msg.x[i] = result_x[i];
                    pub_msg.y[i] = result_y[i];
                    pub_msg.z[i] = 1;
                    // ROS_INFO("%d: (%5.3f, %5.3f), (%f)", i, result_x[i], result_y[i], sqrt(powf32(result_x[i], 2)+powf32(result_y[i], 2)));
                }
                // publish result
                pub.publish(pub_msg);
            }
            
        }
        LaserHandler()
        {
            sub = n.subscribe("scan_new", 1000, &LaserHandler::callback, this);
            pub = n.advertise<detect_lidar::LidarDetect>("cones", 10);
            image_transport::ImageTransport it(n);
            rgb_pub = it.advertise("rgb", 15);
            _loop_times=n.param("/detect_lidar/Loop_Times",5);
        }
        void callback(const sensor_msgs::LaserScan::ConstPtr &msg)  
        {
            // subscribe laser's data
            unsigned int size = (unsigned int)msg->ranges.size();
            // process laser's data
            // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            data_handler(msg);

            // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
            // ROS_INFO_STREAM("Time use: " << std::setprecision(4) << time_used.count() << " ms");
        }
    private: 
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
        detect_lidar::LidarDetect pub_msg;
        image_transport::Publisher rgb_pub;
        int _loop_times=5;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_lidar");
    ros::NodeHandle node;
    LaserHandler laser_handler;
    NewScan new_scan;
    // ROS_INFO("Start!");
    while(node.ok())
    {
        ros::spinOnce();
        cv::waitKey(5);
    }
    return 0;
}