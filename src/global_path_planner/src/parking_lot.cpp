#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Quaternion.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <stdlib.h>
#include <signal.h>
///parameter
#define lookforward 200
#define lookside 50
#define lane_boundary 10 //pixel, 1 pixel = 0.1m^2
#define obsatcle_boundary 16
//#define lidar_gps_offset 1.35 //meter
//#define lidar_gps_offset 1.55 //meter - in sim
#define lidar_gps_offset 0.85 //meter - in sim
#define max_global_path 20 //meter
#define min_global_path 2 //meter
#define PI 3.14159265359
using namespace std;
//using namespace cv;
int pose_count = 0;
bool region_map_loaded = false;

bool parking_searched[6] = {false};
int parking_points[6] = {0};
std::vector<int> parking_queue;

nav_msgs::Odometry::ConstPtr currentPose;
pcl::PointCloud <pcl::PointXYZI> obstacle_pcl;
string state_string = "go";
unsigned int parking_lot[6] = {0};
bool parking_done = false;
int parking_num = -1;
#define occupied_threshold 300
#define parking_buffer 5

ros::Publisher parking_num_pub;
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pose_count++;
//    cout << "pose!\n";
    std::queue<nav_msgs::Odometry::ConstPtr> temp_q;
    temp_q.push(msg);
    currentPose = temp_q.front();
    temp_q.pop();
}

void obstacleCallback(const sensor_msgs::PointCloud2ConstPtr& scan)
{
    pcl::fromROSMsg(*scan,obstacle_pcl);
}
void stateCallback(const std_msgs::String::ConstPtr& msg)
{
    state_string = msg->data.c_str();
//    cout << "state!";
}

int main(int argc, char **argv){
    ros::init(argc, argv, "parking_lot");
    ros::NodeHandle n;
    ros::Subscriber traffic_light_sub = n.subscribe("/state", 1, stateCallback);
    ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom",10,poseCallback);
//    /gps_utm_odom
    ros::Subscriber obstacle_sub = n.subscribe("/Lidar/obj_pcl",10,obstacleCallback);
    parking_num_pub = n.advertise<std_msgs::Int16>("/parking_num",1);
    ros::Rate r(10);
    bool state_ok = true;
    string arg_str = "unknown";
    cout << argc;
    if(argc == 2)
    {
        arg_str = argv[1];
    }
    if(arg_str == "no_parking")
    {
        parking_done = true;
    }
    std::string file_path = ros::package::getPath("global_path_planner");
    for(int i = 0;i<parking_buffer;i++)
    {
        parking_queue.push_back(-1);
    }
//    cout << file_path;
    cv::Mat parking_lot_image = cv::imread(file_path+"/map_data/parking_lot.png",1);
    int frame_check = 0;
    while (ros::ok())
    {
//        cout << pose_count;
//        if(pose_count != 0 && !parking_done && state_string == "parking_search")
        if(pose_count != 0 && (state_string == "parking_search" || state_string == "parking"))
        {
            if(state_ok)
            {
                state_ok = false;
                cout << "algorithm works!\n";
            }
            double current_x = currentPose->pose.pose.position.x;
            double current_y = currentPose->pose.pose.position.y;
            double resolution = 0.1;
            double current_th;
//            cout << "kike";
            tf::Quaternion q(
                    currentPose->pose.pose.orientation.x,
                    currentPose->pose.pose.orientation.y,
                    currentPose->pose.pose.orientation.z,
                    currentPose->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_th = yaw;
            double cos_th = cos(yaw);
            double sin_th = sin(yaw);

            int region_map_x = (current_x + 182.4)/resolution;
            int region_map_y = (current_y + 626)/resolution;

            cv::Vec3b original_map_data;

//            cout << region_map_x << ", " << region_map_y << "\n";
//            cout <<  parking_lot_image.cols  << ", " << parking_lot_image.rows << "\n";
            bool parking_searched_final = false;
            if(region_map_x < parking_lot_image.cols && parking_lot_image.rows - region_map_y < parking_lot_image.rows && region_map_x>0 && parking_lot_image.rows - region_map_y>0)
            {
//                original_map_data = parking_lot_image.at<cv::Vec3b>(parking_lot_image.rows - region_map_y,region_map_x);
//                int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;

                for(int i = 0;i<obstacle_pcl.size();i++)
                {
                    double pcl_x = -(obstacle_pcl.at(i).x + lidar_gps_offset);
                    double pcl_y = -obstacle_pcl.at(i).y;
                    if(!(pcl_x >-2.0 && pcl_x<2.0 &&pcl_y > -0.5 &&pcl_y<0.5))
                    {
//                        pcl_x = 0.0;
//                        pcl_y = 10.0;
                        double rot_x = cos_th*(pcl_x) - sin_th*(pcl_y);
                        double rot_y = sin_th*(pcl_x) + cos_th*(pcl_y);
                        double global_x = -(rot_x-current_x);
                        double global_y = -(rot_y-current_y);
                        int map_x = (global_x + 182.4)/resolution;
                        int map_y = (global_y + 626)/resolution;
//                        cv::circle(parking_lot_image, cv::Point(map_x,parking_lot_image.rows - map_y),1, cv::Scalar(0,0,255),CV_FILLED, 8);
                        cv::Vec3b original_map_data;
                        cv::Vec3b original_pcl_data;
                        if(map_x < parking_lot_image.cols && parking_lot_image.rows - map_y < parking_lot_image.rows && map_x>0 && parking_lot_image.rows - map_y>0)
                        {
                            original_map_data = parking_lot_image.at<cv::Vec3b>(parking_lot_image.rows - map_y,map_x);
                        }



                        int point = (original_map_data[0] + original_map_data[1] + original_map_data[2]) / 3;
//                        cout << point << endl;
                        if(point != 255)
                        {
//                            cout << point << "\n";
                        }
                        if(region_map_x < parking_lot_image.cols && parking_lot_image.rows - region_map_y < parking_lot_image.rows && region_map_x>0 && parking_lot_image.rows - region_map_y>0)
                        {
    //                        original_pcl_data = parking_lot_image.at<cv::Vec3b>(map_y,map_x);

    //                        int point = (original_pcl_data[0] + original_pcl_data[1] + original_pcl_data[2]) / 3;
                            int index = 6 - point / 30;

                            if(index >= 0 && index <= 60)
                            {
                                parking_points[index]++;
                                if(index > 2)
                                {
                                    parking_searched[6 - index - 1] = true;
                                }

    //                            cout << "index : " << parking_points[index] << "\n";
                            }

                        }
                    }


                }
                for(int i = 5;i>=0;i--)
                {
                    cout  << parking_points[i] << "  ";
//                    if(parking_searched[i] && parking_points[i] < occupied_threshold)
//                    {
//                        parking_num = i + 1;
//                    }
                    if(parking_points[i] < occupied_threshold)
                    {
                        parking_num = i + 1;
                    }
                }
                cout << "parking_num : " <<parking_num<< "\n";
                std_msgs::Int16 msg;
                msg.data = parking_num;
                parking_num_pub.publish(msg);
//                int five_searched = 0;
//                for(int i = 0;i<4;i++)
//                {
//                    if(parking_points[i] > occupied_threshold)
//                    {
//                        five_searched++;
//                    }
//                }
//                if(five_searched == 5 && parking_num == -1)
//                {
//                    parking_num = 6;
//                }
//                parking_queue.erase(parking_queue.begin());
//                parking_queue.push_back(parking_num);
//                bool search_complete = true;
//                int back_num = parking_queue.at(0);
//                for(int i = 0;i<parking_buffer;i++)
//                {
//                    if(back_num != parking_queue.at(i))
//                    {
//                        search_complete = false;
//                    }
//                }
//                cout << "\n";

//                if(parking_num != -1 && search_complete)
//                {
//                    //publish
//                    bool is_close_first = true;
//                    for(int i = 0;i<parking_num;i++)
//                    {
//                        if(!parking_searched[i])
//                        {
//                            is_close_first = false;
//                        }
//                    }
//                    if(is_close_first)
//                    {
//                        parking_searched_final = true;

//                    }

//                }
            }
//            if(parking_searched_final)
//            {
//                cout << "parking_num : " <<parking_num<< "\n";
//                std_msgs::Int16 msg;
//                msg.data = parking_num;
//                parking_num_pub.publish(msg);
//            }

        }
        else {
//            cout << "pose call back failed!" << endl;
        }
        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

