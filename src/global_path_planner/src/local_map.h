#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
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
///parameter

#define bigger_frame 6
#define lookforward 200
#define lookside 60
int obsatcle_boundary = 15;
int front_obsatcle_boundary = 30;
#define occupancy_weight 20
//#define lidar_gps_offset 1.45 //meter
#define lidar_gps_offset 0.85 //meter
int max_global_path = 20; //meter
//string previous =
#define min_global_path 2 //meter
#define PI 3.14159265359
using namespace std;
//using namespace cv;
int erase_path = 15;
int erase_path_obs = 5;
int lane_boundary = 7; //pixel, 1 pixel = 0.1m^2
int side_min_boundary = 8;
#define side_max_boundary 20
int front_min_boundary = 13;
#define front_max_boundary 40
double between_dist = 3.5;
int pose_count = 0;
bool map_loaded = false;
bool plain_map_loaded = false;
bool region_map_loaded = false;
bool global_loaded = false;
bool avoid_loaded = false;
string state_string = "go";
string trffic_state_string = "go";
const string state_table[6] = {"go","stop","cross_road","wait_for_traffic_light","static_obs","outbreak_obs"};
string gps_state = "narrow_int";
ros::Publisher local_path_pub;
ros::Publisher original_path_pub;
ros::Publisher local_costmap;
ros::Publisher local_obstacle_state;
ros::Publisher start_point_pub;
ros::Publisher goal_point_pub;
nav_msgs::Path previous_path;
int pre_boundary = 0;
int pre_boundary_f = 0;
int over_path_count = 0;
nav_msgs::Path::ConstPtr global_path;
nav_msgs::Path::ConstPtr avoid_path;
nav_msgs::Odometry::ConstPtr currentPose;
nav_msgs::OccupancyGrid global_map;
nav_msgs::OccupancyGrid plain_map;
nav_msgs::OccupancyGrid region_map;
std::queue<nav_msgs::OccupancyGrid> map_queue;
std::queue<nav_msgs::OccupancyGrid::ConstPtr> path_queue;
pcl::PointCloud <pcl::PointXYZI> obstacle_pcl;
cv::Mat previous_obs_image;
//pixel
int size_front = lookforward;
int size_side = lookside;
int global_path_index = 0;
const int dx[8] = {-1,-1,-1,0,0,1,1,1};
const int dy[8] = {-1,0,1,-1,1,-1,0,1};
const int h_func[8] = {14,10,14,10,10,14,10,14};
bool shortest_path_searched = false;
int path_index_end = 0;
int path_index_start = 0;
string gps_stable = "stable";
int under_path = 0;