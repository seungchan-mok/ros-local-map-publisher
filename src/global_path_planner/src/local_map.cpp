#include "local_map_include.h"
using namespace std;
nav_msgs::OccupancyGrid global_map;
nav_msgs::Odometry pose;
class localMap
{
private:
    nav_msgs::MapMetaData info;
    nav_msgs::Odometry pose;
public:
    localMap(){}
    localMap(nav_msgs::OccupancyGrid source_map)
    {
        this->info = source_map.info;
    }
    tf::StampedTransform transform;
    bool initialize(nav_msgs::OccupancyGrid source_map);
    void set_transform(const char* source_frame, const char* child_frame);
    static tf::StampedTransform get_transform(const char* source_frame, const char* child_frame);
    static void Get_local_map(nav_msgs::OccupancyGrid *local_map,nav_msgs::Odometry arg_pose);
    void Get_local_map(nav_msgs::OccupancyGrid *local_map);
    void set_pose(nav_msgs::Odometry arg_pose);
};

void localMap::set_transform(const char* source_frame, const char* child_frame)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(source_frame, child_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    this->transform = transform;
}

tf::StampedTransform localMap::get_transform(const char* source_frame, const char* child_frame)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.lookupTransform(source_frame, child_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    return transform;
}

void localMap::set_pose(nav_msgs::Odometry arg_pose)
{
    this->pose = arg_pose;
}
/*
odom method
Get_local_map(nav_msgs::OccupancyGrid *local_map, nav_msgs::Odometry arg_pose)
*/
void localMap::Get_local_map(nav_msgs::OccupancyGrid *local_map, nav_msgs::Odometry arg_pose)
{

}
/*
tf method
Get_local_map(nav_msgs::OccupancyGrid *local_map)
*/
void localMap::Get_local_map(nav_msgs::OccupancyGrid *local_map)
{

}


void mapCallback(const nav_msgs::OccupancyGrid msg)
{
    global_map = msg;
}

void odomCallback(const nav_msgs::Odometry msg)
{
    pose = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_costmap");
    ros::NodeHandle n;
    ros::Publisher local_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    // set parameter
    std::string source_frame;
    std::string child_frame;
    std::string map_name;
    bool is_static_map;
    std::string method;
    std::string odom_topic;
    n.getParam("/source_frame",source_frame);
    n.getParam("/child_frame",child_frame);
    n.getParam("/map_topic",map_name);
    n.getParam("global_map_static",is_static_map);
    n.getParam("method",method);
    n.getParam("odom_topic",odom_topic);
    //end set parameter

    //TODO: static map이 아닐경우 -> publish?
    // ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom", 10, poseCallback);
    // ros::Subscriber obstacle_sub = n.subscribe("/Lidar/obj_pcl", 10, obstacleCallback);
    bool map_loaded = false;
    if(is_static_map)
    {
        ros::ServiceClient map_client1 = n.serviceClient<nav_msgs::GetMap>("/map/static_map");
        nav_msgs::GetMap srv;
        if (map_client1.call(srv))
        {
            global_map = srv.response.map;
            global_map.data = srv.response.map.data;
            cout << global_map.info.width << " x " << global_map.info.height << endl;
            map_loaded = true;
        }
    }
    else
    {
        ros::Subscriber global_map_sub = n.subscribe(map_name, 10, mapCallback);
    }
    if(method == "odom")
    {
        ros::Subscriber pose_sub = n.subscribe(odom_topic, 10, odomCallback);
    }
    
    std::string file_path = ros::package::getPath("global_path_planner");
    ros::Rate r(10); //frequency
    bool state_ok = true;
    
    while (ros::ok())
    {
        //TODO: 함수로 분리시키기 - tf 
        tf::StampedTransform transform;
        //transform = localMap::get_transform("/map","/base_link");
        localMap local_costmap;
        local_costmap.set_transform(source_frame.c_str(),child_frame.c_str());
        nav_msgs::OccupancyGrid temp;
        local_costmap.Get_local_map(&temp);
        local_costmap_pub.publish(temp);
        //TODO: get local map - tf

        //TODO: get local map - odom
        //TODO: odom 2 transform set

        //TODO: local cost map
        //TODO: make sample bag - all method

        //TODO: pub local map
        // Check map loaded - publish되는 map 일 경우?
        //code here
        //
        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
