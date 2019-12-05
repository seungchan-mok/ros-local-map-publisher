#include "local_map_include.h"

class localMap
{
private:
    nav_msgs::MapMetaData info;
    tf::StampedTransform transform;
public:
    localMap(){}
    localMap(nav_msgs::OccupancyGrid source_map)
    {
        this->info = source_map.info;
    }
    bool initialize(nav_msgs::OccupancyGrid source_map);
    void set_transform(const char* source_frame, const char* child_frame);
    static tf::StampedTransform get_transform(const char* source_frame, const char* child_frame);
    static void Get_local_map(nav_msgs::OccupancyGrid *local_map);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_costmap");
    ros::NodeHandle n;
    ros::Publisher local_costmap = n.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    //TODO: static map이 아닐경우 -> publish?
    // ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom", 10, poseCallback);
    // ros::Subscriber obstacle_sub = n.subscribe("/Lidar/obj_pcl", 10, obstacleCallback);
    ros::ServiceClient map_client1 = n.serviceClient<nav_msgs::GetMap>("/map/static_map");
    nav_msgs::GetMap srv;
    std::string file_path = ros::package::getPath("global_path_planner");
    ros::Rate r(10); //frequency
    bool state_ok = true;
    std::string source_frame;
    std::string child_frame;
    std::string map_name;
    n.getParam("/source_frame",source_frame);
    n.getParam("/child_frame",child_frame;
    n.getParam("/map_topic",map_name);
    while (ros::ok())
    {
        //TODO: 함수로 분리시키기 - tf 
        tf::StampedTransform transform;
        transform = localMap::get_transform("/map","/base_link");

        // Check map loaded - publish되는 map 일 경우?
        //code here
        //TODO: pose - tf에 상관없이 Update 방법?
        //TODO: yaml configuration file
        //
        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
