#include "local_map_include.h"
class localMap
{
public:
    bool initialize(){}
};
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
    while (ros::ok())
    {
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
