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
    int x_axis;
    int y_axis;
    void set_xy(int x,int y){ x_axis = x; y_axis = y; }
    const char* m_source_frame;
    const char* m_child_frame;
    bool initialize(nav_msgs::OccupancyGrid source_map);
    void set_transform(const char* source_frame, const char* child_frame);
    static tf::StampedTransform get_transform(const char* source_frame, const char* child_frame);
    static void Get_local_map(nav_msgs::OccupancyGrid *local_map,nav_msgs::Odometry arg_pose);
    void Get_local_map(nav_msgs::OccupancyGrid *local_map);
    void set_pose(nav_msgs::Odometry arg_pose);
};

void localMap::set_transform(const char* source_frame, const char* child_frame)
{
    m_source_frame = source_frame;
    m_child_frame = child_frame;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        cout << source_frame << endl;
        cout << child_frame << endl;
        listener.lookupTransform(source_frame, child_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        cout << "here!\n";
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
    local_map->header.frame_id = m_child_frame;
    local_map->header.stamp = ros::Time::now();
    local_map->info.resolution = info.resolution;
    local_map->info.height = 2*x_axis;
    local_map->info.width = 2*y_axis;

    std::vector<signed char> temp_data;
    temp_data.assign(x_axis*2*y_axis*2,0);
    double current_x = transform.getOrigin().getX();
    double current_y = transform.getOrigin().getY();
    double current_th;
    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_th = yaw;
    double resolution = local_map->info.resolution;
    double cos_th = cos(yaw);
    double sin_th = sin(yaw);
    for(int i = 0;i < local_map->info.width;i++)
    {
        for(int j = 0;j<local_map->info.height;j++)
        {
            double robot_x = (i - 2*x_axis)*resolution;
            double robot_y = (j-y_axis)*resolution;
            double rot_x = cos_th*(robot_x) - sin_th*(robot_y);
            double rot_y = sin_th*(robot_x) + cos_th*(robot_y);
            double global_x = -(rot_x-current_x);
            double global_y = -(rot_y-current_y);
            unsigned int map_x = (global_x - info.origin.position.x)/resolution;
            unsigned int map_y = (global_y - info.origin.position.y)/resolution;
            int map_index = map_y*info.width + map_x;
            int local_index = local_map->info.width*local_map->info.height - (j*local_map->info.width + i) -1;
            unsigned int original_map_data;
            if(map_index >= info.height*info.width)
            {
                original_map_data = -1;
            }
            else
            {
                original_map_data = global_map.data.at(map_index);
                if(original_map_data > 90)
                {
//                            lane_vector_index.push_back(local_index);
                }
            }
            temp_data.at(local_index) = original_map_data;
//                    r.sleep();
        }
    }
    local_map->data = temp_data;
    // double current_x = 
    // nav_msgs::OccupancyGrid temp_local_map;
    //         temp_local_map.header.frame_id = "/novatel";
    //         temp_local_map.header.stamp = ros::Time::now();

    //         temp_local_map.info.height = 2*size_side;
    //         temp_local_map.info.width = size_front;
    //         temp_local_map.info.resolution = global_map.info.resolution;
    //         temp_local_map.info.origin.position.y = -(int)(size_side*global_map.info.resolution);

    //         std::vector<signed char> temp_data;
    //         temp_data.assign(size_side*2*size_front,0);
    // double current_x = currentPose->pose.pose.position.x;
    //         double current_y = currentPose->pose.pose.position.y;
    //         double current_th;
    //         tf::Quaternion q(
    //                 currentPose->pose.pose.orientation.x,
    //                 currentPose->pose.pose.orientation.y,
    //                 currentPose->pose.pose.orientation.z,
    //                 currentPose->pose.pose.orientation.w);
    //         tf::Matrix3x3 m(q);
    //         double roll, pitch, yaw;
    //         m.getRPY(roll, pitch, yaw);
    //         current_th = yaw;
    //         double resolution = temp_local_map.info.resolution;
    //         double cos_th = cos(yaw);
    //         double sin_th = sin(yaw);
    //         int pixel_count = 0;
    //         std::vector<int> lane_vector_index;
//     for(int i = 0;i < temp_local_map.info.width;i++)
//     {
//         for(int j = 0;j<temp_local_map.info.height;j++)
//         {
//             double robot_x = (i - size_front)*resolution;
//             double robot_y = (j-size_side)*resolution;
//             double rot_x = cos_th*(robot_x) - sin_th*(robot_y);
//             double rot_y = sin_th*(robot_x) + cos_th*(robot_y);
//             double global_x = -(rot_x-current_x);
//             double global_y = -(rot_y-current_y);
//             unsigned int map_x = (global_x - plain_map.info.origin.position.x)/resolution;
//             unsigned int map_y = (global_y - plain_map.info.origin.position.y)/resolution;
//             int map_index = map_y*plain_map.info.width + map_x;
//             int local_index = temp_local_map.info.width*temp_local_map.info.height - (j*temp_local_map.info.width + i) -1;
//             unsigned int original_map_data;
//             if(map_index >= plain_map.info.height*plain_map.info.width)
//             {
//                 original_map_data = -1;
//             }
//             else
//             {
//                 original_map_data = plain_map.data.at(map_index);
//                 if(original_map_data > 90)
//                 {
// //                            lane_vector_index.push_back(local_index);
//                 }
//             }
//             temp_data.at(local_index) = original_map_data;
//             pixel_count ++;
// //                    r.sleep();
//         }
//     }

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
        local_costmap.Get_local_map(&temp);//tf method
        local_costmap_pub.publish(temp);
        cout << "pub!"<<endl;
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
