#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import tf
from math import cos,sin
import sensor_msgs.point_cloud2

#local_costmap_pub = rospy.Publisher('local_costmap', OccupancyGrid,queue_size=1)
step = 50
step_count = 0
obj_pcl = PointCloud2()
cur_x = 0.0
cur_y = 0.0
cur_th = 0.0
pcl_x = []
pcl_y = []
original_path = Path()
#/gps_utm_odom
is_odom = False
def odom_callback(data):
    global cur_x,cur_y,cur_th,is_odom
    is_odom = True
    cur_x = data.pose.pose.position.x
    cur_y = data.pose.pose.position.y
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    cur_th = euler[2]

def pcl_callback(data):
    global obj_pcl,cur_th,cur_x,cur_y,pcl_x,pcl_y
    #pcl_ros.fromROSMsg(data,obj_pcl)
    obj_pcl = data

    if is_odom:
        temp_pcl_x_list = []
        temp_pcl_y_list = []
        for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            temp_pcl_x = cos(cur_th)*pt_x - sin(cur_th)*pt_y + cur_x
            temp_pcl_y = sin(cur_th)*pt_x + cos(cur_th)*pt_y + cur_y
            temp_pcl_x_list.append(temp_pcl_x)
            temp_pcl_y_list.append(temp_pcl_y)
        pcl_x = temp_pcl_x_list
        pcl_y = temp_pcl_y_list

def original_path_callback(data):
    global original_path
    original_path = data

def callback(data):
    global step,step_count,pcl_x,pcl_y,original_path
    step_count = step_count + 1
    if step == step_count:
        step_count = 0

        x = []
        y = []
        original_path_x = []
        original_path_y = []
        print(len(original_path.poses))
        for i in range(len(original_path.poses)):
            original_path_x.append(original_path.poses[i].pose.position.x)
            original_path_y.append(original_path.poses[i].pose.position.y)

        for i in range(len(data.poses)):
            x.append(data.poses[i].pose.position.x)
            y.append(data.poses[i].pose.position.y)

        plt.plot(x,y,label='New Path',color='red',linewidth=3)
        plt.plot(original_path_x,original_path_y,'--',label='Origin Path',color='green',linewidth=3)
        plt.scatter(pcl_x,pcl_y,label='Obstacles')
#        plt.scatter(cur_x,cur_y,label='Pose',color='blue')
        plt.annotate("Pose", xy=(cur_x-1, cur_y+1), xytext=(cur_x, cur_y), arrowprops=dict(arrowstyle="->"),fontsize=15)
        plt.grid()
        plt.xlabel('x')
        plt.ylabel('y')

        plt.savefig('/home/a/asdasdasdasd.png', dpi = 300)
        plt.legend()
        plt.show()

def listener():
    rospy.init_node('local_costmap', anonymous=True)
    rospy.Subscriber("/local_path", Path, callback)
    rospy.Subscriber("/original_path", Path, original_path_callback)
    rospy.Subscriber("/Lidar/obj_pcl", PointCloud2, pcl_callback)
    rospy.Subscriber("/gps_utm_odom", Odometry, odom_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
