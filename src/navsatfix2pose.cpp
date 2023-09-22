#include<ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen3/Eigen/Dense>
using namespace std;


/*********************2023.08.18*****************
LYF
订阅ins Odometry数据
转换为xyz 发布TransformStamped数据
方便lidar_align

********************************************/






//角度制转弧度制
double rad(double d)
{
	return d * 3.14159265 / 180.0;
}
// ROS订阅者和发布者
ros::Subscriber gps_data_sub;
ros::Publisher path_pub ,gps_data_pub,gps_odom_pub;
// 全局变量
string gps_sub_topic = "";
string output_frame_name ="";
double z_rotate_value = 0.0;
double origin_latitude_value = 0.0;
double origin_longitude_value = 0.0;
double origin_altitude_value = 0.0;
double latitude_resolution = 0.0;
double longitude_resolution = 0.0;
double altitude_resolution = 0.0;
Eigen::Quaterniond origin_quat_value;
bool init_flag = true;
nav_msgs::Path path;

Eigen::Vector3d origin_xyz;

Eigen::Vector3d lla2xyz(float lon, float lat,float height){


    // 3.1415926535898/180.0
    double iPI = 0.0174532925199433;
    //# 3度带
    double zoneWide = 3;
    //# 长半轴
    double a = 6378137;
    //# 扁率
    double f = 1/298.257222101;

    int projNo = int(lon/zoneWide);
    double longitude0 = projNo*3;
    longitude0 = longitude0 * iPI;
    double longitude1 = lon * iPI;
    double latitude1 = lat * iPI;

    double e2 = 2 * f - f * f;
    double ee = e2 * (1.0 - e2);
    double NN = a / sqrt(1.0 - e2 * sin(latitude1) * sin(latitude1));
    double T = tan(latitude1) * tan(latitude1);
    double C = ee * cos(latitude1) * cos(latitude1);
    double A = (longitude1 - longitude0) * cos(latitude1);
    double M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * latitude1 - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) *
             sin(2 * latitude1) + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * latitude1) - (35 * e2 * e2 * e2 / 3072) * sin(6 * latitude1));

    double xval = NN * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
    double yval = M + NN * tan(latitude1) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A *
                                           A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A * A / 720);

    //# X0 = 1000000 * projNo + 500000 不加带号
    double X0 = 500000;   //#北方向加500000;
    double Y0 = 0;
     xval = xval + X0;
     yval = yval + Y0;
    double guss_x = yval;
    double guss_y = xval;
    return Eigen::Vector3d(guss_x, -guss_y,height);
}
void gps_to_xyz(const nav_msgs::Odometry::ConstPtr& gps_msg)
{
    if (init_flag == true)
    {   
        origin_longitude_value = gps_msg->pose.pose.position.x;
        origin_latitude_value = gps_msg->pose.pose.position.y;
        origin_altitude_value = gps_msg->pose.pose.position.z;

        origin_xyz=lla2xyz(origin_longitude_value,origin_latitude_value,origin_altitude_value);
         origin_quat_value = Eigen::Quaterniond(gps_msg->pose.pose.orientation.w,gps_msg->pose.pose.orientation.x,
                                gps_msg->pose.pose.orientation.y,gps_msg->pose.pose.orientation.z);
        init_flag = false;
    }
    else
    {
        double gps_lat = gps_msg->pose.pose.position.y;
        double gps_lon = gps_msg->pose.pose.position.x;
        double gps_hei = gps_msg->pose.pose.position.z;
        Eigen::Vector3d nowxyz =  lla2xyz(gps_lon,gps_lat,gps_hei);
          
 	
	Eigen::Vector3d gps_pose= nowxyz-origin_xyz;
	Eigen::Vector3d gps_odom_pose = origin_quat_value.inverse()*gps_pose;
	Eigen::Quaterniond gps_quat(gps_msg->pose.pose.orientation.w,gps_msg->pose.pose.orientation.x,
                                gps_msg->pose.pose.orientation.y,gps_msg->pose.pose.orientation.z);
	Eigen::Quaterniond gps_odom_quat = origin_quat_value.inverse()*gps_quat;

        geometry_msgs::TransformStamped gps_Tran;
        gps_Tran.header.frame_id = "camera_init";
        gps_Tran.transform.translation.x = gps_odom_pose(0);
        gps_Tran.transform.translation.y = gps_odom_pose(1);
        gps_Tran.transform.translation.z = gps_odom_pose(2);
        gps_Tran.transform.rotation.x = gps_odom_quat.x();
        gps_Tran.transform.rotation.y = gps_odom_quat.y();
        gps_Tran.transform.rotation.z = gps_odom_quat.z();
        gps_Tran.transform.rotation.w = gps_odom_quat.w();


	nav_msgs::Odometry gps_odom;
	gps_odom.header.frame_id = "camera_init";
	gps_odom.pose.pose.position.x = gps_odom_pose(0);
        gps_odom.pose.pose.position.y = gps_odom_pose(1);
        gps_odom.pose.pose.position.z = gps_odom_pose(2);
        gps_odom.pose.pose.orientation.x = gps_odom_quat.x();
        gps_odom.pose.pose.orientation.y = gps_odom_quat.y();
        gps_odom.pose.pose.orientation.z = gps_odom_quat.z();
        gps_odom.pose.pose.orientation.w = gps_odom_quat.w();
	gps_odom_pub.publish(gps_odom);



       // std::cout<<"   "<<origin_quat_value.x()<<"   "<<origin_quat_value.y()<<"   "<<origin_quat_value.z()<<std::endl;
        gps_data_pub.publish(gps_Tran);
        if (path_pub.getNumSubscribers() > 0)
        {
            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.header.frame_id = "camera_init";
            this_pose_stamped.pose.position.x = gps_odom_pose(0);
            this_pose_stamped.pose.position.y = gps_odom_pose(1);
            this_pose_stamped.pose.position.z = gps_odom_pose(2);
            this_pose_stamped.pose.orientation.x = gps_odom_quat.x();
            this_pose_stamped.pose.orientation.y = gps_odom_quat.y();
            this_pose_stamped.pose.orientation.z = gps_odom_quat.z();
            this_pose_stamped.pose.orientation.w = gps_odom_quat.w();
            path.poses.push_back(this_pose_stamped);
            path.header.frame_id = "camera_init";
        }
        path_pub.publish(path);
    }
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"gps_deal");
    ros::NodeHandle nh;

    gps_data_sub = nh.subscribe<nav_msgs::Odometry>("/gps/ins", 1000, gps_to_xyz);
    gps_data_pub = nh.advertise<geometry_msgs::TransformStamped>("/test/trans", 1000, true);
    gps_odom_pub = nh.advertise<nav_msgs::Odometry>("/test/odom", 1000, true);

    path_pub = nh.advertise<nav_msgs::Path>("/gpsTrack",1, true);
    ros::spin();
    return 0;
}
