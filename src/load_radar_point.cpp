#include <stdio.h>
#include <stdlib.h>
#include <iomanip>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "pcl/io/pcd_io.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl/impl/point_types.hpp"
#include <pcl/point_cloud.h>

#include "GlobalParaDef.h"

#define postprocess

using namespace std;

RscanMemoryStructPtr RscanMem = new RscanMemory;

void RScan_callback(const sensor_msgs::PointCloud2::ConstPtr &RScan_msg) 
{
    double t = RScan_msg->header.stamp.toSec();
    cout << setprecision(14);
    cout << t << endl;
    int point_bytes = RScan_msg->point_step;
    int offset_x, offset_y, offset_z, offset_intensity;

    const auto& fields = RScan_msg->fields;
    for (int f = 0; f < fields.size(); ++f)
    {
        if (fields[f].name == "x") offset_x = fields[f].offset;
        if (fields[f].name == "y") offset_y = fields[f].offset;
        if (fields[f].name == "z") offset_z = fields[f].offset;
        if (fields[f].name == "intensity") offset_intensity = fields[f].offset;
    }

    // Tranformation
    for (int i = 0; i < RScan_msg->width; ++i)
    {
        pcl::PointXYZI point;
        point.x = *(float*)(RScan_msg->data.data() + point_bytes*i + offset_x);
        point.y = *(float*)(RScan_msg->data.data() + point_bytes*i + offset_y);
        point.z = *(float*)(RScan_msg->data.data() + point_bytes*i + offset_z);

        if (sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)) < 1)
            continue;
        // else if (point.intensity <= 50)
        //     continue;

        point.intensity = *(unsigned char*)(RScan_msg->data.data() + point_bytes*i + offset_intensity);

        auto tmp = point.x;
        // point.x = point.z;
        point.x = -point.y;
        point.y = tmp;

        // cout << "intensity " << point.intensity << " " << (point.intensity < 50) << endl;

        RscanMem->cloud.push_back(point);
    }
    RscanMem->cloud.width = RScan_msg->width;
    RscanMem->cloud.height = RScan_msg->height;
}

void Ground_truth_callback(const nav_msgs::Odometry::ConstPtr &GT_msg)
{
    RscanMem->trans_nb = Eigen::Matrix4f::Identity();
    /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
    */
    // Translation
    RscanMem->trans_nb(0, 3) = GT_msg->pose.pose.position.x;
    RscanMem->trans_nb(1, 3) = GT_msg->pose.pose.position.y;
    RscanMem->trans_nb(2, 3) = GT_msg->pose.pose.position.z;

    // Rotation
    // Quaternion
    RscanMem->q_nb.w() = GT_msg->pose.pose.orientation.w;
    RscanMem->q_nb.x() = GT_msg->pose.pose.orientation.x;
    RscanMem->q_nb.y() = GT_msg->pose.pose.orientation.y;
    RscanMem->q_nb.z() = GT_msg->pose.pose.orientation.z;
    // Rotation Matrix
    RscanMem->C_nb = RscanMem->q_nb.normalized().toRotationMatrix();
    // Transformation Matrix (in homogenious form)
    RscanMem->trans_nb(0, 0) = RscanMem->C_nb(0, 0);
    RscanMem->trans_nb(0, 1) = RscanMem->C_nb(0, 1);
    RscanMem->trans_nb(0, 2) = RscanMem->C_nb(0, 2);
    RscanMem->trans_nb(1, 0) = RscanMem->C_nb(1, 0);
    RscanMem->trans_nb(1, 1) = RscanMem->C_nb(1, 1);
    RscanMem->trans_nb(1, 2) = RscanMem->C_nb(1, 2);
    RscanMem->trans_nb(2, 0) = RscanMem->C_nb(2, 0);
    RscanMem->trans_nb(2, 1) = RscanMem->C_nb(2, 1);
    RscanMem->trans_nb(2, 2) = RscanMem->C_nb(2, 2);

    // cout << RscanMem->trans_nb << endl;
}

void SavePCD(const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::string& path)
{
    cout << "pcd_path: " << path << endl;
    pcl::io::savePCDFileASCII(path, cloud);
}

// Realtime
#ifdef Realtime
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_fusion");
	ros::NodeHandle n("~");
    ros::Subscriber sub_RScan = n.subscribe("/mmWaveDataHdl/RScan", 100, RScan_callback);
    ros::spin();

    return 0;
}
#endif
#ifdef postprocess
int main(int argc, char** argv)
{
    rosbag::Bag bag;
    cout << "Opening bagfile ......";
    bag.open("/home/point001/TS/Coloradar/outdoors_run0.bag", rosbag::bagmode::Read);
    cout << " Done." << endl;

    std::vector<std::string> topics;
    topics.push_back(std::string("/mmWaveDataHdl/RScan"));
    topics.push_back(std::string("/lidar_ground_truth"));

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudDg (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPose (new pcl::PointCloud<pcl::PointXYZ>);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator iter;
    iter = view.begin();

    // Visualizing
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D viewer"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI> (cloud, "cloud");
    viewer->addPointCloud<pcl::PointXYZ> (cloudPose, "cloudPose");
    viewer->spinOnce();

    while(iter != view.end())
    {
        iter++;
        auto m = *iter;

        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != NULL)
        {
            // Clear previous cloud for new scan
            RscanMem->cloud.clear();
            RScan_callback(pc_msg);
            
            // Tranform to DG pointcloud by Ground Truth pose
            pcl::transformPointCloud (RscanMem->cloud, RscanMem->cloud, RscanMem->trans_nb);
            *cloudDg += RscanMem->cloud;

            viewer->removePointCloud("cloud", 0);
            viewer->addPointCloud<pcl::PointXYZI> (cloudDg, "cloud");
            viewer->spinOnce();

            // string pcd_path = "../../PCD/" + to_string(msg->header.stamp.toNSec()) + ".pcd";
            // SavePCD(cloud, pcd_path);
        }

        nav_msgs::Odometry::ConstPtr gt_msg = m.instantiate<nav_msgs::Odometry>();
        if (gt_msg != NULL )
        {
            Ground_truth_callback(gt_msg);
            pcl::PointXYZ pose;
            pose.x = RscanMem->position(0);
            pose.y = RscanMem->position(1);
            pose.z = RscanMem->position(2);
            cloudPose->push_back(pose);
            viewer->removePointCloud("cloudPose", 0);
            viewer->addPointCloud<pcl::PointXYZ> (cloudPose, "cloudPose");
            viewer->spinOnce();
        }
    }
    bag.close();
    return 0;
}
#endif