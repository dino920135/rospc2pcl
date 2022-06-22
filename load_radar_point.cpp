#include "pcl/io/pcd_io.h"
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


#define postprocess

using namespace std;

void RScan_callback(const sensor_msgs::PointCloud2::ConstPtr &RScan_msg) 
{
    double t = RScan_msg->header.stamp.toSec();
    cout << setprecision(14);
    cout << t << endl;
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
    bag.open("/mnt/e/DATA/ColoRadar/outdoors_run0.bag", rosbag::bagmode::Read);
    cout << " Done." << endl;

    std::vector<std::string> topics;
    topics.push_back(std::string("/mmWaveDataHdl/RScan"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != NULL)
        {
            RScan_callback(msg);
            int point_bytes = msg->point_step;
            int offset_x, offset_y, offset_z, offset_intensity;

            const auto& fields = msg->fields;
            for (int f = 0; f < fields.size(); ++f)
            {
                if (fields[f].name == "x") offset_x = fields[f].offset;
                if (fields[f].name == "y") offset_y = fields[f].offset;
                if (fields[f].name == "z") offset_z = fields[f].offset;
                if (fields[f].name == "intensity") offset_intensity = fields[f].offset;
            }

            // Tranformation
            pcl::PointCloud<pcl::PointXYZI> cloud;
            for (int i = 0; i < msg->width; ++i)
            {
                pcl::PointXYZI point;
                point.x = *(float*)(msg->data.data() + point_bytes*i + offset_x);
                point.y = *(float*)(msg->data.data() + point_bytes*i + offset_y);
                point.z = *(float*)(msg->data.data() + point_bytes*i + offset_z);
                point.intensity = *(unsigned char*)(msg->data.data() + point_bytes*i + offset_intensity);

                // auto tmp = point.x;
                // point.x = point.z;
                // point.y = -point.y;
                // point.z = tmp;

                cloud.push_back(point);
            }
            cloud.width = msg->width;
            cloud.height = msg->height;

            string pcd_path = "../../PCD/" + to_string(msg->header.stamp.toNSec()) + ".pcd";
            SavePCD(cloud, pcd_path);
        }
        
    }
    bag.close();
    return 0;
}
#endif