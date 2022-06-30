#include <iostream>
#include <stdio.h>

#include "Eigen/src/Geometry/Quaternion.h"
#include "Eigen/src/Core/Matrix.h"

#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"

typedef struct RscanMemory
{
    Eigen::Matrix4f pose; // Rotation & Translation in one Homogenious form
    Eigen::Quaterniond q_nb;
    Eigen::Matrix3d C_nb;
    Eigen::Matrix4f trans_nb;
    Eigen::Vector3f position;

    pcl::PointCloud<pcl::PointXYZI> cloud;

} RscanMemoryStruct, *RscanMemoryStructPtr;