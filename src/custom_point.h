#pragma once
// #include <ros/ros.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
#include  <pcl/register_point_struct.h> 
#include <Eigen/Core>

namespace pcl {
    struct PointXYZRGBINormal;
}

#include "custom_point_def.h"

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZRGBINormal,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (std::uint32_t, rgba, rgba)
                                //    (std::uint8_t, r, r)
                                //    (std::uint8_t, g, g)
                                //    (std::uint8_t, b, b)
                                //    (std::uint8_t, a, a)
                                //    (float, rgba, rgba)
                                   (float, intensity, intensity)
                                   (float, curvature, curvature)
);
