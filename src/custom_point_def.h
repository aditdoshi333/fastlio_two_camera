#pragma once
// #include <ros/ros.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
#include  <pcl/register_point_struct.h> 
#include <Eigen/Core>

namespace pcl {
 struct PointXYZRGBINormal
 {
   PCL_ADD_POINT4D;    // This adds the members x,y,z which can also be accessed using the point (which is float[4])
   PCL_ADD_NORMAL4D;   // This adds the member normal[3] which can also be accessed using the point (which is float[4])   
   union
     {
       struct
       {
         PCL_ADD_UNION_RGB;
         PCL_ADD_INTENSITY;
         float curvature;
        //  bool has_color;
       };
       float data_c[4];
     };
  //  union
  //  {
  //    struct
  //    {
  //      float rgb;
  //      float intensity;
  //      float curvature;
  //    };
  //    float data_c[4];
  //  };
  //   union
  //  {
  //    struct
  //    {
  //      float rgba;
  //      float r, g, b, a;
  //    };
  //    float data_d[4];
  //  };
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 } EIGEN_ALIGN16;

 inline std::ostream& operator << (std::ostream& os, const PointXYZRGBINormal& p)
 {
   os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.intensity << " - " << p.normal[0] << "," << p.normal[1] << "," << p.normal[2] << " - " << p.curvature << ")";
   return (os);
 }
}