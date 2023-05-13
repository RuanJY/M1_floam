// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_H_
#define _LIDAR_H_
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
//define lidar parameter
//rjy
namespace robosenseM1_ros {
    struct Point {
        PCL_ADD_POINT4D

                PCL_ADD_INTENSITY;
        uint16_t ring;
        double timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT (
        robosenseM1_ros::Point,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(uint16_t, ring, ring)
(double, timestamp, timestamp)
)

namespace lidar{

class Lidar
{
    public:
        Lidar();

        void setScanPeriod(double scan_period_in);
        void setLines(double num_lines_in);
        void setVerticalAngle(double vertical_angle_in);
        void setVerticalResolution(double vertical_angle_resolution_in);
        //by default is 100. pls do not change
        void setMaxDistance(double max_distance_in);
        void setMinDistance(double min_distance_in);

    	double max_distance;
        double min_distance;
        int num_lines;
        double scan_period;
        int points_per_line;
        double horizontal_angle_resolution;
        double horizontal_angle;
        double vertical_angle_resolution;
        double vertical_angle;
        //used in laserProcessing
        int num_sub_cloud;
        int lidar_type;
        //used in odomEstimation
        double max_search_dis; // kdtree_d
        int optimization_times; //icp
        int iteration_times; //ceres
        double box_sides; //odom map
        double feature_resolution; //downsample feature
};


}


#endif // _LIDAR_H_

