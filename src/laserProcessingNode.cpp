// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
    //ROS_INFO("velodyneHandler");
   
}

double total_time =0;
int total_frame=0;

void laser_processing(){
    while(ros::ok()){
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();

            //convert M1 point cloud into 5 * 5 * n sector
            std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>>> pointcloud_subcloud_channel(5, std::vector<pcl::PointCloud<pcl::PointXYZI>>(5));
            // 5 * 5 * width array to store small point channel
            //ROS_INFO("before put");
            //old M1 bag zju, the width and height are exchanged
/*            for(int i_subcloud = 0; i_subcloud < pointcloud_in->height; i_subcloud++){//height, horizental
                for(int i_width = 0; i_width < pointcloud_in->width; i_width ++){//all point in this sector
                    //pointcloud_in->at(i_width, i_subcloud).intensity = i_subcloud + 10 * (i_width% 5);
                    pointcloud_subcloud_channel[i_subcloud][i_width % 5].push_back(pointcloud_in->at(i_width, i_subcloud));
                }
            }*/
            //new bag sdk
            for(int i_subcloud = 0; i_subcloud < pointcloud_in->width; i_subcloud++){//height, horizental
                for(int i_width = 0; i_width < pointcloud_in->height; i_width ++){//all point in this sector
                    //pointcloud_in->at(i_width, i_subcloud).intensity = i_subcloud + 10 * (i_width% 5);
                    pointcloud_subcloud_channel[i_subcloud][i_width % 5].push_back(pointcloud_in->at(i_subcloud, i_width));
                }
            }
            //ROS_INFO("after put");

            //test if point is correct, result: correct
/*            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*pointcloud_in,*pointcloud_in, indices);*/
/*            sensor_msgs::PointCloud2 laserCloudRaw;
            pcl::toROSMsg(*pointcloud_in, laserCloudRaw);
            laserCloudRaw.header.stamp = pointcloud_time;
            laserCloudRaw.header.frame_id = "base_link";
            pubLaserCloudFiltered.publish(laserCloudRaw);
            std::cout << "laserCloudRaw num: " << pointcloud_in->points.size() <<std::endl;
            std::cout << "height: " << pointcloud_in->height <<std::endl;

            for(int i=0; i<50; i++){
                std::cout << "point: " << pointcloud_in->at(i,0) <<std::endl;
            }*/

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            //laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            laserProcessing.featureExtractionM1(pointcloud_subcloud_channel, pointcloud_edge, pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    //ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 100, velodyneHandler);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);

    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); 

    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}

