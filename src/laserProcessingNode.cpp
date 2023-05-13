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
            int lidar_type = lidar_param.lidar_type; //0 velodyne, 1 M1
            if(lidar_type == 0){
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());
                //read data
                mutex_lock.lock();
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
                ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
                pointCloudBuf.pop();
                mutex_lock.unlock();

                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);

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
            else if(lidar_type == 1){
                mutex_lock.lock();
                pcl::PointCloud<robosenseM1_ros::Point> pl_orig;
                pcl::fromROSMsg(*pointCloudBuf.front(), pl_orig);
                ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
                int plsize = pl_orig.size();
                double time_stamp = pointCloudBuf.front()->header.stamp.toSec();
                pointCloudBuf.pop();
                mutex_lock.unlock();

                int i_sub_cloud, num_sub_cloud = lidar_param.num_sub_cloud;
                double strat_time,  end_time;
                double blind = 1.0;
                //reordered
                int num_point_each_sub_cloud = plsize/pl_orig.width/num_sub_cloud;
                robosenseM1_ros::Point first_point =pl_orig.points[num_point_each_sub_cloud * i_sub_cloud];
                for(i_sub_cloud = 0; i_sub_cloud < num_sub_cloud; i_sub_cloud++ ){//split
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());
                    std::vector<pcl::PointCloud<pcl::PointXYZI>> pointcloud_subcloud_channel(5);
                    for(int i_ori_width = 0; i_ori_width < pl_orig.width; i_ori_width ++){
                        for(int i_ori_height = num_point_each_sub_cloud * i_sub_cloud;
                            i_ori_height < num_point_each_sub_cloud * (i_sub_cloud+1); i_ori_height ++) {

                            robosenseM1_ros::Point & ori_point = pl_orig.at(i_ori_width, i_ori_height);
                            if(i_ori_height == num_point_each_sub_cloud * i_sub_cloud){//record time of the first point
                                strat_time = ori_point.timestamp;
                            }else if(i_ori_height == num_point_each_sub_cloud * (i_sub_cloud+1) - 1){//record time of the last point
                                end_time = ori_point.timestamp;
                            }
                            //if (i_ori_height % point_filter_num != 0) {continue;}

                            double range = ori_point.x * ori_point.x + ori_point.y * ori_point.y + ori_point.z * ori_point.z;
                            if(sqrt(range) < 150 && sqrt(range) > blind){

                                Eigen::Vector3d pt_vec;
                                pcl::PointXYZI added_pt;
                                added_pt.x = ori_point.x;
                                added_pt.y = ori_point.y;
                                added_pt.z = ori_point.z;
                                added_pt.intensity = ori_point.intensity;
                                pointcloud_subcloud_channel[i_ori_width].push_back(added_pt);
                            }

                        }
                    }
                    laserProcessing.featureExtractionM1(pointcloud_subcloud_channel, pointcloud_edge, pointcloud_surf);

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

            }
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

    nh.getParam("/num_sub_cloud", lidar_param.num_sub_cloud);
    nh.getParam("/lidar_type", lidar_param.lidar_type);

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

