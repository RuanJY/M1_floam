// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

#define PCL_NO_PRECOMPILE

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>
#include "pointtype.h"
#include "write_log.h"

class OdomEstimationClass 
{

    public:
    	OdomEstimationClass();
    	
		void init(lidar::Lidar lidar_param, double map_resolution);	
		void initMapWithPoints(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& edge_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& surf_in);
		void updatePointsToMap(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& edge_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& surf_in,
		                        FILE *fp1, FILE *fp2);
		void getMap(pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& laserCloudMap);

		Eigen::Isometry3d odom;
		pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr laserCloudCornerMap;
		pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr laserCloudSurfMap;
	private:
        //param
        double max_search_dis = 25; // kdtree_d
        int optimization_times = 6; //icp
        int iteration_times = 8; //ceres
        double box_sides = 100; //odom map
        double feature_resolution = 0.5; //downsample feature

    //optimization variable
		double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
		Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

		Eigen::Isometry3d last_odom;

		//kd-tree
		pcl::KdTreeFLANN<RslidarM1PointXYZIRT>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<RslidarM1PointXYZIRT>::Ptr kdtreeSurfMap;

		//points downsampling before add to map
		pcl::VoxelGrid<RslidarM1PointXYZIRT> downSizeFilterEdge;
		pcl::VoxelGrid<RslidarM1PointXYZIRT> downSizeFilterSurf;

		//local map
		pcl::CropBox<RslidarM1PointXYZIRT> cropBoxFilter;

		//optimization count 
		int optimization_count;

		//function
		void addEdgeCostFactor(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& pc_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
        void addEdgeCostFactor_deskew(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& pc_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function, const Eigen::Isometry3d& T_last_curr);
		void addSurfCostFactor(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& pc_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
        void addSurfCostFactor_deskew(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& pc_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function, const Eigen::Isometry3d& T_last_curr);
		void addPointsToMap(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& downsampledSurfCloud);
		void pointAssociateToMap(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po);
		void pointAssociateToMap_CostFactor(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po, const Eigen::Isometry3d& T_last_curr);
        void pointAssociateToMap_UpdateMap(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po);
		void downSamplingToMap(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& edge_pc_in, pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& edge_pc_out, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& surf_pc_in, pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& surf_pc_out);
};

#endif // _ODOM_ESTIMATION_CLASS_H_

