// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"
#include "pointcloud_deskew.h"

#define DESKEW 0

void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution){
    //init local map
    laserCloudCornerMap = pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr(new pcl::PointCloud<RslidarM1PointXYZIRT>());
    laserCloudSurfMap = pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr(new pcl::PointCloud<RslidarM1PointXYZIRT>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(feature_resolution, feature_resolution, feature_resolution);
    downSizeFilterSurf.setLeafSize(feature_resolution * 2, feature_resolution * 2, feature_resolution * 2);

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<RslidarM1PointXYZIRT>::Ptr(new pcl::KdTreeFLANN<RslidarM1PointXYZIRT>());
    kdtreeSurfMap = pcl::KdTreeFLANN<RslidarM1PointXYZIRT>::Ptr(new pcl::KdTreeFLANN<RslidarM1PointXYZIRT>());

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();
    optimization_count=optimization_times;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& edge_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& surf_in){
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    optimization_count=12;//in the beginning, point cloud is sparse.
}


void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& edge_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& surf_in,
                                            FILE *fp1, FILE *fp2){

    if(optimization_count > optimization_times)
        optimization_count--;

    Eigen::Isometry3d T_last_curr = last_odom.inverse() * odom; // 此时 last_odom = T_{w}_{t-2}, T_last_curr = T_{t-2}_{t-1}
//    Eigen::Quaterniond q_last_curr(T_last_curr.linear());
    Eigen::Quaterniond q_last_curr = Eigen::Quaterniond(T_last_curr.rotation());
    Eigen::Vector3d t_last_curr = T_last_curr.translation();
    Eigen::Isometry3d odom_prediction = odom * T_last_curr;
//    Eigen::Isometry3d odom_prediction = odom;
    last_odom = odom; // 此时 last_odom = T_{w}_{t-1}
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    bool deskewPoint = false;

    pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr downsampledEdgeCloud(new pcl::PointCloud<RslidarM1PointXYZIRT>());
    pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr downsampledSurfCloud(new pcl::PointCloud<RslidarM1PointXYZIRT>());

    if (deskewPoint)
    {
        pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr edge_in_deskewed(new pcl::PointCloud<RslidarM1PointXYZIRT>());
        pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr surf_in_deskewed(new pcl::PointCloud<RslidarM1PointXYZIRT>());
        RslidarM1PointXYZIRT point_deskewed;

        for (size_t i = 0; i < edge_in->points.size(); ++i) {
            TransformToStart(&(edge_in->points[i]), &point_deskewed, q_last_curr, t_last_curr);
            edge_in_deskewed->push_back(point_deskewed);
        }
        for (size_t i = 0; i < surf_in->points.size(); ++i) {
            TransformToStart(&(surf_in->points[i]), &point_deskewed, q_last_curr, t_last_curr);
            surf_in_deskewed->push_back(point_deskewed);
        }
        downSamplingToMap(edge_in_deskewed,downsampledEdgeCloud,surf_in_deskewed,downsampledSurfCloud);
    }else{
        downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
    }

    bool write_size_log = false;

    if (write_size_log)
    {
        size_log(fp1, downsampledEdgeCloud->size());
        size_log(fp2, downsampledSurfCloud->size());
    }

    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if(laserCloudCornerMap->points.size()>10 && laserCloudSurfMap->points.size()>50){
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());

            if (DESKEW)
            {
                addEdgeCostFactor_deskew(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function,T_last_curr);
                addSurfCostFactor_deskew(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function,T_last_curr);
            }else {
                addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
                addSurfCostFactor(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function);
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = iteration_times;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);

        }
    }else{
        printf("not enough points in OdomEstimation::laserCloudCornerMap, map error");
    }
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;
    addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);

}

void OdomEstimationClass::pointAssociateToMap(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

//void OdomEstimationClass::pointAssociateToMap(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po)
//{
//    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
//
//    Eigen::Isometry3d T_w_curr = Eigen::Isometry3d::Identity();
//    T_w_curr.linear() = q_w_curr.toRotationMatrix();
//    T_w_curr.translation() = t_w_curr;
//
//    Eigen::Isometry3d T_last_curr_todeskew = last_odom.inverse() * T_w_curr;
//    Eigen::Quaterniond q_last_curr_todeskew = Eigen::Quaterniond(T_last_curr_todeskew.rotation());
//    Eigen::Vector3d t_last_curr_todeskew = T_last_curr_todeskew.translation();
//    //interpolation ratio
//    double s;
//    if (DISTORTION)
//        s = pi->intensity / SCAN_PERIOD;
//    else
//        s = 1.0;
//    Eigen::Quaterniond q_last_curr_deskewed = Eigen::Quaterniond::Identity().slerp(s, q_last_curr_todeskew);
//    Eigen::Vector3d t_last_curr_deskewed = s * t_last_curr_todeskew;
//    Eigen::Isometry3d T_last_curr_deskewed = Eigen::Isometry3d::Identity();
//    T_last_curr_deskewed.linear() = q_last_curr_deskewed.toRotationMatrix();
//    T_last_curr_deskewed.translation() = t_last_curr_deskewed;
//
//    Eigen::Isometry3d T_point_w_curr = Eigen::Isometry3d::Identity();
//    T_point_w_curr = last_odom * T_last_curr_deskewed;
//    Eigen::Quaterniond q_point_w_curr = Eigen::Quaterniond(T_point_w_curr.rotation());
//    Eigen::Vector3d t_point_w_curr = T_point_w_curr.translation();
//
//    Eigen::Vector3d point_w = q_point_w_curr * point_curr + t_point_w_curr;
//    po->x = point_w.x();
//    po->y = point_w.y();
//    po->z = point_w.z();
//    po->intensity = pi->intensity;
//    //po->intensity = 1.0;
//}

//void OdomEstimationClass::pointAssociateToMap(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po)
//{
//    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
//
//    Eigen::Isometry3d T_w_curr = Eigen::Isometry3d::Identity();
//    T_w_curr.linear() = q_w_curr.toRotationMatrix();
//    T_w_curr.translation() = t_w_curr;
//
//    Eigen::Isometry3d T_last_curr_todeskew = last_odom.inverse() * T_w_curr;
//    Eigen::Quaterniond q_last_curr_todeskew = Eigen::Quaterniond(T_last_curr_todeskew.rotation());
//    Eigen::Vector3d t_last_curr_todeskew = T_last_curr_todeskew.translation();
//    //interpolation ratio
//    double s;
//    if (DISTORTION)
//        s = 1.0 - pi->intensity / SCAN_PERIOD;
//    else
//        s = 1.0;
//    //插值得到当前帧结束时刻到当前点时刻的位姿变换，取逆可将当前点去运动畸变到当前帧结束时刻
//    Eigen::Quaterniond q_last_curr_deskewed = Eigen::Quaterniond::Identity().slerp(s, q_last_curr_todeskew);
//    Eigen::Vector3d t_last_curr_deskewed = s * t_last_curr_todeskew;
//    //transform point to end
//    Eigen::Vector3d un_point = q_last_curr_deskewed.inverse() * (point_curr - t_last_curr_deskewed);
//
//    Eigen::Vector3d point_w = q_w_curr * un_point + t_w_curr;
//    po->x = point_w.x();
//    po->y = point_w.y();
//    po->z = point_w.z();
//    po->intensity = pi->intensity;
//    //po->intensity = 1.0;
//}

void OdomEstimationClass::pointAssociateToMap_CostFactor(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po, const Eigen::Isometry3d& T_last_curr)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);

    Eigen::Isometry3d T_last_curr_todeskew = T_last_curr;
    Eigen::Quaterniond q_last_curr_todeskew = Eigen::Quaterniond(T_last_curr_todeskew.rotation());
    Eigen::Vector3d t_last_curr_todeskew = T_last_curr_todeskew.translation();
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = pi->intensity / SCAN_PERIOD;
    else
        s = 1.0;
    Eigen::Quaterniond q_last_curr_deskewed = Eigen::Quaterniond::Identity().slerp(s, q_last_curr_todeskew);
    Eigen::Vector3d t_last_curr_deskewed = s * t_last_curr_todeskew;
    Eigen::Isometry3d T_last_curr_deskewed = Eigen::Isometry3d::Identity();
    T_last_curr_deskewed.linear() = q_last_curr_deskewed.toRotationMatrix();
    T_last_curr_deskewed.translation() = t_last_curr_deskewed;

    Eigen::Isometry3d T_point_w_curr = Eigen::Isometry3d::Identity();
    T_point_w_curr = last_odom * T_last_curr_deskewed;
    Eigen::Quaterniond q_point_w_curr = Eigen::Quaterniond(T_point_w_curr.rotation());
    Eigen::Vector3d t_point_w_curr = T_point_w_curr.translation();

    Eigen::Vector3d point_w = q_point_w_curr * point_curr + t_point_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void OdomEstimationClass::pointAssociateToMap_UpdateMap(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);

    Eigen::Isometry3d T_w_curr = Eigen::Isometry3d::Identity(); // T_w_curr此时已经是经过估计优化后的当前帧位姿
    T_w_curr.linear() = q_w_curr.toRotationMatrix();
    T_w_curr.translation() = t_w_curr;

    Eigen::Isometry3d T_last_curr_todeskew = last_odom.inverse() * T_w_curr; // 优化后的当前帧到上一帧的位姿变换
    Eigen::Quaterniond q_last_curr_todeskew = Eigen::Quaterniond(T_last_curr_todeskew.rotation());
    Eigen::Vector3d t_last_curr_todeskew = T_last_curr_todeskew.translation();
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = pi->intensity / SCAN_PERIOD;
    else
        s = 1.0;
    Eigen::Quaterniond q_last_curr_deskewed = Eigen::Quaterniond::Identity().slerp(s, q_last_curr_todeskew);
    Eigen::Vector3d t_last_curr_deskewed = s * t_last_curr_todeskew;

//    Eigen::Isometry3d T_last_curr_deskewed = Eigen::Isometry3d::Identity();
//    T_last_curr_deskewed.linear() = q_last_curr_deskewed.toRotationMatrix();
//    T_last_curr_deskewed.translation() = t_last_curr_deskewed;
//
//    Eigen::Isometry3d T_point_w_curr = Eigen::Isometry3d::Identity();
//    T_point_w_curr = last_odom * T_last_curr_deskewed;
//    Eigen::Quaterniond q_point_w_curr = Eigen::Quaterniond(T_point_w_curr.rotation());
//    Eigen::Vector3d t_point_w_curr = T_point_w_curr.translation();

    Eigen::Vector3d point_w = q_last_curr_deskewed * point_curr + t_last_curr_deskewed;
//    Eigen::Vector3d point_w = q_point_w_curr * point_curr + t_point_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& edge_pc_in, pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& edge_pc_out, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& surf_pc_in, pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);    
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& pc_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        RslidarM1PointXYZIRT point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp); // 此时是T_{w}_{t}

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); 
        if (pointSearchSqDis[4] < max_search_dis)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            { 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;   
            }                           
        }
    }
    if(corner_num<20){
        std::cout << "not enough valid edge points" << std::endl;
    }

}

void OdomEstimationClass::addEdgeCostFactor_deskew(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& pc_in,
                                                   const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& map_in,
                                                   ceres::Problem& problem,
                                                   ceres::LossFunction *loss_function,
                                                   const Eigen::Isometry3d& T_last_curr){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        RslidarM1PointXYZIRT point_temp;
//        pointAssociateToMap(&(pc_in->points[i]), &point_temp); // 此时是T_{w}_{t}

        pointAssociateToMap_CostFactor(&(pc_in->points[i]), &point_temp, T_last_curr);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis[4] < max_search_dis)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;
            }
        }
    }
    if(corner_num<20){
        std::cout << "not enough valid edge points" << std::endl;
    }

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& pc_in, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        RslidarM1PointXYZIRT point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp); // 此时是T_{w}_{t}
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < max_search_dis)
        {
            
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);    
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        std::cout << "not enough valid surf points" << std::endl;
    }

}

void OdomEstimationClass::addSurfCostFactor_deskew(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& pc_in,
                                                   const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& map_in,
                                                   ceres::Problem& problem,
                                                   ceres::LossFunction *loss_function,
                                                   const Eigen::Isometry3d& T_last_curr){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        RslidarM1PointXYZIRT point_temp;
//        pointAssociateToMap(&(pc_in->points[i]), &point_temp); // 此时是T_{w}_{t}
        pointAssociateToMap_CostFactor(&(pc_in->points[i]), &point_temp, T_last_curr);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < max_search_dis)
        {

            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        std::cout << "not enough valid surf points" << std::endl;
    }

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& downsampledSurfCloud){

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        RslidarM1PointXYZIRT point_temp;
        if (DESKEW)
        {
            pointAssociateToMap_UpdateMap(&downsampledEdgeCloud->points[i], &point_temp);
        }else {
            pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        }
        laserCloudCornerMap->push_back(point_temp);
    }
    
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        RslidarM1PointXYZIRT point_temp;
        if (DESKEW)
        {
            pointAssociateToMap_UpdateMap(&downsampledSurfCloud->points[i], &point_temp);
        }else {
            pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
        }
        laserCloudSurfMap->push_back(point_temp);
    }
    //double box_sides = 200;
    double x_min = +odom.translation().x() - box_sides;
    double y_min = +odom.translation().y() - box_sides;
    double z_min = +odom.translation().z() - box_sides;
    double x_max = +odom.translation().x() + box_sides;
    double y_max = +odom.translation().y() + box_sides;
    double z_max = +odom.translation().z() + box_sides;
    
    //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false); // 是保留立方体内的点而去除其他点，还是反之。false是将盒子外的点去除，默认为false

    pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr tmpCorner(new pcl::PointCloud<RslidarM1PointXYZIRT>());
    pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr tmpSurf(new pcl::PointCloud<RslidarM1PointXYZIRT>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    downSizeFilterSurf.setInputCloud(tmpSurf);
    downSizeFilterSurf.filter(*laserCloudSurfMap);
    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);

}

void OdomEstimationClass::getMap(pcl::PointCloud<RslidarM1PointXYZIRT>::Ptr& laserCloudMap){
    
    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

OdomEstimationClass::OdomEstimationClass(){

}
