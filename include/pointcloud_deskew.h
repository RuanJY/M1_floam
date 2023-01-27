#ifndef _POINTCLOUD_DESKEW_H_
#define _POINTCLOUD_DESKEW_H_

#define DISTORTION 1
#define SCAN_PERIOD 0.1

// undistort lidar point
void TransformToStart(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po,
                      const Eigen::Quaterniond& q_last_curr, const Eigen::Vector3d& t_last_curr)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = pi->timestamp / SCAN_PERIOD;
    else
        s = 1.0;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last; // undistorted_point

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

void TransformToEnd(RslidarM1PointXYZIRT const *const pi, RslidarM1PointXYZIRT *const po,
                    const Eigen::Quaterniond& q_last_curr, const Eigen::Vector3d& t_last_curr)
{
    // undistort point first
    RslidarM1PointXYZIRT un_point_tmp;
    TransformToStart(pi, &un_point_tmp, q_last_curr, t_last_curr); // 先将所有点做运动补偿到同一时刻(上一时刻)

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity); // 所有po点的索引只保留了scanID，int强制类型转换只保留了整数部分
}

#endif // _POINTCLOUD_DESKEW_H_