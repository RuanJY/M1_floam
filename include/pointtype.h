#ifndef _POINTTYPE_H_
#define _POINTTYPE_H_

struct RslidarM1PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (RslidarM1PointXYZIRT,
        (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (double, timestamp, timestamp)
)

#endif // _POINTTYPE_H_