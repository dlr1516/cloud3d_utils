#ifndef CLOUD3D_UTILS_TYPES_H_
#define CLOUD3D_UTILS_TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Dense>

namespace cloud3d_utils {

using PointType = pcl::PointXYZ;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudTypePtr = pcl::PointCloud<PointType>::Ptr;

using Vector3 = Eigen::Vector3f;
using VectorVector3 = std::vector<Vector3>;
using Matrix3 = Eigen::Matrix3f;
using VectorMatrix3 = std::vector<Matrix3>;
using Transform3 = Eigen::Affine3f;

}  // namespace cloud3d_utils

#endif /*CLOUD3D_UTILS_TYPES_H_*/
