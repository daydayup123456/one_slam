#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include "base.h"
#include "frame.h"
#include "parameter.h"
// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

// joinPointCloud 
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) ;

void createPointCloud(vector< FRAME > &keyframes,g2o::SparseOptimizer& globalOptimizer,CAMERA_INTRINSIC_PARAMETERS& camera);
#endif