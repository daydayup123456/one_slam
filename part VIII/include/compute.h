#ifndef COMPUTE_H
#define COMPUTE_H
#include "base.h"
#include "frame.h"
// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor );

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

// 度量运动的大小
double normofTransform( cv::Mat rvec, cv::Mat tvec );
#endif