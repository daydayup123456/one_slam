#ifndef FRAME_H
#define FRAME_H
#include "common.h"
#include "parameter.h"
// 帧结构
struct FRAME
{
    int frameID; 
    cv::Mat rgb, depth; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};
// 给定index，读取一帧数据
FRAME readFrame( int index, ParameterReader& pd,string rgb_path,string depth_path);
#endif