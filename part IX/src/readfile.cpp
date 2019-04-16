#include "readfile.h"
int readfile_getstring( vector<string>& path_to_rgb,vector<string>& path_to_depth)
{
    ParameterReader pd;
    string path_to_dataset = pd.getData("path_to_dataset");
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );
    //给予路径。图片信息文件路径和机器人运动数据文件路径
    string associate_file = path_to_dataset + "/associate.txt";
    //找不到怎么办
    ifstream fin1( associate_file );
    if ( !fin1 ) 
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return 1;
    }
    //给予读取时的内容名
    string rgb_file, depth_file, time_rgb, time_depth;    
    //读取图片时存储名
    cv::Mat color, depth;
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;         // 相机位姿
    //标志位，检测到图片时间和机器人位置时间相等时变成1
    for ( int index=0; index<=endIndex; index++)
    {
	//读取图像位置信息，并存储
	 fin1>>time_rgb>>rgb_file>>time_depth>>depth_file;
	path_to_rgb.push_back  (path_to_dataset+"/"+rgb_file  );
	path_to_depth.push_back(path_to_dataset+"/"+depth_file);
    }
}