#include "frame.h"
FRAME readFrame( int index, ParameterReader& pd,string rgb_path,string depth_path )
{
    FRAME f;//path_to_rgb,path_to_depth
    f.rgb = cv::imread(rgb_path );
    f.depth = cv::imread(depth_path , -1 );
    f.frameID = index;
    return f;
}