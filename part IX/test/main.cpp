#include "base.h"
#include "compute.h"
#include "readfile.h"
#include "pointcloud.h"
#include "loop.h"

typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

vector<string> path_to_rgb;vector<string> path_to_depth;

int main( int argc, char** argv )
{
    ParameterReader pd;
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );
    int class_solver = atoi( pd.getData( "class_solver" ).c_str() );
    int class_optiza = atoi( pd.getData( "class_optiza" ).c_str() );
    // initialize
    cout<<"Initializing ..."<<endl;
    readfile_getstring( path_to_rgb,path_to_depth);
    // 所有的关键帧都放在了这里
    vector< FRAME > keyframes;
    int currIndex = startIndex; // 当前索引为currIndex
    string rgb_path = path_to_rgb.at(currIndex) ;
    string depth_path = path_to_depth.at(currIndex) ;
    FRAME lastFrame = readFrame( currIndex, pd ,rgb_path,depth_path);
    // 我们总是在比较currFrame和lastFrame
    string detector = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp( lastFrame, detector, descriptor );
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );
    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );
     
    // 新增:有关g2o的初始化
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver;
    if(class_solver==1) linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
    //if(class_solver==2) linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    if(class_solver==3) linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
    if(class_solver==4) linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* blockSolver = new Block( unique_ptr<Block::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithm* solver ;
    if(class_optiza==1) solver = new g2o::OptimizationAlgorithmLevenberg( unique_ptr<Block>(blockSolver) );
    if(class_optiza==2) solver = new g2o::OptimizationAlgorithmDogleg( unique_ptr<Block>(blockSolver) );
    if(class_optiza==3) solver = new g2o::OptimizationAlgorithmGaussNewton( unique_ptr<Block>(blockSolver) );
    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm( solver ); 
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );
    
    keyframes.push_back( lastFrame );
    
    double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    bool check_loop_closure = pd.getData("check_loop_closure")==string("yes");
    int lastIndex = currIndex;
    //循环
    for ( currIndex=startIndex+1; currIndex<endIndex; currIndex++ )
    {
        cout<<"Reading files "<<currIndex<<endl;
	rgb_path = path_to_rgb.at(currIndex) ;
	depth_path = path_to_depth.at(currIndex) ;
        FRAME currFrame = readFrame( currIndex,pd ,rgb_path,depth_path); // 读取currFrame
	computeKeyPointsAndDesp( currFrame, detector, descriptor ); //提取特征
        CHECK_RESULT result = checkKeyframes( keyframes.back(), currFrame, globalOptimizer ); //匹配该帧与keyframes里最后一帧
        switch (result) // 根据匹配结果不同采取不同策略
        {
        case NOT_MATCHED:
            //没匹配上，直接跳过
            cout<<"Not enough inliers."<<endl;
            break;
        case TOO_FAR_AWAY:
            // 太近了，也直接跳
            cout<<"Too far away, may be an error."<<endl;
            break;
        case TOO_CLOSE:
            // 太远了，可能出错了
            cout<<"Too close, not a keyframe"<<endl;
            break;
        case KEYFRAME:
            cout<<"This is a new keyframe"<<endl;
            // 不远不近，刚好
            // 检测回环
            if (check_loop_closure)
            {
                checkNearbyLoops( keyframes, currFrame, globalOptimizer );
                checkRandomLoops( keyframes, currFrame, globalOptimizer );
            }
            keyframes.push_back( currFrame );
            
            break;
        default:
            break;
        }
    }
    // 优化
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./result_after.g2o" );
    cout<<"Optimization done."<<endl;
    createPointCloud(keyframes,globalOptimizer,camera);
    return 0;
}
