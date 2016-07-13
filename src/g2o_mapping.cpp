#include "g2o_mapping.h"

namespace plane_slam
{

G2OMapping::G2OMapping()
{
    // 选择优化方法
    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverCSparse< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

     // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

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

    int lastIndex = currIndex; // 上一帧的id
}

bool G2OMapping::mapping( const Frame &frame )
{

}

bool G2OMapping::doMapping( const Frame &frame )
{

}

bool G2OMapping::addFirstFrame( const Frame &frame )
{

}

bool G2OMapping::isKeyFrame( const Frame &frame )
{
    tf::Transform rel_tf = last_estimated_pose_tf_.inverse() * frame.pose_;
    double rad, distance;
    calAngleAndDistance( rel_tf, rad, distance );

    if( distance > keyframe_linear_threshold_ || rad > keyframe_angular_threshold_ )
        return true;
    else
        return false;
}

} // end of namespace plane_slam
