#include<stdio.h>
#include"mex.h"

#include "EdgeSE3ProjectDirectWithDirstortG2oLM.cpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
 
// 李群李代数 库 
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace g2o;

// 一次测量的值，包括一个世界坐标系下三维点,以及投影的对应的图像
struct Measurement
{
    Measurement ( Eigen::Vector3d p, float * im) : pos_world (p), image(im){}
    Eigen::Vector3d pos_world;
    float* image;
};

bool calibrationEstimationDirectG2OLM ( const vector<Measurement>& measurements, double *D, double *K, double *imageSize, Eigen::Isometry3d& Tcw, bool verbose, int max_iter)
{
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( verbose );

    // 添加顶点
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( g2o::SE3Quat ( Tcw.rotation(), Tcw.translation() ) );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    // 添加边
    int id=1;
    for ( Measurement m: measurements )
    {
        EdgeSE3ProjectDirectWithDirstortG2oLM* edge = new EdgeSE3ProjectDirectWithDirstortG2oLM (
            m.pos_world,
            D, K, imageSize, m.image
        );
        edge->setVertex (0, pose );
        edge->setMeasurement (0);
        edge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );
        edge->setId ( id++ );
        optimizer.addEdge ( edge );
    }
    // cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
    mexPrintf("edges in graph: %d\n", optimizer.edges().size());
    optimizer.initializeOptimization();
    optimizer.optimize ( max_iter );
    Tcw = pose->estimate();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    // nlhs represent the number of parameters of the output
    // plhs is a array of the mxarray pointers, each pointing to the output
    // nrhs represents the number of parameters of the input
    // prhs is a array of the mxarray pointers, each pointing to the input

    // prhs[0], 6x1 matrix
    // prhs[1], Mx1 cell, each cell with NX3 points
    // prhs[2], Mx1 cell, each cell with PXQ single matrix
    // prhs[3], 1x2, or 1x3, or 1x4, or 1x5 matrix
    // prhs[4], 3x3 matrix
    // prhs[5], 1x2 matrix
    // prhs[6], 1x1 matrix
    // prhs[7], 1x1 matrix

    if(nrhs < 8){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidNumInputs", "at least 8 input arguments required");
        return;
    }

    // get the init xi
    const size_t *dimArrayOfInitXi = mxGetDimensions(prhs[0]);
    size_t sizeRowsInitXi = *(dimArrayOfInitXi + 0);
    size_t sizeColsInitXi = *(dimArrayOfInitXi + 1);
    if(sizeRowsInitXi != 6 || sizeColsInitXi != 1){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 1st param should be 6x1");
        return;
    }
    double *ptrInitXi = (double *)(mxGetPr(prhs[0]));
    Eigen::Matrix<double, 6, 1> initXi;
    for(int i = 0; i < 6; i++){
        initXi(i, 0) = *(ptrInitXi + i);
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the chessboard lidar points
    const size_t *dimArrayOfChessboardLidarPoints = mxGetDimensions(prhs[1]);
    size_t sizeRowsCellChessboardLidarPoints = *(dimArrayOfChessboardLidarPoints + 0);
    size_t sizeColsCellChessboardLidarPoints = *(dimArrayOfChessboardLidarPoints + 1);
    if(sizeColsCellChessboardLidarPoints != 1){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 2st param should be Mx1 cell");
        return;
    }
    std::vector<std::vector<Eigen::Vector3d>>chessboardLidarPoints; 
    size_t total_measurements = 0;
    for(int i = 0; i < sizeRowsCellChessboardLidarPoints; i++){
        // get the dimensions of each cell
        mxArray *cur_mArray_chessboardLidarPoints = mxGetCell(prhs[1], i);
        const size_t *dimArrayOfCurrentChessboardLidarPoints = mxGetDimensions(cur_mArray_chessboardLidarPoints);
        size_t sizeRowsCurrentChessboardLidarPoints = *(dimArrayOfCurrentChessboardLidarPoints + 0);
        size_t sizeColsCurrentChessboardLidarPoints = *(dimArrayOfCurrentChessboardLidarPoints + 1);
        if(sizeColsCurrentChessboardLidarPoints != 3){
            mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 2st param should be cell with Px3");
            return;
        }
        float *cur_ptr_chessboardLidarPoints = (float *)(mxGetPr(cur_mArray_chessboardLidarPoints));
        std::vector<Eigen::Vector3d>currentCellChessboardLidarPoints;
        total_measurements = total_measurements + sizeRowsCurrentChessboardLidarPoints;
        for(int j = 0; j < sizeRowsCurrentChessboardLidarPoints; j++){
            Eigen::Vector3d current_chessboardLidarPoint;
            current_chessboardLidarPoint[0] = cur_ptr_chessboardLidarPoints[0 * sizeRowsCurrentChessboardLidarPoints + j];
            current_chessboardLidarPoint[1] = cur_ptr_chessboardLidarPoints[1 * sizeRowsCurrentChessboardLidarPoints + j];
            current_chessboardLidarPoint[2] = cur_ptr_chessboardLidarPoints[2 * sizeRowsCurrentChessboardLidarPoints + j];
            currentCellChessboardLidarPoints.push_back(current_chessboardLidarPoint);
        }
        chessboardLidarPoints.push_back(currentCellChessboardLidarPoints);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the chessoard_Dt_Mask
    const size_t *dimArrayOfChessboardMaskDT = mxGetDimensions(prhs[2]);
    size_t sizeRowsCellChessboardMaskDT = *(dimArrayOfChessboardMaskDT + 0);
    size_t sizeColsCellChessboardMaskDT = *(dimArrayOfChessboardMaskDT + 1);
    if(sizeColsCellChessboardMaskDT != 1){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 3st param should be Mx1 cell");
        return;
    }
    std::vector<float *>chessboardMaskDTPtr;
    for(int i = 0; i < sizeRowsCellChessboardMaskDT; i++){
        // get the dimensions of each cell
        mxArray *cur_mArray_chessboardMaskDT = mxGetCell(prhs[2], i);

        float *cur_ptr_chessboardMaskDT = (float *)(mxGetPr(cur_mArray_chessboardMaskDT));
        chessboardMaskDTPtr.push_back(cur_ptr_chessboardMaskDT);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the distortion parameters
    const size_t *dimArrayOfD = mxGetDimensions(prhs[3]);
    size_t sizeRowsD = *(dimArrayOfD + 0);
    size_t sizeColsD = *(dimArrayOfD + 1);
    if(sizeRowsD != 1 || sizeColsD != 5){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 4st param should be 1x5 array");
        return;
    }
    double *ptrD = (double *)(mxGetPr(prhs[3]));

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the intrinsic parameters K
    const size_t *dimArrayOfK = mxGetDimensions(prhs[4]);
    size_t sizeRowsK = *(dimArrayOfK + 0);
    size_t sizeColsK = *(dimArrayOfK + 1);
    if(sizeRowsK != 3 || sizeColsK != 3){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 5st param should be 3X3 array");
        return;
    }
    double *ptrK = (double *)(mxGetPr(prhs[4]));

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the imageSize
    const size_t *dimArrayOfImageSize = mxGetDimensions(prhs[5]);
    size_t sizeRowsImageSize = *(dimArrayOfImageSize + 0);
    size_t sizeColsImageSize = *(dimArrayOfImageSize + 1);
    if(sizeRowsImageSize != 1 || sizeColsImageSize != 2){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 6st param should be 1x2 array");
        return;
    }
    double *ptrImageSize = (double *)(mxGetPr(prhs[5]));

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the verbose params
    const size_t *dimArrayOfVerbose = mxGetDimensions(prhs[6]);
    size_t sizeRowsVerbose= *(dimArrayOfVerbose + 0);
    size_t sizeColsVerbose = *(dimArrayOfVerbose + 1);
    if(sizeRowsVerbose != 1 || sizeColsVerbose != 1){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 7st param should be 1x1 array");
        return;
    }
    double *ptrVerbose = (double *)(mxGetPr(prhs[6]));

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the maxIter params
    const size_t *dimArrayOfMaxIter = mxGetDimensions(prhs[6]);
    size_t sizeRowsMaxIter = *(dimArrayOfMaxIter + 0);
    size_t sizeColsMaxIter = *(dimArrayOfMaxIter + 1);
    if(sizeRowsMaxIter != 1 || sizeColsMaxIter != 1){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 8st param should be 1x1 array");
        return;
    }
    double *ptrMaxIter = (double *)(mxGetPr(prhs[7]));

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<Measurement> measurements;
    std::vector<bool> isInners;
    Eigen::Isometry3d Tlc = Eigen::Isometry3d::Identity();

    // get the init tcl from the init Xi
    Sophus::SE3<double> SE3 = Sophus::SE3<double>::exp(initXi);
    Eigen::Matrix<double, 4, 4> SE3Matrix = SE3.matrix();
    Tlc.matrix() << SE3Matrix;

    for(int i = 0; i < chessboardMaskDTPtr.size(); i++){
        float *current_chessboardMaskDTPtr = chessboardMaskDTPtr[i];
        std::vector<Eigen::Vector3d> current_chessboardLidarPoints = chessboardLidarPoints[i];
        for(int j = 0; j < current_chessboardLidarPoints.size(); j++){
            measurements.push_back(Measurement(current_chessboardLidarPoints[j], current_chessboardMaskDTPtr));
        }
    }
    isInners.resize(measurements.size());

    calibrationEstimationDirectG2OLM ( measurements, ptrD, ptrK, ptrImageSize, Tlc, bool(ptrVerbose[0]), int(ptrMaxIter[0]));

    Sophus::SE3<double> SE3_Rt(Tlc.rotation(), Tlc.translation());

    // cout<<"SE3 = "<<endl<<SE3_Rt.matrix()<<endl;

    Eigen::Matrix<double,6,1> se3_out = SE3_Rt.log();

    // the output xi will be 1x6
    size_t dimArrayOfXi[2] = { 1, 6 };
    plhs[0] = mxCreateNumericArray(2, dimArrayOfXi, mxDOUBLE_CLASS, mxREAL);
    double *out_xi = (double *)mxGetData(plhs[0]);

    for(int i = 0; i < 6; i++){
        out_xi[i] = se3_out(i,0);
    }
}