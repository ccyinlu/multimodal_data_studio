#include<stdio.h>
#include"mex.h"

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

double reProjectError(Eigen::Matrix<double, 6, 1> initXi, Eigen::Vector3d x_world_, float* maskDT_, double k1_, double k2_, double k3_, double p1_, double p2_, double fx_, double fy_, double cx_, double cy_, int width, int height);
float getPixelValue ( int x, int y, float* maskDT_, int image_width_, int image_height_);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    // nlhs represent the number of parameters of the output
    // plhs is a array of the mxarray pointers, each pointing to the output
    // nrhs represents the number of parameters of the input
    // prhs is a array of the mxarray pointers, each pointing to the input

    // prhs[0], 6x1 matrix
    // prhs[1], Mx1 cell, each cell with NX3 points, with single matrix
    // prhs[2], Mx1 cell, each cell with PXQ single matrix
    // prhs[3], 1x2, or 1x3, or 1x4, or 1x5 matrix
    // prhs[4], 3x3 matrix
    // prhs[5], 1x2 matrix

    if(nrhs < 6){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidNumInputs", "at least 6 input arguments required");
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
    std::vector<float *>ptr_chessboardLidarPoints;
    std::vector<size_t>num_chessboardLidarPoints;
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
        ptr_chessboardLidarPoints.push_back(cur_ptr_chessboardLidarPoints);
        num_chessboardLidarPoints.push_back(sizeRowsCurrentChessboardLidarPoints);
        #ifdef MEASUREMENT_PRE_POINT
        total_measurements = total_measurements + sizeRowsCurrentChessboardLidarPoints;
        #elif MEASUREMENT_PRE_PAIR
        total_measurements = total_measurements + 1;
        #endif
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

    size_t dimArrayOfError[2] = {total_measurements, 1};
    plhs[0] = mxCreateNumericArray(2, dimArrayOfError, mxDOUBLE_CLASS, mxREAL);
    double *error_output = (double *)mxGetData(plhs[0]);

    #ifdef MEASUREMENT_PRE_POINT
    size_t measurement_counter = 0;
    for(int i = 0; i < num_chessboardLidarPoints.size(); i++){
        size_t cur_num_chessboardLidarPoints = num_chessboardLidarPoints[i];
        float *cur_chessboard_points = ptr_chessboardLidarPoints[i];
        float *cur_chessboard_mask_dt = chessboardMaskDTPtr[i];
        for(int j = 0; j < cur_num_chessboardLidarPoints; j++){
            Eigen::Vector3d current_chessboardLidarPoint;
            current_chessboardLidarPoint[0] = cur_chessboard_points[0 * cur_num_chessboardLidarPoints + j];
            current_chessboardLidarPoint[1] = cur_chessboard_points[1 * cur_num_chessboardLidarPoints + j];
            current_chessboardLidarPoint[2] = cur_chessboard_points[2 * cur_num_chessboardLidarPoints + j];

            double error_ = reProjectError(initXi, \
                            current_chessboardLidarPoint, \
                            cur_chessboard_mask_dt, \
                            ptrD[0], \
                            ptrD[1], \
                            ptrD[2], \
                            ptrD[3], \
                            ptrD[4], \
                            ptrK[0], \
                            ptrK[4], \
                            ptrK[6], \
                            ptrK[7], \
                            floor(ptrImageSize[0]), floor(ptrImageSize[1]));
            error_output[measurement_counter++] = error_;
        }
    }
    #elif MEASUREMENT_PRE_PAIR
    size_t measurement_counter = 0;
    for(int i = 0; i < num_chessboardLidarPoints.size(); i++){
        size_t cur_num_chessboardLidarPoints = num_chessboardLidarPoints[i];
        float *cur_chessboard_points = ptr_chessboardLidarPoints[i];
        float *cur_chessboard_mask_dt = chessboardMaskDTPtr[i];
        double error_chessboard = 0;
        for(int j = 0; j < cur_num_chessboardLidarPoints; j++){
            Eigen::Vector3d current_chessboardLidarPoint;
            current_chessboardLidarPoint[0] = cur_chessboard_points[0 * cur_num_chessboardLidarPoints + j];
            current_chessboardLidarPoint[1] = cur_chessboard_points[1 * cur_num_chessboardLidarPoints + j];
            current_chessboardLidarPoint[2] = cur_chessboard_points[2 * cur_num_chessboardLidarPoints + j];

            double error_ = reProjectError(initXi, \
                            current_chessboardLidarPoint, \
                            cur_chessboard_mask_dt, \
                            ptrD[0], \
                            ptrD[1], \
                            ptrD[2], \
                            ptrD[3], \
                            ptrD[4], \
                            ptrK[0], \
                            ptrK[4], \
                            ptrK[6], \
                            ptrK[7], \
                            floor(ptrImageSize[0]), floor(ptrImageSize[1]));
            error_chessboard = error_chessboard + error_;
        }
        error_output[measurement_counter++] = error_chessboard/cur_num_chessboardLidarPoints;
    }
    #endif
}

double reProjectError(Eigen::Matrix<double, 6, 1> initXi, Eigen::Vector3d x_world_, float* maskDT_, double k1_, double k2_, double k3_, double p1_, double p2_, double fx_, double fy_, double cx_, double cy_, int width, int height){
    double error_ = 0;
    Eigen::Matrix<double, 1, 6>_jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();

    Eigen::Matrix<double, 4, 4>SE3Matrix = Sophus::SE3<double>::exp(initXi).matrix();
    Eigen::Vector3d xyz_trans = SE3Matrix.block<3,3>(0,0) * x_world_ + SE3Matrix.block<3,1>(0,3);

    //归一化
    float x = xyz_trans(0)/xyz_trans(2);
    float y = xyz_trans(1)/xyz_trans(2);

    //畸变
    float r2 = (x*x + y*y);
    float a = (1 + k1_*r2 + k2_*pow(r2,2) + k3_*pow(r2,3));
    float xx = a*x + 2*p1_*x*y + p2_*(r2 + 2*x*x);
    float yy = a*y + p1_*(r2+2*y*y) + 2*p2_*x*y;

    //内参变化
    float u = fx_ * xx + cx_;
    float v = fy_ * yy + cy_;

    // check x,y is in the image
    if ( u<0 || ( u ) >=width || ( v) <0 || ( v ) >=height )
    {
        error_ = 10000.0;
    }
    else
    {
        error_ = getPixelValue ( floor(u), floor(v), maskDT_, width, height);
    }
    return error_;
}

float getPixelValue ( int x, int y, float* maskDT_, int image_width_, int image_height_)
{
    if(x < 0 )
        x = 0;
    else if (x > image_width_ - 1)
    {
        x = image_width_ - 1;
    }

    if(y < 0 )
        y = 0;
    else if (y > image_height_ - 1)
    {
        y = image_height_ - 1;
    }

    return maskDT_[x * image_height_ + y];
}