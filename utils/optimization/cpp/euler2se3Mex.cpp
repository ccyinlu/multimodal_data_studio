#include <iostream>
#include <cmath>
using namespace std; 
#include <Eigen/Core>
#include <Eigen/Geometry>
 
// 李群李代数 库 
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

#include<stdio.h>
#include"mex.h"

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

    if(nrhs < 1){
        mexErrMsgIdAndTxt( "euler2se3Mex:invalidNumInputs", "at least 1 input arguments required");
        return;
    }

    // get the euler transformation
    const size_t *dimArrayOfEulerTransformation = mxGetDimensions(prhs[0]);
    size_t sizeRowsEulerTransformation = *(dimArrayOfEulerTransformation + 0);
    size_t sizeColsEulerTransformation = *(dimArrayOfEulerTransformation + 1);
    if(sizeRowsEulerTransformation != 6 || sizeColsEulerTransformation != 1){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 1st param should be 6x1");
        return;
    }
    double *ptrEulerTransformation = (double *)(mxGetPr(prhs[0]));
    Eigen::Matrix<double, 6, 1> eulerTransformation;
    for(int i = 0; i < 6; i++){
        eulerTransformation(i, 0) = *(ptrEulerTransformation + i);
    }

    // cout<<"eulerTransformation = "<<endl<<eulerTransformation<<endl;

    Eigen::Matrix<double,6,1> se3;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_Z_vector ( eulerTransformation(0,0), Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 yaw
    Eigen::AngleAxisd rotation_Y_vector ( eulerTransformation(1,0), Eigen::Vector3d ( 0,1,0 ) );      //沿 Y 轴旋转 pitch
    Eigen::AngleAxisd rotation_X_vector ( eulerTransformation(2,0), Eigen::Vector3d ( 1,0,0 ) );      //沿 X 轴旋转 roll
    R = rotation_Z_vector.toRotationMatrix() * rotation_Y_vector.toRotationMatrix() * rotation_X_vector.toRotationMatrix();

    Eigen::Vector3d t(eulerTransformation(3,0), eulerTransformation(4,0), eulerTransformation(5,0));
    Sophus::SE3<double> SE3_Rt(R, t);           // 从R,t构造SE(3)

    // cout<<"SE3 = "<<endl<<SE3_Rt.matrix()<<endl;

    se3 = SE3_Rt.log();

    // cout<<"se3 = "<<endl<<se3.matrix()<<endl;

    // the se3 will be 6x1
    size_t dimArrayOfSe3[2] = { 6, 1 };
    plhs[0] = mxCreateNumericArray(2, dimArrayOfSe3, mxDOUBLE_CLASS, mxREAL);
    double *ptrSe3 = (double *)mxGetData(plhs[0]);
    for(int i = 0; i < 6; i++){
        ptrSe3[i] = se3(i, 0);
    }
}