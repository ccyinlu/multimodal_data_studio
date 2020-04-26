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
    const size_t *dimArrayOfSe3 = mxGetDimensions(prhs[0]);
    size_t sizeRowsSe3 = *(dimArrayOfSe3 + 0);
    size_t sizeColsSe3 = *(dimArrayOfSe3 + 1);
    if(sizeRowsSe3 != 6 || sizeColsSe3 != 1){
        mexErrMsgIdAndTxt( "EdgeSE3ProjectDirectWithDirstortJacobian:invalidInputs", "the 1st param should be 6x1");
        return;
    }
    double *ptrSe3 = (double *)(mxGetPr(prhs[0]));
    Eigen::Matrix<double, 6, 1> se3;
    for(int i = 0; i < 6; i++){
        se3(i, 0) = *(ptrSe3 + i);
    }

    Sophus::SE3<double> SE3 = Sophus::SE3<double>::exp(se3);
    Eigen::Matrix<double, 4, 4> SE3Matrix = SE3.matrix();

    // cout<<"SE3 updated = "<<endl<<SE3Matrix<<endl;

    // the eulerTransform will be 4x4
    size_t dimArrayOfEulerTransform[2] = { 4, 4 };
    plhs[0] = mxCreateNumericArray(2, dimArrayOfEulerTransform, mxDOUBLE_CLASS, mxREAL);
    double *ptrEulerTransform = (double *)mxGetData(plhs[0]);
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            ptrEulerTransform[i * 4 + j] = SE3Matrix(j, i);
        }
    }
}