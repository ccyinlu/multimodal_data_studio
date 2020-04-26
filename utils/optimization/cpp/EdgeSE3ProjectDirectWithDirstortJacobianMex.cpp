#include<stdio.h>
#include"mex.h"

#include "EdgeSE3ProjectDirectWithDirstortJacobian.cpp"

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
        #ifdef MEASUREMENT_PRE_POINT
        total_measurements = total_measurements + sizeRowsCurrentChessboardLidarPoints;
        #elif MEASUREMENT_PRE_PAIR
        total_measurements = total_measurements + 1;
        #endif
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


#ifdef MEASUREMENT_PRE_POINT
    // the average_jacobian will be mx6
    size_t dimArrayOfAverageJacobian[2] = { total_measurements, 6 };
    plhs[0] = mxCreateNumericArray(2, dimArrayOfAverageJacobian, mxDOUBLE_CLASS, mxREAL);
    double *_jacobian_output = (double *)mxGetData(plhs[0]);

    // calc the sum of the jacobian
    std::vector<Eigen::Matrix<double, 1, 6>>_jacobian;
    int measurements_counter = 0;
    for(int i = 0; i < chessboardMaskDTPtr.size(); i++){
        float *current_chessboardMaskDTPtr = chessboardMaskDTPtr[i];
        std::vector<Eigen::Vector3d> current_chessboardLidarPoints = chessboardLidarPoints[i];
        EdgeSE3ProjectDirectWithDistortJacobian mEdgeSE3ProjectDirectWithDistortJacobian = EdgeSE3ProjectDirectWithDistortJacobian(current_chessboardLidarPoints, \
                                                                                            ptrD, \
                                                                                            ptrK, \
                                                                                            ptrImageSize, \
                                                                                            current_chessboardMaskDTPtr, \
                                                                                            initXi);
        _jacobian = mEdgeSE3ProjectDirectWithDistortJacobian.jacobians_new();
        for(int j = 0; j < _jacobian.size(); j++){
            Eigen::Matrix<double, 1, 6> current_jacobian = _jacobian[j];
            for(int k = 0; k < 6; k++){
                _jacobian_output[ k * total_measurements + measurements_counter] = current_jacobian(0, k);
            }
            measurements_counter++;
        }
    }
#elif MEASUREMENT_PRE_PAIR
    // the average_jacobian will be mx6
    size_t dimArrayOfAverageJacobian[2] = { total_measurements, 6 };
    plhs[0] = mxCreateNumericArray(2, dimArrayOfAverageJacobian, mxDOUBLE_CLASS, mxREAL);
    double *_jacobian_output = (double *)mxGetData(plhs[0]);

    // calc the sum of the jacobian
    Eigen::Matrix<double, 1, 6>_jacobian;
    int measurements_counter = 0;
    for(int i = 0; i < chessboardMaskDTPtr.size(); i++){
        float *current_chessboardMaskDTPtr = chessboardMaskDTPtr[i];
        std::vector<Eigen::Vector3d> current_chessboardLidarPoints = chessboardLidarPoints[i];
        EdgeSE3ProjectDirectWithDistortJacobian mEdgeSE3ProjectDirectWithDistortJacobian = EdgeSE3ProjectDirectWithDistortJacobian(current_chessboardLidarPoints, \
                                                                                            ptrD, \
                                                                                            ptrK, \
                                                                                            ptrImageSize, \
                                                                                            current_chessboardMaskDTPtr, \
                                                                                            initXi);
        _jacobian = mEdgeSE3ProjectDirectWithDistortJacobian.jacobians();
        for(int k = 0; k < 6; k++){
            _jacobian_output[ k * total_measurements + measurements_counter] = _jacobian(0, k);
        }
        measurements_counter++;
    }
#endif
}