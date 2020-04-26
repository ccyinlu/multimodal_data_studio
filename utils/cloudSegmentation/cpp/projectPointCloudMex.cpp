#include<stdio.h>
#include"mex.h"

#include <string.h>
#include <math.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //nlhs represent the number of parameters of the output
    //plhs is a array of the mxarray pointers, each pointing to the output
    //nrhs represents the number of parameters of the input
    //prhs is a array of the mxarray pointers, each pointing to the input

    // the first matrix represent the raw pointCloud: Nx4, | x | y | z | intensity |
    /* check proper input and output */
    #ifdef DEBUG
        mexPrintf("=========================================================================\n");
    #endif
    
    if(nrhs < 2)
        mexErrMsgIdAndTxt( "projectPointCloud:invalidNumInputs",
                "at least two input arguments required");
    else if(!mxIsStruct(prhs[0]))
        mexErrMsgIdAndTxt( "projectPointCloud:inputNotStruct",
                "the first input should be a struct");

    if(mxGetNumberOfDimensions(prhs[1]) != 2){
        mexPrintf("%s%d\n", "raw pointCloud matrix dimensions: ", mxGetNumberOfDimensions(prhs[1]));
        mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
                "raw pointCloud matrix dimensions not equal to 2, exit!");
    }
    const size_t *dimArrayOfRawPointCloud = mxGetDimensions(prhs[1]);
    double *ptrRawPointCloud = mxGetPr(prhs[1]);
    size_t numOfPoints = *dimArrayOfRawPointCloud;
    size_t numOfCols = *(dimArrayOfRawPointCloud + 1);

    mxArray * fPtr;
    int N_SCAN = 40;
    int Horizon_SCAN = 1800;
    double * vertical_theta;
    size_t numOfVerticalTheta;
    // char * lidar_type;

    // get the parameter : N_SCAN
    fPtr = mxGetField(prhs[0], 0, "N_SCAN");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        N_SCAN = int(realPtr[0]);
        if(N_SCAN < 1 || N_SCAN > 400){
            mexPrintf("%s%d\n", "N_SCAN: ", N_SCAN);
            mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
                    "N_SCAN must be within the range of 1 ~ 400");
        }
        #ifdef DEBUG
            mexPrintf("%s%d\n", "N_SCAN: ", N_SCAN);
        #endif
    }else{
        mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
            "field N_SCAN not found in the params struct");
    }

    // get the parameter : Horizon_SCAN
    fPtr = mxGetField(prhs[0], 0, "Horizon_SCAN");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        Horizon_SCAN = int(realPtr[0]);
        if(Horizon_SCAN < 1 || Horizon_SCAN > 6400){
            mexPrintf("%s%d\n", "Horizon_SCAN: ", Horizon_SCAN);
            mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
                    "Horizon_SCAN must be within the range of 1 ~ 6400");
        }
        #ifdef DEBUG
            mexPrintf("%s%d\n", "Horizon_SCAN: ", Horizon_SCAN);
        #endif
    }else{
        mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
            "field Horizon_SCAN not found in the params struct");
    }

    // get the parameter : vertical_theta
    fPtr = mxGetField(prhs[0], 0, "vertical_theta");
    if ((fPtr != NULL)){
        const size_t *dimArrayOfVeritcalTheta = mxGetDimensions(fPtr);
        numOfVerticalTheta = *(dimArrayOfVeritcalTheta + 1);
        vertical_theta = mxGetPr(fPtr);
        if(numOfVerticalTheta != N_SCAN){
            mexPrintf("%s%d\n", "numOfVerticalTheta: ", numOfVerticalTheta);
            mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
                    "numOfVerticalTheta must be equal with N_SCAN");
        }
        #ifdef DEBUG
            mexPrintf("%s%d\n", "numOfVerticalTheta: ", numOfVerticalTheta);
            mexPrintf("%s\n", "vertical_theta: ");
            for(int i = 0; i < numOfVerticalTheta; i++){
                mexPrintf("%.3f\t", vertical_theta[i]);
                if((i+1)%8 == 0){
                    mexPrintf("\n");
                }
            }
        #endif
    }else{
        mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
            "field vertical_theta not found in the params struct");
    }

    // get the parameter : lidar_type
    // fPtr = mxGetField(prhs[0], 0, "lidar_type");
    // if ((fPtr != NULL)){
    //     double *realPtr = mxGetPr(fPtr);
    //     lidar_type = mxArrayToString(fPtr);
    //     if(!(strcmp(lidar_type, "hesai_p40p") == 0) && \
    //        !(strcmp(lidar_type, "velodyne_hdl_64e") == 0) && \
    //         !(strcmp(lidar_type, "prescan_p40p") == 0)){
    //         mexPrintf("%s%s\n", "lidar_type: ", lidar_type);
    //         mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
    //                 "lidar_type not supported");
    //     }
    //     #ifdef DEBUG
    //         mexPrintf("%s%s\n", "lidar_type: ", lidar_type);
    //     #endif
    // }else{
    //     mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
    //         "field lidar_type not found in the params struct");
    // }

    #ifdef DEBUG
        mexPrintf("%s%d\t%s%d\t%s%d\n", "number of scans: ", N_SCAN, "number of cols: ", Horizon_SCAN, "number of points: ", numOfPoints);
    #endif

    // the rangeMat will be MxNx6
    size_t dimArrayOfRangeMat[3] = { (size_t)N_SCAN, (size_t)Horizon_SCAN, 6 };
    plhs[0] = mxCreateNumericArray(3, dimArrayOfRangeMat, mxDOUBLE_CLASS, mxREAL);
    double *rangeMat = (double *)mxGetData(plhs[0]);

    double *rangeMatFilled;
    if(nlhs > 1){
        // rangeMatFilled
        plhs[1] = mxCreateNumericArray(3, dimArrayOfRangeMat, mxDOUBLE_CLASS, mxREAL);
        rangeMatFilled = (double *)mxGetData(plhs[1]);
    }

    double *rangeMatRaw;
    if(nlhs > 2){
        // rangeMatRaw
        plhs[2] = mxCreateNumericArray(3, dimArrayOfRangeMat, mxDOUBLE_CLASS, mxREAL);
        rangeMatRaw = (double *)mxGetData(plhs[2]);
    }

    double *v_angle_raw;
    if(nlhs > 3){
        // v_angle_raw
        size_t dimArrayOf_v_angle_raw[1] = { numOfPoints};
        plhs[3] = mxCreateNumericArray(1, dimArrayOf_v_angle_raw, mxDOUBLE_CLASS, mxREAL);
        v_angle_raw = (double *)mxGetData(plhs[3]);
    }

    // init the output
    for(int j = 0; j < Horizon_SCAN; j ++){
        for(int i = 0; i < N_SCAN; i ++){
            // init the valid_label of the rangeMat to zeros
            rangeMat[(Horizon_SCAN * N_SCAN) * 0 + j * N_SCAN + i] = 0;
            // init the range of the rangeMat to Nan
            rangeMat[(Horizon_SCAN * N_SCAN) * 1 + j * N_SCAN + i] = mxGetNaN();
            // init the x of the rangeMat to Nan
            rangeMat[(Horizon_SCAN * N_SCAN) * 2 + j * N_SCAN + i] = mxGetNaN();
            // init the y of the rangeMat to Nan
            rangeMat[(Horizon_SCAN * N_SCAN) * 3 + j * N_SCAN + i] = mxGetNaN();
            // init the z of the rangeMat to Nan
            rangeMat[(Horizon_SCAN * N_SCAN) * 4 + j * N_SCAN + i] = mxGetNaN();
            // init the intensity of the rangeMat to Nan
            rangeMat[(Horizon_SCAN * N_SCAN) * 5 + j * N_SCAN + i] = mxGetNaN();

            if(nlhs > 2){
                // init the valid_label of the rangeMatRaw to zeros
                rangeMatRaw[(Horizon_SCAN * N_SCAN) * 0 + j * N_SCAN + i] = 0;
                // init the range of the rangeMatRaw to Nan
                rangeMatRaw[(Horizon_SCAN * N_SCAN) * 1 + j * N_SCAN + i] = mxGetNaN();
                // init the x of the rangeMatRaw to Nan
                rangeMatRaw[(Horizon_SCAN * N_SCAN) * 2 + j * N_SCAN + i] = mxGetNaN();
                // init the y of the rangeMatRaw to Nan
                rangeMatRaw[(Horizon_SCAN * N_SCAN) * 3 + j * N_SCAN + i] = mxGetNaN();
                // init the z of the rangeMatRaw to Nan
                rangeMatRaw[(Horizon_SCAN * N_SCAN) * 4 + j * N_SCAN + i] = mxGetNaN();
                // init the intensity of the rangeMatRaw to Nan
                rangeMatRaw[(Horizon_SCAN * N_SCAN) * 5 + j * N_SCAN + i] = mxGetNaN();
            }
        }
    }

    // generate the horizontal_theta
    double * horizonal_theta = new double[Horizon_SCAN];
    double horizontal_theta_step = 2 * M_PI / Horizon_SCAN;
    for(int i = 0; i < Horizon_SCAN; i++){
        horizonal_theta[i] = -M_PI + i * horizontal_theta_step;
    }

    double current_x, current_y, current_z, current_i, v_angle, h_angle;
    int h_index, v_index, v_index_raw;
    double v_angle_res, v_angle_res_min;

    for(int i = 0; i < numOfPoints; i++){
        // calc the v_angle and h_angle for the current point
        current_x = ptrRawPointCloud[(numOfPoints * 0) + i];
        current_y = ptrRawPointCloud[(numOfPoints * 1) + i];
        current_z = ptrRawPointCloud[(numOfPoints * 2) + i];
        current_i = ptrRawPointCloud[(numOfPoints * 3) + i];
        v_angle = atan2(current_z, sqrt(current_x*current_x + current_y*current_y));

        v_angle_raw[i] = v_angle;

        // if(strcmp(lidar_type, "hesai_p40p") == 0){
        //     h_angle = -atan2(current_x, current_y);
        // }else if(strcmp(lidar_type, "velodyne_hdl_64e") == 0){
        //     h_angle = atan2(-current_y, current_x);
        // }else if(strcmp(lidar_type, "prescan_p40p") == 0){
        //     h_angle = -atan2(current_x, current_y);
        // }else{
        //     mexPrintf("%s%s\n", "lidar_type: ", lidar_type);
        //     mexErrMsgIdAndTxt( "projectPointCloud:invalidField",
        //             "lidar_type not supported");
        // }

        h_angle = atan2(current_y, current_x);

        h_index = int((h_angle + M_PI)/(2*M_PI)*Horizon_SCAN);
        if(h_index < 0){
            h_index = 0;
        }else if(h_index > Horizon_SCAN - 1){
            h_index = Horizon_SCAN - 1;
        }

        v_index = 0;
        v_angle_res_min = 1000000;
        for(int j = 0; j < N_SCAN; j++){
            v_angle_res = fabs(v_angle - vertical_theta[j]);
            if(v_angle_res < v_angle_res_min){
                v_angle_res_min = v_angle_res;
                v_index = j;
            }else{
                break;
            }
        }

        if(nlhs > 2){
            v_index_raw = 0;
            if(v_angle > vertical_theta[0]){
                v_index_raw = 0;
            }else if (v_angle < vertical_theta[N_SCAN - 1])
            {
                v_index_raw = N_SCAN - 1;
            }else{
                v_index_raw = int((vertical_theta[0] - v_angle)/(vertical_theta[0] - vertical_theta[N_SCAN - 1])*(N_SCAN - 1));
            }
        }

        double range = sqrt(current_x * current_x + current_y * current_y + current_z * current_z);
        // if(range < 0.1){
        //     continue;
        // }

        #ifdef DEBUG
            // mexPrintf("%s%f\t%s%f\t%s%f\t%s%f\t%s%d\t%s%d\n", "x: ", current_x, "y: ", current_y, "z: ", current_z, "i: ", current_i, "h: ", h_index, "v: ", v_index);
        #endif

        // update the current h_index and v_index data
        rangeMat[(Horizon_SCAN * N_SCAN) * 0 + h_index * N_SCAN + v_index] = 1;
        rangeMat[(Horizon_SCAN * N_SCAN) * 1 + h_index * N_SCAN + v_index] = range;
        rangeMat[(Horizon_SCAN * N_SCAN) * 2 + h_index * N_SCAN + v_index] = current_x;
        rangeMat[(Horizon_SCAN * N_SCAN) * 3 + h_index * N_SCAN + v_index] = current_y;
        rangeMat[(Horizon_SCAN * N_SCAN) * 4 + h_index * N_SCAN + v_index] = current_z;
        rangeMat[(Horizon_SCAN * N_SCAN) * 5 + h_index * N_SCAN + v_index] = current_i;

        if(nlhs > 2){
            // update the current h_index and v_index data
            rangeMatRaw[(Horizon_SCAN * N_SCAN) * 0 + h_index * N_SCAN + v_index_raw] = 1;
            rangeMatRaw[(Horizon_SCAN * N_SCAN) * 1 + h_index * N_SCAN + v_index_raw] = range;
            rangeMatRaw[(Horizon_SCAN * N_SCAN) * 2 + h_index * N_SCAN + v_index_raw] = current_x;
            rangeMatRaw[(Horizon_SCAN * N_SCAN) * 3 + h_index * N_SCAN + v_index_raw] = current_y;
            rangeMatRaw[(Horizon_SCAN * N_SCAN) * 4 + h_index * N_SCAN + v_index_raw] = current_z;
            rangeMatRaw[(Horizon_SCAN * N_SCAN) * 5 + h_index * N_SCAN + v_index_raw] = current_i;
        }
    }

    if(nlhs > 1){
        // init the output
        for(int j = 0; j < Horizon_SCAN; j ++){
            for(int i = 0; i < N_SCAN; i ++){
                // init the valid_label of the rangeMat to zeros
                rangeMatFilled[(Horizon_SCAN * N_SCAN) * 0 + j * N_SCAN + i] = 0;
                // init the range of the rangeMat to Nan
                rangeMatFilled[(Horizon_SCAN * N_SCAN) * 1 + j * N_SCAN + i] = mxGetNaN();
                // init the x of the rangeMat to Nan
                rangeMatFilled[(Horizon_SCAN * N_SCAN) * 2 + j * N_SCAN + i] = mxGetNaN();
                // init the y of the rangeMat to Nan
                rangeMatFilled[(Horizon_SCAN * N_SCAN) * 3 + j * N_SCAN + i] = mxGetNaN();
                // init the z of the rangeMat to Nan
                rangeMatFilled[(Horizon_SCAN * N_SCAN) * 4 + j * N_SCAN + i] = mxGetNaN();
                // init the intensity of the rangeMat to Nan
                rangeMatFilled[(Horizon_SCAN * N_SCAN) * 5 + j * N_SCAN + i] = mxGetNaN();
            }
        }

        for(int j = 0; j < Horizon_SCAN; j ++){
            for(int i = 0; i < N_SCAN; i ++){
                int current_valid_label = int(rangeMat[(Horizon_SCAN * N_SCAN) * 0 + j * N_SCAN + i]);
                if(current_valid_label == 1){
                    // the current point is valid, copy the raw data to the filled data
                    rangeMatFilled[(Horizon_SCAN * N_SCAN) * 0 + j * N_SCAN + i] = rangeMat[(Horizon_SCAN * N_SCAN) * 0 + j * N_SCAN + i];
                    rangeMatFilled[(Horizon_SCAN * N_SCAN) * 1 + j * N_SCAN + i] = rangeMat[(Horizon_SCAN * N_SCAN) * 1 + j * N_SCAN + i];
                    rangeMatFilled[(Horizon_SCAN * N_SCAN) * 2 + j * N_SCAN + i] = rangeMat[(Horizon_SCAN * N_SCAN) * 2 + j * N_SCAN + i];
                    rangeMatFilled[(Horizon_SCAN * N_SCAN) * 3 + j * N_SCAN + i] = rangeMat[(Horizon_SCAN * N_SCAN) * 3 + j * N_SCAN + i];
                    rangeMatFilled[(Horizon_SCAN * N_SCAN) * 4 + j * N_SCAN + i] = rangeMat[(Horizon_SCAN * N_SCAN) * 4 + j * N_SCAN + i];
                    rangeMatFilled[(Horizon_SCAN * N_SCAN) * 5 + j * N_SCAN + i] = rangeMat[(Horizon_SCAN * N_SCAN) * 5 + j * N_SCAN + i];
                }else{
                    // up, down, left, right
                    int valid_label[4] = {0};
                    double range[4] = {0.0}; 
                    double position_x[4] = {0.0}; 
                    double position_y[4] = {0.0};
                    double position_z[4] = {0.0};
                    double position_i[4] = {0.0};

                    double sum_range = 0.0; 
                    double sum_x = 0.0;
                    double sum_y = 0.0;
                    double sum_z = 0.0;
                    double sum_i = 0.0;
                    int count = 0;

                    if(i>1){
                        valid_label[0] = int(rangeMat[(Horizon_SCAN * N_SCAN) * 0 +  (j)* N_SCAN + (i-1)]);
                        if(valid_label[0]){
                            range[0] = rangeMat[(Horizon_SCAN * N_SCAN) * 1 + (j) * N_SCAN + (i-1)];
                            position_x[0] = rangeMat[(Horizon_SCAN * N_SCAN) * 2 + (j) * N_SCAN + (i-1)];
                            position_y[0] = rangeMat[(Horizon_SCAN * N_SCAN) * 3 + (j) * N_SCAN + (i-1)];
                            position_z[0] = rangeMat[(Horizon_SCAN * N_SCAN) * 4 + (j) * N_SCAN + (i-1)];
                            position_i[0] = rangeMat[(Horizon_SCAN * N_SCAN) * 5 + (j) * N_SCAN + (i-1)];

                            // add the range, x, y, z, i, seperately
                            count++;
                            sum_range += range[0];
                            sum_x += position_x[0];
                            sum_y += position_y[0];
                            sum_z += position_z[0];
                            sum_i += position_i[0];
                        }
                    }
                    if(i<N_SCAN-1){
                        valid_label[1] = int(rangeMat[(Horizon_SCAN * N_SCAN) * 0 + (j) * N_SCAN + (i+1)]);
                        if(valid_label[1]){
                            range[1] = rangeMat[(Horizon_SCAN * N_SCAN) * 1 + (j) * N_SCAN + (i+1)];
                            position_x[1] = rangeMat[(Horizon_SCAN * N_SCAN) * 2 + (j) * N_SCAN + (i+1)];
                            position_y[1] = rangeMat[(Horizon_SCAN * N_SCAN) * 3 + (j) * N_SCAN + (i+1)];
                            position_z[1] = rangeMat[(Horizon_SCAN * N_SCAN) * 4 + (j) * N_SCAN + (i+1)];
                            position_i[1] = rangeMat[(Horizon_SCAN * N_SCAN) * 5 + (j) * N_SCAN + (i+1)];

                            // add the range, x, y, z, i, seperately
                            count++;
                            sum_range += range[1];
                            sum_x += position_x[1];
                            sum_y += position_y[1];
                            sum_z += position_z[1];
                            sum_i += position_i[1];
                        }
                    }
                    if(j>1){
                        valid_label[2] = int(rangeMat[(Horizon_SCAN * N_SCAN) * 0 + (j-1) * N_SCAN + (i)]);
                        if(valid_label[2]){
                            range[2] = rangeMat[(Horizon_SCAN * N_SCAN) * 1 + (j-1) * N_SCAN + (i)];
                            position_x[2] = rangeMat[(Horizon_SCAN * N_SCAN) * 2 + (j-1) * N_SCAN + (i)];
                            position_y[2] = rangeMat[(Horizon_SCAN * N_SCAN) * 3 + (j-1) * N_SCAN + (i)];
                            position_z[2] = rangeMat[(Horizon_SCAN * N_SCAN) * 4 + (j-1) * N_SCAN + (i)];
                            position_i[2] = rangeMat[(Horizon_SCAN * N_SCAN) * 5 + (j-1) * N_SCAN + (i)];

                            // add the range, x, y, z, i, seperately
                            count++;
                            sum_range += range[2];
                            sum_x += position_x[2];
                            sum_y += position_y[2];
                            sum_z += position_z[2];
                            sum_i += position_i[2];
                        }
                    }
                    if(j<Horizon_SCAN-1){
                        valid_label[3] = int(rangeMat[(Horizon_SCAN * N_SCAN) * 0 + (j+1) * N_SCAN + (i)]);
                        if(valid_label[3]){
                            range[3] = rangeMat[(Horizon_SCAN * N_SCAN) * 1 + (j+1) * N_SCAN + (i)];
                            position_x[3] = rangeMat[(Horizon_SCAN * N_SCAN) * 2 + (j+1) * N_SCAN + (i)];
                            position_y[3] = rangeMat[(Horizon_SCAN * N_SCAN) * 3 + (j+1) * N_SCAN + (i)];
                            position_z[3] = rangeMat[(Horizon_SCAN * N_SCAN) * 4 + (j+1) * N_SCAN + (i)];
                            position_i[3] = rangeMat[(Horizon_SCAN * N_SCAN) * 5 + (j+1) * N_SCAN + (i)];

                            // add the range, x, y, z, i, seperately
                            count++;
                            sum_range += range[3];
                            sum_x += position_x[3];
                            sum_y += position_y[3];
                            sum_z += position_z[3];
                            sum_i += position_i[3];
                        }
                    }

                    if(count>1){
                        // printf("up_range: %f, down_range: %f, left_range: %f, right_range: %f\n", range[0], range[1], range[2], range[3]);
                        // printf("count: %d, sum_range: %f, sum_x: %f, sum_y: %f, sum_z: %f, sum_i: %f\n", count, sum_range, sum_x, sum_y, sum_z, sum_i);
                        rangeMatFilled[(Horizon_SCAN * N_SCAN) * 0 + j * N_SCAN + i] = 1;
                        rangeMatFilled[(Horizon_SCAN * N_SCAN) * 1 + j * N_SCAN + i] = sum_range/count;
                        rangeMatFilled[(Horizon_SCAN * N_SCAN) * 2 + j * N_SCAN + i] = sum_x/count;
                        rangeMatFilled[(Horizon_SCAN * N_SCAN) * 3 + j * N_SCAN + i] = sum_y/count;
                        rangeMatFilled[(Horizon_SCAN * N_SCAN) * 4 + j * N_SCAN + i] = sum_z/count;
                        rangeMatFilled[(Horizon_SCAN * N_SCAN) * 5 + j * N_SCAN + i] = sum_i/count;
                    }
                }
            }
        }
    }
    

    delete []horizonal_theta;
}