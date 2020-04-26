#include<stdio.h>
#include"mex.h"

#include <math.h>
 
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    //nlhs represent the number of parameters of the output
    //plhs is a array of the mxarray pointers, each pointing to the output
    //nrhs represents the number of parameters of the input
    //prhs is a array of the mxarray pointers, each pointing to the input

    // the first matrix represent the rangeMat: MxNx6, | valid | range | x | y | z | intensity |
    /* check proper input and output */
    #ifdef DEBUG
        mexPrintf("=========================================================================\n");
    #endif
    
    if(nrhs < 2)
        mexErrMsgIdAndTxt( "groundRemoval:invalidNumInputs",
                "at least two input arguments required");
    else if(!mxIsStruct(prhs[0]))
        mexErrMsgIdAndTxt( "groundRemoval:inputNotStruct",
                "the first input should be a struct");

    if(mxGetNumberOfDimensions(prhs[1]) < 3){
        mexPrintf("%s%d\n", "range matrix dimensions: ", mxGetNumberOfDimensions(prhs[1]));
        mexErrMsgIdAndTxt( "groundRemoval:invalidField",
                "range matrix dimensions lower than 3, exit!");
    }
    const size_t *dimArrayOfRangeMat = mxGetDimensions(prhs[1]);
    double *ptrRangeMat = mxGetPr(prhs[1]);
    size_t N_SCAN = *dimArrayOfRangeMat;
    size_t Horizon_SCAN = *(dimArrayOfRangeMat + 1);
    size_t numOfChannel = *(dimArrayOfRangeMat + 2);

    mxArray * fPtr;
    int groundScanInd = 30;
    double sensorMountAngle = 0.0;
    double groundRemovalAngleT = 10.0;
    fPtr = mxGetField(prhs[0], 0, "groundScanInd");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        groundScanInd = int(realPtr[0]);
        if(groundScanInd < 1 || groundScanInd > N_SCAN){
            mexPrintf("%s%d\n", "groundScanInd: ", groundScanInd);
            mexErrMsgIdAndTxt( "groundRemoval:invalidField",
                    "groundScanInd must be within the range of 1~N_SCAN");
        }
        #ifdef DEBUG
            mexPrintf("%s%d\n", "groundScanInd: ", groundScanInd);
        #endif
    }else{
        mexErrMsgIdAndTxt( "groundRemoval:invalidField",
            "field groundScanInd not found in the params struct");
    }

    fPtr = mxGetField(prhs[0], 0, "sensorMountAngle");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        sensorMountAngle = realPtr[0];
        if(sensorMountAngle < -30.0 || sensorMountAngle > 30.0){
            mexPrintf("%s%f\n", "sensorMountAngle: ", sensorMountAngle);
            mexErrMsgIdAndTxt( "groundRemoval:invalidField",
                    "sensorMountAngle must be within the range of -30 ~ 30");
        }
        #ifdef DEBUG
            mexPrintf("%s%f\n", "sensorMountAngle: ", sensorMountAngle);
        #endif
    }else{
        mexErrMsgIdAndTxt( "groundRemoval:invalidField",
            "field sensorMountAngle not found in the params struct");
    }

    fPtr = mxGetField(prhs[0], 0, "groundRemovalAngleT");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        groundRemovalAngleT = realPtr[0];
        #ifdef DEBUG
            mexPrintf("%s%f\n", "groundRemovalAngleT: ", groundRemovalAngleT);
        #endif
    }else{
        mexErrMsgIdAndTxt( "groundRemoval:invalidField",
            "field groundRemovalAngleT not found in the params struct");
    }
    
    if(numOfChannel != 6){
        mexPrintf("%s%d\n", "numOfChannel: ", numOfChannel);
        mexErrMsgIdAndTxt( "groundRemoval:invalidField",
                "numOfChannel must be 6, representing valid_label, range, x, y, z, intensity");
    }

    #ifdef DEBUG
        mexPrintf("%s%d\t%s%d\t%s%d\n", "number of scans: ", N_SCAN, "number of cols: ", Horizon_SCAN, "number of channels: ", numOfChannel);
    #endif

    plhs[0] = mxCreateNumericArray(2, dimArrayOfRangeMat, mxINT8_CLASS, mxREAL);
    signed char * groundLabel = (signed char *)mxGetData(plhs[0]);

    size_t lowerInd, upperInd, curInd;
    float diffX, diffY, diffZ, angle, angle_upper2cur, angle_cur2lower;

    // init the groundLabel to zeros
    for (size_t j = 0; j < Horizon_SCAN; ++j){
        for (size_t i = 0; i < N_SCAN; i++){
            groundLabel[ N_SCAN * j + i] = 0;
        }
    }

    // groundLabel
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    /*
    for (size_t j = 0; j < Horizon_SCAN; j++){
        for (size_t i = N_SCAN - 1; i >= 1; i--){

            lowerInd = j * N_SCAN + i;
            upperInd = j * N_SCAN + (i-1);

            // if(i == N_SCAN - 1){
            //     mexPrintf("%s%f\n", "lowerInd: ", ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + lowerInd]);
            //     mexPrintf("%s%f\n", "upperInd: ", ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + upperInd]);
            //     mexPrintf("----\n");
            // }

            if (ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + lowerInd] == 0 ||
                ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + upperInd] == 0){
                // no info to check, invalid points
                groundLabel[lowerInd] = -1;
                groundLabel[upperInd] = -1;
                continue;
            }

            if (i < groundScanInd){
                // above the groundIndx, set to zeros
                groundLabel[lowerInd] = 0;
                continue;
            }

            diffX = ptrRangeMat[(N_SCAN*Horizon_SCAN)*2 + upperInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*2 + lowerInd];
            diffY = ptrRangeMat[(N_SCAN*Horizon_SCAN)*3 + upperInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*3 + lowerInd];
            diffZ = ptrRangeMat[(N_SCAN*Horizon_SCAN)*4 + upperInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*4 + lowerInd];

            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= groundRemovalAngleT){
                // groundLabel[lowerInd] = 1;
                groundLabel[upperInd] = 1;
            }else{
                groundLabel[lowerInd] = 0;
            }
        }
    }
    */
    
    
    for (size_t j = 0; j < Horizon_SCAN; j++){
        for (size_t i = N_SCAN - 1; i >= 1; i--){

            if(i == N_SCAN - 1){
                lowerInd = j * N_SCAN + i;
            }else{
                lowerInd = j * N_SCAN + (i + 1);
            }
            
            curInd = j * N_SCAN + i;
            upperInd = j * N_SCAN + (i - 1);

            // if(i == N_SCAN - 1){
            //     mexPrintf("%s%f\n", "lowerInd: ", ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + lowerInd]);
            //     mexPrintf("%s%f\n", "curInd: ", ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + curInd]);
            //     mexPrintf("%s%f\n", "upperInd: ", ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + upperInd]);
            //     mexPrintf("----\n");
            // }

            if (ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + lowerInd] == 0 ||
                ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + curInd] == 0 ||
                ptrRangeMat[(N_SCAN*Horizon_SCAN)*0 + upperInd] == 0){
                // no info to check, invalid points
                groundLabel[curInd] = -1;
                continue;
            }

            if (i < groundScanInd){
                // above the groundIndx, set to zeros
                groundLabel[curInd] = 0;
                continue;
            }

            diffX = ptrRangeMat[(N_SCAN*Horizon_SCAN)*2 + upperInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*2 + curInd];
            diffY = ptrRangeMat[(N_SCAN*Horizon_SCAN)*3 + upperInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*3 + curInd];
            diffZ = ptrRangeMat[(N_SCAN*Horizon_SCAN)*4 + upperInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*4 + curInd];

            angle_upper2cur = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

            if(i == N_SCAN - 1){
                angle_cur2lower = 0;
            }else{
                diffX = ptrRangeMat[(N_SCAN*Horizon_SCAN)*2 + curInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*2 + lowerInd];
                diffY = ptrRangeMat[(N_SCAN*Horizon_SCAN)*3 + curInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*3 + lowerInd];
                diffZ = ptrRangeMat[(N_SCAN*Horizon_SCAN)*4 + curInd] - ptrRangeMat[(N_SCAN*Horizon_SCAN)*4 + lowerInd];

                angle_cur2lower = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
            }

            // if(i == N_SCAN - 1){
            //     mexPrintf("%s%f\n", "angle_upper2cur: ", angle_upper2cur);
            //     mexPrintf("%s%f\n", "angle_cur2lower: ", angle_cur2lower);
            //     mexPrintf("----\n");
            // }
            if (abs(angle_upper2cur - sensorMountAngle) <= groundRemovalAngleT || abs(angle_cur2lower - sensorMountAngle) <= groundRemovalAngleT){
                if(i < N_SCAN - 1){
                    if(groundLabel[curInd + 1] == 1){
                        groundLabel[curInd] = 1;
                    }else{
                        groundLabel[curInd] = 0;
                    }
                }else{
                    groundLabel[curInd] = 1;
                }
                
            }else{
                groundLabel[curInd] = 0;
            }
        }
    }
    
}