#include<stdio.h>
#include"mex.h"

#include "cloudSegmentation.cpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    //nlhs represent the number of parameters of the output
    //plhs is a array of the mxarray pointers, each pointing to the output
    //nrhs represents the number of parameters of the input
    //prhs is a array of the mxarray pointers, each pointing to the input

    // the first matrix represent the raw pointCloud: Nx4, | x | y | z | intensity |
    /* check proper input and output */

    // mexPrintf("=========================================================================\n");
    if(nrhs < 3)
        mexErrMsgIdAndTxt( "cloudSegmentation:invalidNumInputs",
                "at least three input arguments required");
    else if(!mxIsStruct(prhs[0]))
        mexErrMsgIdAndTxt( "cloudSegmentation:inputNotStruct",
                "the first input should be a struct");

    const size_t *dimArrayOfRangeMat = mxGetDimensions(prhs[1]);
    double *ptrRangeMat = (double *)(mxGetPr(prhs[1]));
    size_t N_SCAN = *dimArrayOfRangeMat;
    size_t Horizon_SCAN = *(dimArrayOfRangeMat + 1);

    int8_t *ptrGroundLabel = (int8_t *)(mxGetPr(prhs[2]));

    // get the params
    mxArray * fPtr;
    double * vertical_theta;
    double segmentTheta;
    int feasibleSegmentValidPointNum;
    int segmentValidPointNum;
    int segmentValidLineNum;
    int groundScanInd;
    double *debugInfo;

    // get the parameter : vertical_theta
    fPtr = mxGetField(prhs[0], 0, "vertical_theta");
    if ((fPtr != NULL)){
        const size_t *dimArrayOfVeritcalTheta = mxGetDimensions(fPtr);
        int numOfVerticalTheta = *(dimArrayOfVeritcalTheta + 1);
        vertical_theta = mxGetPr(fPtr);
        if(numOfVerticalTheta != N_SCAN){
            mexPrintf("%s%d\n", "numOfVerticalTheta: ", numOfVerticalTheta);
            mexErrMsgIdAndTxt( "cloudSegmentation:invalidField",
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
        mexErrMsgIdAndTxt( "cloudSegmentation:invalidField",
            "field vertical_theta not found in the params struct");
    }

    fPtr = mxGetField(prhs[0], 0, "segmentTheta");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        segmentTheta = realPtr[0];
        #ifdef DEBUG
            mexPrintf("%s%f\n", "segmentTheta: ", segmentTheta);
        #endif
    }else{
        mexErrMsgIdAndTxt( "cloudSegmentation:invalidField",
            "field segmentTheta not found in the params struct");
    }

    fPtr = mxGetField(prhs[0], 0, "feasibleSegmentValidPointNum");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        feasibleSegmentValidPointNum = int(realPtr[0]);
        #ifdef DEBUG
            mexPrintf("%s%d\n", "feasibleSegmentValidPointNum: ", feasibleSegmentValidPointNum);
        #endif
    }else{
        mexErrMsgIdAndTxt( "cloudSegmentation:invalidField",
            "field feasibleSegmentValidPointNum not found in the params struct");
    }

    fPtr = mxGetField(prhs[0], 0, "segmentValidPointNum");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        segmentValidPointNum = int(realPtr[0]);
        #ifdef DEBUG
            mexPrintf("%s%d\n", "segmentValidPointNum: ", segmentValidPointNum);
        #endif
    }else{
        mexErrMsgIdAndTxt( "cloudSegmentation:invalidField",
            "field segmentValidPointNum not found in the params struct");
    }

    fPtr = mxGetField(prhs[0], 0, "segmentValidLineNum");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        segmentValidLineNum = int(realPtr[0]);
        #ifdef DEBUG
            mexPrintf("%s%d\n", "segmentValidLineNum: ", segmentValidLineNum);
        #endif
    }else{
        mexErrMsgIdAndTxt( "cloudSegmentation:invalidField",
            "field segmentValidLineNum not found in the params struct");
    }

    fPtr = mxGetField(prhs[0], 0, "groundScanInd");
    if ((fPtr != NULL)){
        double *realPtr = mxGetPr(fPtr);
        groundScanInd = int(realPtr[0]);
        #ifdef DEBUG
            mexPrintf("%s%d\n", "groundScanInd: ", groundScanInd);
        #endif
    }else{
        mexErrMsgIdAndTxt( "cloudSegmentation:invalidField",
            "field groundScanInd not found in the params struct");
    }

    // get the parameter : debugInfo
    fPtr = mxGetField(prhs[0], 0, "debugInfo");
    if ((fPtr != NULL)){
        const size_t *dimArrayOfDebugInfo = mxGetDimensions(fPtr);
        int numOfDebugInfo = *(dimArrayOfDebugInfo + 1);
        debugInfo = mxGetPr(fPtr);
        #ifdef DEBUG
            mexPrintf("%s%d\n", "debugInfo: ", numOfDebugInfo);
            mexPrintf("%s\n", "debugInfo: ");
            for(int i = 0; i < numOfDebugInfo; i++){
                mexPrintf("%d\t", int(debugInfo[i]));
                if((i+1)%8 == 0){
                    mexPrintf("\n");
                }
            }
            mexPrintf("\n");
        #endif
    }else{
        mexErrMsgIdAndTxt( "cloudSegmentation:invalidField",
            "field debugInfo not found in the params struct");
    }

    // ===========================================================================
    // mexPrintf("declear the cloudSegmentation class object for the segmentation...\n");
    CloudSegmentation cloudSegmentation;
    cloudSegmentation.setDimension(N_SCAN, Horizon_SCAN);
    cloudSegmentation.setData(ptrRangeMat, ptrGroundLabel);
    cloudSegmentation.allocateMemory();
    cloudSegmentation.setContents();
    cloudSegmentation.setSegmentAlpha(segmentTheta, vertical_theta);
    cloudSegmentation.setConfig(feasibleSegmentValidPointNum, segmentValidPointNum, segmentValidLineNum, groundScanInd);
    cloudSegmentation.setDebugInfo(int(debugInfo[0]), int(debugInfo[1]), int(debugInfo[2]));

    // segment the cloud
    cloudSegmentation.segCloud();

    // define the output of the mex function
    plhs[0] = mxCreateNumericArray(2, dimArrayOfRangeMat, mxINT32_CLASS, mxREAL);
    int32_t *labelMat = (int32_t *)mxGetData(plhs[0]);

    for (size_t i = 0; i < N_SCAN; i++)
        for (size_t j = 0; j < Horizon_SCAN; j++){
            labelMat[j * N_SCAN + i] = cloudSegmentation.labelMat.at<int>(i, j);
        }
}