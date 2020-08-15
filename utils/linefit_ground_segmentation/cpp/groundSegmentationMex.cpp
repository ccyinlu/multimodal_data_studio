#include<stdio.h>
#include"mex.h"

#include "ground_segmentation.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    //nlhs represent the number of parameters of the output
    //plhs is a array of the mxarray pointers, each pointing to the output
    //nrhs represents the number of parameters of the input
    //prhs is a array of the mxarray pointers, each pointing to the input

    // the first matrix represent the raw pointCloud: Nx4, | x | y | z | intensity |
    /* check proper input and output */

    // prhs[0] GroundSegmentationParams
    // prhs[1] pointCloud, Nx3, | x | y | z |
    #ifdef DEBUG
      mexPrintf("=========================================================================\n");
    #endif
    if(nrhs < 2)
        mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidNumInputs",
                "at least 2 input arguments required");
    else if(!mxIsStruct(prhs[0]))
        mexErrMsgIdAndTxt( "linefitGroundSegmentation:inputNotStruct",
                "the first input should be a struct");

    ///////////////////////////////////////////////////////////////////////////////////////////
    GroundSegmentationParams params_;
    // get the params
    mxArray * fPtr;
    // Minimum range of segmentation.
    double r_min_square;
    // Maximum range of segmentation.
    double r_max_square;
    // Number of radial bins.
    int n_bins;
    // Number of angular segments.
    int n_segments;
    // Maximum distance to a ground line to be classified as ground.
    double max_dist_to_line;
    // Max slope to be considered ground line.
    double max_slope;
    // Max error for line fit.
    double max_error_square;
    // Distance at which points are considered far from each other.
    double long_threshold;
    // Maximum slope for
    double max_long_height;
    // Maximum heigh of starting line to be labelled ground.
    double max_start_height;
    // Height of sensor above ground.
    double sensor_height;
    // How far to search for a line in angular direction [rad].
    double line_search_angle;
    // Number of threads.
    int n_threads;

    // get the parameter : r_min_square
    {
      fPtr = mxGetField(prhs[0], 0, "r_min_square");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          r_min_square = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "r_min_square: ", r_min_square);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field r_min_square not found in the params struct");
      }
    }
    
    // get the parameter : r_max_square
    {
      fPtr = mxGetField(prhs[0], 0, "r_max_square");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          r_max_square = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "r_max_square: ", r_max_square);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field r_max_square not found in the params struct");
      }
    }

    // get the parameter : n_bins
    {
      fPtr = mxGetField(prhs[0], 0, "n_bins");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          n_bins = int(realPtr[0]);
          #ifdef DEBUG
              mexPrintf("%s%d\n", "n_bins: ", n_bins);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field n_bins not found in the params struct");
      }
    }

    // get the parameter : n_segments
    {
      fPtr = mxGetField(prhs[0], 0, "n_segments");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          n_segments = int(realPtr[0]);
          #ifdef DEBUG
              mexPrintf("%s%d\n", "n_segments: ", n_segments);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field n_segments not found in the params struct");
      }
    }

    // get the parameter : max_dist_to_line
    {
      fPtr = mxGetField(prhs[0], 0, "max_dist_to_line");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          max_dist_to_line = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "max_dist_to_line: ", max_dist_to_line);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field max_dist_to_line not found in the params struct");
      }
    }

    // get the parameter : max_slope
    {
      fPtr = mxGetField(prhs[0], 0, "max_slope");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          max_slope = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "max_slope: ", max_slope);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field max_slope not found in the params struct");
      }
    }

    // get the parameter : max_error_square
    {
      fPtr = mxGetField(prhs[0], 0, "max_error_square");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          max_error_square = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "max_error_square: ", max_error_square);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field max_error_square not found in the params struct");
      }
    }

    // get the parameter : long_threshold
    {
      fPtr = mxGetField(prhs[0], 0, "long_threshold");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          long_threshold = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "long_threshold: ", long_threshold);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field long_threshold not found in the params struct");
      }
    }

    // get the parameter : max_long_height
    {
      fPtr = mxGetField(prhs[0], 0, "max_long_height");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          max_long_height = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "max_long_height: ", max_long_height);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field max_long_height not found in the params struct");
      }
    }

    // get the parameter : max_start_height
    {
      fPtr = mxGetField(prhs[0], 0, "max_start_height");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          max_start_height = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "max_start_height: ", max_start_height);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field max_start_height not found in the params struct");
      }
    }

    // get the parameter : sensor_height
    {
      fPtr = mxGetField(prhs[0], 0, "sensor_height");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          sensor_height = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "sensor_height: ", sensor_height);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field sensor_height not found in the params struct");
      }
    }

    // get the parameter : line_search_angle
    {
      fPtr = mxGetField(prhs[0], 0, "line_search_angle");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          line_search_angle = realPtr[0];
          #ifdef DEBUG
              mexPrintf("%s%f\n", "line_search_angle: ", line_search_angle);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field line_search_angle not found in the params struct");
      }
    }

    // get the parameter : n_threads
    {
      fPtr = mxGetField(prhs[0], 0, "n_threads");
      if ((fPtr != NULL)){
          double *realPtr = mxGetPr(fPtr);
          n_threads = int(realPtr[0]);
          #ifdef DEBUG
              mexPrintf("%s%d\n", "n_threads: ", n_threads);
          #endif
      }else{
          mexErrMsgIdAndTxt( "linefitGroundSegmentation:invalidField",
              "field n_threads not found in the params struct");
      }
    }

    params_.r_min_square = r_min_square;
    params_.r_max_square = r_max_square;
    params_.n_bins = n_bins;
    params_.n_segments = n_segments;
    params_.max_dist_to_line = max_dist_to_line;
    params_.max_slope = max_slope;
    params_.max_error_square = max_error_square;
    params_.long_threshold = long_threshold;
    params_.max_long_height = max_long_height;
    params_.max_start_height = max_start_height;
    params_.sensor_height = sensor_height;
    params_.line_search_angle = line_search_angle;
    params_.n_threads = n_threads;

    // get the pointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const size_t *dimArrayOfRawPointCloud = mxGetDimensions(prhs[1]);
    double *ptrRawPointCloud = mxGetPr(prhs[1]);
    size_t numOfPoints = *dimArrayOfRawPointCloud;
    size_t numOfCols = *(dimArrayOfRawPointCloud + 1);
    pcl::PointXYZ p;
    for(int i = 0; i < numOfPoints; i++){
      p.x = ptrRawPointCloud[(numOfPoints * 0) + i];
      p.y = ptrRawPointCloud[(numOfPoints * 1) + i];
      p.z = ptrRawPointCloud[(numOfPoints * 2) + i];
      cloud.push_back(p);
    }

    // ===========================================================================
    // mexPrintf("declear the GroundSegmentation class object for the segmentation...\n");
    GroundSegmentation segmenter(params_);
    std::vector<int> labels;

    segmenter.segment(cloud, &labels);
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;

    for (size_t i = 0; i < cloud.size(); ++i) {
      if (labels[i] == 1) ground_cloud.push_back(cloud[i]);
      else obstacle_cloud.push_back(cloud[i]);
    }

    size_t ground_cloud_num = ground_cloud.size();
    size_t obstacle_cloud_num = obstacle_cloud.size();

    #ifdef DEBUG
      mexPrintf("%s%d\n", "ground_cloud_num: ", ground_cloud_num);
    #endif

    #ifdef DEBUG
      mexPrintf("%s%d\n", "obstacle_cloud_num: ", obstacle_cloud_num);
    #endif

    // define the output of the mex function
    size_t dimArrayOfGroundCloud[2] = { ground_cloud_num, 3};
    plhs[0] = mxCreateNumericArray(2, dimArrayOfGroundCloud, mxDOUBLE_CLASS, mxREAL);
    double *groundCloudPtr = (double *)mxGetData(plhs[0]);

    size_t dimArrayOfObstacleCloud[2] = { obstacle_cloud_num, 3};
    plhs[1] = mxCreateNumericArray(2, dimArrayOfObstacleCloud, mxDOUBLE_CLASS, mxREAL);
    double *obstacleCloudPtr = (double *)mxGetData(plhs[1]);

    for (size_t i = 0; i < ground_cloud_num; i++) {
      groundCloudPtr[ground_cloud_num * 0 + i] = ground_cloud[i].x;
      groundCloudPtr[ground_cloud_num * 1 + i] = ground_cloud[i].y;
      groundCloudPtr[ground_cloud_num * 2 + i] = ground_cloud[i].z;
    }

    for (size_t i = 0; i < obstacle_cloud_num; i++) {
      obstacleCloudPtr[obstacle_cloud_num * 0 + i] = obstacle_cloud[i].x;
      obstacleCloudPtr[obstacle_cloud_num * 1 + i] = obstacle_cloud[i].y;
      obstacleCloudPtr[obstacle_cloud_num * 2 + i] = obstacle_cloud[i].z;
    }
}