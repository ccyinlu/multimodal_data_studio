#include<stdio.h>
#include"mex.h"

#include<ceres/ceres.h>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

struct NORMAL_COST
{
  NORMAL_COST(Eigen::Matrix<double, 1, 3> parent_normal, Eigen::Matrix<double, 1, 3> child_normal):_parent_normal(parent_normal),_child_normal(child_normal){}
  template <typename T>
  bool operator()(const T *child_to_parent_q, T *residual)const
  {
    // get the transformation from the child_to_parent_q [qw qx qy qz]
    // Quaternion [w, x, y, z]
    Eigen::Quaternion<T> child_to_parent_q_(child_to_parent_q[0], child_to_parent_q[1], child_to_parent_q[2], child_to_parent_q[3]);
    Eigen::Matrix<T, 3, 3> child_to_parent_rotationalMatrix = child_to_parent_q_.toRotationMatrix();


    // get the vector from the parent_normal and child_normal [nx ny nz]
    Eigen::Matrix<T, 3, 1> parent_normal_v(T(_parent_normal(0, 0)), T(_parent_normal(0, 1)), T(_parent_normal(0, 2)));

    Eigen::Matrix<T, 3, 1> child_normal_v(T(_child_normal(0, 0)), T(_child_normal(0, 1)), T(_child_normal(0, 2)));

    Eigen::Matrix<T, 3, 1> _residual_normal_v = child_to_parent_rotationalMatrix * child_normal_v - parent_normal_v;

    residual[0] = _residual_normal_v(0, 0);
    residual[1] = _residual_normal_v(1, 0);
    residual[2] = _residual_normal_v(2, 0);

    #ifdef DEBUG
      mexPrintf("child_normal_v: [%f, %f, %f]\n", _child_normal(0, 0), _child_normal(0, 1), _child_normal(0, 2));
      mexPrintf("parent_normal_v: [%f, %f, %f]\n", _parent_normal(0, 0), _parent_normal(0, 1), _parent_normal(0, 2));
      mexPrintf("child_to_parent_rotationalMatrix: \n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]\n", \
              child_to_parent_rotationalMatrix(0, 0), child_to_parent_rotationalMatrix(0, 1), child_to_parent_rotationalMatrix(0, 2), \
              child_to_parent_rotationalMatrix(1, 0), child_to_parent_rotationalMatrix(1, 1), child_to_parent_rotationalMatrix(1, 2), \
              child_to_parent_rotationalMatrix(2, 0), child_to_parent_rotationalMatrix(2, 1), child_to_parent_rotationalMatrix(2, 2) \
              );
      mexPrintf("residual: [%f, %f, %f]\n", residual[0], residual[1], residual[2]);
    #endif
    
    return true;
  }
  const Eigen::Matrix<double, 1, 3> _parent_normal, _child_normal;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    // nlhs represent the number of parameters of the output
    // plhs is a array of the mxarray pointers, each pointing to the output
    // nrhs represents the number of parameters of the input
    // prhs is a array of the mxarray pointers, each pointing to the input

    // prhs[0], nx3 double, parent_normal, [nx ny nz]
    // prhs[1], nx3 double, child_normal, [nx ny nz]
    // prhs[2], 1x4 double, child_to_parent_q, [qx qy qz qw]
    // prhs[3], 1x1 double, verbose

    if(nrhs < 4){
        mexErrMsgIdAndTxt( "estimatePitchRollByCoNormalCeresMex:invalidNumInputs", "at least 4 input arguments required");
        return;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the parent_normals
    const size_t *dimArrayOfParentNormals = mxGetDimensions(prhs[0]);
    size_t sizeRowsParentNormals = *(dimArrayOfParentNormals + 0);
    size_t sizeColsParentNormals = *(dimArrayOfParentNormals + 1);

    if(sizeColsParentNormals != 3){
        mexErrMsgIdAndTxt( "estimatePitchRollByCoNormalCeresMex:invalidInputs", "the 1st param should be Nx3");
        return;
    }

    double *ptrParentNormals = (double *)(mxGetPr(prhs[0]));
    std::vector<Eigen::Matrix<double, 1, 3>>parent_normals_all;
    for(int i = 0; i < sizeRowsParentNormals; i++){
        Eigen::Matrix<double, 1, 3> current_parent_normal;
        for(int j = 0; j < 3; j++){
            current_parent_normal(0, j) = ptrParentNormals[j * sizeRowsParentNormals + i];
        }
        parent_normals_all.push_back(current_parent_normal);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the child_normals
    const size_t *dimArrayOfChildNormals = mxGetDimensions(prhs[1]);
    size_t sizeRowsChildNormals = *(dimArrayOfChildNormals + 0);
    size_t sizeColsChildNormals = *(dimArrayOfChildNormals + 1);
    if(sizeColsChildNormals != 3){
        mexErrMsgIdAndTxt( "estimatePitchRollByCoNormalCeresMex:invalidInputs", "the 2st param should be Nx3");
        return;
    }

    double *ptrChildNormals = (double *)(mxGetPr(prhs[1]));
    std::vector<Eigen::Matrix<double, 1, 3>>child_normals_all;
    for(int i = 0; i < sizeRowsChildNormals; i++){
        Eigen::Matrix<double, 1, 3> current_child_normal;
        for(int j = 0; j < 3; j++){
            current_child_normal(0, j) = ptrChildNormals[j * sizeRowsChildNormals + i];
        }
        child_normals_all.push_back(current_child_normal);
    }

    if(sizeRowsParentNormals != sizeRowsChildNormals){
        mexErrMsgIdAndTxt( "estimatePitchRollByCoNormalCeresMex:invalidInputs", "the 1st param and the 2st param should have the same size");
        return;
    }

    #ifdef DEBUG
        mexPrintf("observation number: %d\n", sizeRowsParentNormals);
    #endif

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the init_child_to_parent_q
    double child_to_parent_q[4] = {0};
    const size_t *dimArrayOfChildToParentQ = mxGetDimensions(prhs[2]);
    size_t sizeRowsChildToParentQ = *(dimArrayOfChildToParentQ + 0);
    size_t sizeColsChildToParentQ = *(dimArrayOfChildToParentQ + 1);
    if(sizeColsChildToParentQ != 4 && sizeRowsChildToParentQ != 1){
        mexErrMsgIdAndTxt( "estimatePitchRollByCoNormalCeresMex:invalidInputs", "the 3st param should be 1x4");
        return;
    }
    double *ptrChildToParentQ = (double *)(mxGetPr(prhs[2]));
    for(int i = 0; i < 4; i++){
        child_to_parent_q[i] = ptrChildToParentQ[i];
    }

    #ifdef DEBUG
        mexPrintf("init child_to_parent_q: [%.4f,%.4f,%.4f,%.4f]\n", \
                    child_to_parent_q[0], \
                    child_to_parent_q[1], \
                    child_to_parent_q[2], \
                    child_to_parent_q[3]);
    #endif

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get the verbose
    bool verbose = false;
    const size_t *dimArrayOfVerbose = mxGetDimensions(prhs[3]);
    size_t sizeRowsVerbose = *(dimArrayOfVerbose + 0);
    size_t sizeColsVerbose = *(dimArrayOfVerbose + 1);
    if(sizeRowsVerbose != 1 && sizeColsVerbose != 1){
        mexErrMsgIdAndTxt( "estimatePitchRollByCoNormalCeresMex:invalidInputs", "the 4st param should be 1x1");
        return;
    }
    double *ptrVerbose = (double *)(mxGetPr(prhs[3]));
    verbose = bool(ptrVerbose[0]);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ceres::Problem problem;
    for(int i=0; i<sizeRowsParentNormals; i++)
    {
        problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<NORMAL_COST, 3, 4>(
              new NORMAL_COST(parent_normals_all[i], child_normals_all[i])
          ),
          NULL,
          child_to_parent_q
        );
    }

    #ifdef DEBUG
        mexPrintf("AddResidualBlock done!\n");
    #endif

    //配置求解器并求解，输出结果
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_QR;
    options.minimizer_progress_to_stdout=verbose;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    #ifdef DEBUG
        mexPrintf("estimated child_to_parent_q: [%.4f,%.4f,%.4f,%.4f]\n", \
                    child_to_parent_q[0], \
                    child_to_parent_q[1], \
                    child_to_parent_q[2], \
                    child_to_parent_q[3]);
    #endif

    // the output q will be 1x4
    size_t dimArrayOfParams[2] = { 1, 4 };
    plhs[0] = mxCreateNumericArray(2, dimArrayOfParams, mxDOUBLE_CLASS, mxREAL);
    double *out_params = (double *)mxGetData(plhs[0]);

    for(int i = 0; i < 4; i++){
        out_params[i] = child_to_parent_q[i];
    }
}