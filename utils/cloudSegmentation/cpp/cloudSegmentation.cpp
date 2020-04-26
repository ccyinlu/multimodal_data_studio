
#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include"mex.h"

using namespace std;

typedef pcl::PointXYZI  PointType;

struct cloud_info{
    int32_t *startRingIndex;
    int32_t *endRingIndex;

    double startOrientation;
    double endOrientation;
    double orientationDiff;

    bool        *segmentedCloudGroundFlag;  // true - ground point, false - other points
    uint32_t    *segmentedCloudColInd;    // point column index in range image
    double      *segmentedCloudRange;       // point range 
};

class CloudSegmentation{

public:
    int N_SCAN;
    int Horizon_SCAN;

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentation marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    int16_t *allPushedIndX; // array for tracking points of a segmented object
    int16_t *allPushedIndY;

    int16_t *queueIndX; // array for breadth-first search process of segmentation
    int16_t *queueIndY;

    double *ptrRangeMat; // pointers from the outside to store the valid label, rangeMat, x, y, z, intensity
    int8_t *ptrGroundLabel; // pointers from the outside to store the ground label

    double segmentTheta; // decrese this value may improve accuracy

    double segmentAlphaX; // ang_res_x / 180.0 * M_PI;
    double *vertical_theta;

    int feasibleSegmentValidPointNum;
    int segmentValidPointNum;
    int segmentValidLineNum;
    int groundScanInd;

    cloud_info segMsg;

    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr outlierCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;

    int debugIndX;
    int debugIndY;
    int debugLabel;
public:
    ~CloudSegmentation(){}
    CloudSegmentation(){
    }

    CloudSegmentation(int _n_scan, int _horizon_scan, double * _rangeMat, int8_t * _groundLabel){
        N_SCAN = _n_scan;
        Horizon_SCAN = _horizon_scan;
        ptrRangeMat = _rangeMat;
        ptrGroundLabel = _groundLabel;

        allocateMemory();
        setContents();
    }

    void setSegmentAlpha(double _segmentTheta, double * _vertical_theta){
        double ang_res_x = 360.0/double(Horizon_SCAN);
        segmentAlphaX = ang_res_x / 180.0 * M_PI;

        vertical_theta = _vertical_theta;

        segmentTheta = _segmentTheta;
    }

    void setConfig(int _feasibleSegmentValidPointNum, int _segmentValidPointNum, int _segmentValidLineNum, int _groundScanInd){
        feasibleSegmentValidPointNum = _feasibleSegmentValidPointNum;
        segmentValidPointNum = _segmentValidPointNum;
        segmentValidLineNum = _segmentValidLineNum;
        groundScanInd = _groundScanInd;
    }

    void setDebugInfo(int _debugIndX, int _debugIndY, int _debugLabel){
        debugIndX = _debugIndX;
        debugIndY = _debugIndY;
        debugLabel = _debugLabel;
    }

    void allocateMemory(){

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new int16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new int16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new int16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new int16_t[N_SCAN*Horizon_SCAN];

        segMsg.startRingIndex = new int32_t[N_SCAN]();
        segMsg.endRingIndex = new int32_t[N_SCAN]();

        segMsg.segmentedCloudGroundFlag = new bool[N_SCAN*Horizon_SCAN]();
        segMsg.segmentedCloudColInd = new uint32_t[N_SCAN*Horizon_SCAN]();
        segMsg.segmentedCloudRange = new double[N_SCAN*Horizon_SCAN]();

        segmentedCloud.reset(new pcl::PointCloud<PointType>()); 
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>()); 
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();
    }

    void setDimension(int _n_scan, int _horizon_scan){
        N_SCAN = _n_scan;
        Horizon_SCAN = _horizon_scan;
    }

    void setData(double * _rangeMat, int8_t * _groundLabel){
        ptrRangeMat = _rangeMat;
        ptrGroundLabel = _groundLabel;
    }

    void setContents(){
        for(int j = 0; j < Horizon_SCAN; j ++){
            for(int i = 0; i < N_SCAN; i ++){
                if(ptrRangeMat[(Horizon_SCAN * N_SCAN) * 0 + j * N_SCAN + i] == 1){
                    // set the rangeMat
                    rangeMat.at<float>(i, j) = float(ptrRangeMat[(Horizon_SCAN * N_SCAN) * 1 + j * N_SCAN + i]);
                }

                // set the groundMat
                groundMat.at<int8_t>(i, j) = int8_t(ptrGroundLabel[j * N_SCAN + i]);
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){
            // Pop point
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
                // row index corresponds to IndX should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;
                // at range image margin (left or right side)
                if (thisIndY < 0){
                    /*
                    if(labelCount == debugLabel){
                        mexPrintf("thisIndY < 0, thisIndY: %d\n", thisIndY);
                    }*/
                    thisIndY = Horizon_SCAN - 1;
                    /*
                    if(labelCount == debugLabel){
                        mexPrintf("thisIndY changed to: %d\n", thisIndY);
                    }*/
                }
                    
                if (thisIndY >= Horizon_SCAN){
                    /*
                    if(labelCount == debugLabel){
                        mexPrintf("thisIndY large then Horizon_SCAN, thisIndY: %d, fromIndY: %d, (*iter).second: %d\n", thisIndY, fromIndY, (*iter).second);
                    }*/
                    thisIndY = 0;
                    /*
                    if(labelCount == debugLabel){
                        mexPrintf("thisIndY changed to: %d\n", thisIndY);
                    }*/
                }
                    

                if(false){
                    if(row == debugIndX && col == debugIndY){
                        mexPrintf("(fromIndX, fromIndY) = (%d, %d), (thisIndX, thisIndY) = (%d, %d)\n", fromIndX, fromIndY, thisIndX, thisIndY);
                        mexPrintf("from_x=%f, from_y=%f, from_z=%f, this_x=%f, this_y=%f, this_z=%f\n", ptrRangeMat[(Horizon_SCAN * N_SCAN) * 2 + fromIndY * N_SCAN + fromIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 3 + fromIndY * N_SCAN + fromIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 4 + fromIndY * N_SCAN + fromIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 2 + thisIndY * N_SCAN + thisIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 3 + thisIndY * N_SCAN + thisIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 4 + thisIndY * N_SCAN + thisIndX]);
                        mexPrintf("labelMat=%d\n", labelMat.at<int>(thisIndX, thisIndY));
                    }
                }

                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                // if the current neighborhood point corresponds to the gound poitn or NAN point, continue
                if(groundMat.at<int8_t>(fromIndX, fromIndY) != 0 || groundMat.at<int8_t>(thisIndX, thisIndY) != 0){
                    continue;
                }

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else{
                    double segmentAlphaY = abs(vertical_theta[fromIndX] - vertical_theta[thisIndX]);
                    alpha = segmentAlphaY;
                }

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                #ifdef DEBUG
                    if(false && labelCount == debugLabel){
                        mexPrintf("(fromIndX, fromIndY) = (%d, %d), (thisIndX, thisIndY) = (%d, %d), from_rangeMat=%f, this_rangeMat=%f\n", \
                                                                                                                    fromIndX, \
                                                                                                                    fromIndY, \
                                                                                                                    thisIndX, \
                                                                                                                    thisIndY, \
                                                                                                                    rangeMat.at<float>(fromIndX, fromIndY), \
                                                                                                                    rangeMat.at<float>(thisIndX, thisIndY));
                        mexPrintf("from_x=%f, from_y=%f, from_z=%f, this_x=%f, this_y=%f, this_z=%f\n", ptrRangeMat[(Horizon_SCAN * N_SCAN) * 2 + fromIndY * N_SCAN + fromIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 3 + fromIndY * N_SCAN + fromIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 4 + fromIndY * N_SCAN + fromIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 2 + thisIndY * N_SCAN + thisIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 3 + thisIndY * N_SCAN + thisIndX], \
                                                                                                    ptrRangeMat[(Horizon_SCAN * N_SCAN) * 4 + thisIndY * N_SCAN + thisIndX]);
                        mexPrintf("alpha=%f, angle=%f, d1=%f, d2=%f, from_range=%f, this_range=%f\n", alpha, angle, d1, d2, \
                                                   ptrRangeMat[(Horizon_SCAN * N_SCAN) * 1 + fromIndY * N_SCAN + fromIndX], \
                                                   ptrRangeMat[(Horizon_SCAN * N_SCAN) * 1 + thisIndY * N_SCAN + thisIndX]);
                    }
                #endif

                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        if (allPushedIndSize >= feasibleSegmentValidPointNum)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){
            ++labelCount;
        }else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    void segCloud(){
        // segmentation process
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0){
                    //if(groundMat.at<int8_t>(i, j) == 0){
                        labelComponents(i, j);
                    //}
                }

        int sizeOfSegCloud = 0;
        PointType thisPoint;
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {

            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                thisPoint.x = ptrRangeMat[(Horizon_SCAN * N_SCAN) * 2 + j * N_SCAN + i];
                thisPoint.y = ptrRangeMat[(Horizon_SCAN * N_SCAN) * 3 + j * N_SCAN + i];
                thisPoint.z = ptrRangeMat[(Horizon_SCAN * N_SCAN) * 4 + j * N_SCAN + i];

                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i < groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(thisPoint);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    // save seg cloud
                    segmentedCloud->push_back(thisPoint);
                    // size of seg cloud
                    ++sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }
        
        // extract segmented cloud for visualization
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                    PointType thisPoint;
                    thisPoint.x = ptrRangeMat[(Horizon_SCAN * N_SCAN) * 2 + j * N_SCAN + i];
                    thisPoint.y = ptrRangeMat[(Horizon_SCAN * N_SCAN) * 3 + j * N_SCAN + i];
                    thisPoint.z = ptrRangeMat[(Horizon_SCAN * N_SCAN) * 4 + j * N_SCAN + i];
                    segmentedCloudPure->push_back(thisPoint);
                    segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                }
            }
        }
    }
};