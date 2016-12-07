//
//  GlobalOptimize.h
//  odometry code 6
//
//  Created by Himani Arora on 19/12/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//

#ifndef __odometry_code_6__GlobalOptimize__
#define __odometry_code_6__GlobalOptimize__

#include <cstdio>
#include<ctime>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include<complex>
#include <cmath>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <fstream>

#include "LoopFrame.h"
#include "Frame.h"
#include "ExternVariable.h"
#include "DepthPropagation.h"
#include "ImageFunc.h"
//#include "ToggleFlags.h"


using namespace cv;

class globalOptimize
{
public:
    
    loopFrame loopFrameArray[util::MAX_LOOP_ARRAY_LENGTH_SCALE_AVG] ;
    loopFrame currentLoopFrame;
    ofstream match_file;
    
    depthMap* temp_depthMap; //for finding connections
    
    bool detectedShortLoopClosure;
    
    bool isloopClosureDetected;
    bool connectionLost;
    bool loopClosureTerminated;
    
    int loopClosureArrayId;
    int lastTestedLoopClosureArrayId;
    int firstTestedLoopClosureArrayId;
    int currentArrayId; //current empty array element
    int nextArrayId;
    
    float matchValue;
    float rms_error;
    float relative_view_angle;
    
    boost::thread_group t_group; //this thread will run parallel to main program thread.
    
    //constants
    int hsize;
    float hrange[2];
    int hist_w;
    int hist_h;
    int bin_w;
    int match_window_beg;
    int match_window_end;
    
    // center for similarity average
    int simavg_center;

    
    void pushToArray(frame* currentframe, depthMap* currentDepthMap);
    bool findMatch(frame* currentframe, bool strayFlag=false);
    void resetArrayElement(int arrayId);
    void triggerRotation(frame* currentframe);
    void findConnection(frame *testFrame);
    void checkConnection(depthMap* currentDepthMap);
    
    void findMatchParallel(frame* currentframe, int thread_num);
    
    globalOptimize(string matchfilepath);
    
    
    
private:
    void calculateImageHistogram(frame* currentframe);
    void showImageHistogram(MatND histogram_normalized, string name);
    double compareImageHistogram(MatND first_histogram_norm, MatND second_histogram_norm);
    void calculateRotationStats(float* pose1_0, float* pose2_0);
    void calculateViewVec(float* pose, float* view_vec);
    void setSimAvgCenter();
    void printLoopArrayStats();
    
    

};




#endif /* defined(__odometry_code_6__GlobalOptimize__) */
