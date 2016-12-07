//
//  LoopFrame.h
//  odometry code 6
//
//  Created by Himani Arora on 19/12/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//

#ifndef __odometry_code_6__LoopFrame__
#define __odometry_code_6__LoopFrame__

#include <cstdio>
#include<ctime>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include<complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "Frame.h"
#include "DepthPropagation.h"

using namespace cv;
using namespace std;

struct loopFrame
{
    Mat image_histogram;
    Mat image;
    int frameId=-1;
    float poseWrtWorld[6];
    float poseWrtOrigin[6];
    
   // bool doLoopMatch=true;
    bool isValid=false;
    bool isStray=false;
    frame* this_frame;
    depthMap* this_currentDepthMap;
    
   // frame* new_frame;
    
};



#endif /* defined(__odometry_code_6__LoopFrame__) */
