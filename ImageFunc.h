//
//  ImageFunc.h
//  odometry code 3
//
//  Created by Himani Arora on 13/08/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//
#pragma once

#ifndef __odometry_code_3__ImageFunc__
#define __odometry_code_3__ImageFunc__


#include <cstdio>
#include<ctime>
#include <cstdlib>

#include"opencv2/opencv.hpp"
#include<complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "DepthHypothesis.h"
#include "DepthPropagation.h"

#include <fstream>
#include "Frame.h"

using namespace std;
using namespace cv;


vector<float> GetImagePoseEstimate(frame* prev_frame, frame* current_frame, int frame_num, depthMap* currDepthMap,frame* tminus1_prev_frame,float* initial_pose_estimate, bool fromLoopClosure=false, bool homo=false);







#endif /* defined(__odometry_code_3__ImageFunc__) */
