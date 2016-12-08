#pragma once

#ifndef __odometry_code_3__IterationFunc__
#define __odometry_code_3__IterationFunc__

#include <cstdio>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "ExternVariable.h"
//#include <Eigen/Dense>
//#include <unsupported/Eigen/MatrixFunctions>
#include "Frame.h"

using namespace std;
using namespace cv;

//Calculate world points
void CalculateWorldPoints(Mat& saveimg, Mat& worldpoint, frame* image_frame);

//Calculate warped points for pose vector
Mat CalculateWarpedPoints (float *pose, Mat worldpoint,frame* image_frame);

//Calculate warped image with inteerpolated intensity values
Mat CalculateWArpedImage(Mat warpedpoint, frame* current_frame,frame* prev_frame);

//Update pose estimate and check Iteration Loop Termination Condition
int UpdatePose(Mat& residual, Mat saveimg, Mat warpedimg, Mat steepdesc, Mat hessian, float* pose, int no_nonzero_mask);

#endif /* defined(__odometry_code_3__IterationFunc__) */







