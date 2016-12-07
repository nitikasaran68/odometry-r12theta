#pragma once

#ifndef __odometry_code_3__UserDefinedFunc__
#define __odometry_code_3__UserDefinedFunc__

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <cstdio>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "ExternVariable.h"
#include "Frame.h"

using namespace std;
using namespace cv;


//form image mask
Mat FormImageMask(Mat depth_img);

//Find image gradient in a particular direction
void CalculateGradient(Mat Source,Mat& dstx, Mat& desty) ;

//returns steepest descent
Mat CalculateSteepestDescent( frame* image_frame);

//returns hessian
Mat CalculateHessianInverse(Mat steepest_desc);

//returns intrinsic focal parameters
vector<float> GetIntrinsic(int pyrlevel);

Mat CalculateTransformationMatrix(float *pose);

#endif /* defined(__odometry_code_3__UserDefinedFunc__) */
