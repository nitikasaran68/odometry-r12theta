//
//  SO3GraphAveraging.h
//  odometry code 6
//
//  Created by Himani Arora on 27/12/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//

#ifndef __odometry_code_6__SO3GraphAveraging__
#define __odometry_code_6__SO3GraphAveraging__

#pragma once

#include <cstdio>
#include<ctime>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include<complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using namespace cv;
using namespace std;

void RotToQuat(Matrix3f Rot, MatrixXf quat)
{
    
}






#endif /* defined(__odometry_code_6__SO3GraphAveraging__) */
