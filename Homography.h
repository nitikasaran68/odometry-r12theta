//
//  Homography.h
//  odometry_depth_estimation_not_pixelwise
//
//  Created by Himanshu Aggarwal on 10/8/15.
//  Copyright (c) 2015 Himanshu Aggarwal. All rights reserved.
//

#ifndef __odometry_depth_estimation_not_pixelwise__Homography__
#define __odometry_depth_estimation_not_pixelwise__Homography__

#include <stdio.h>
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
#include "EigenInitialization.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>


#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <fstream>
#include "Frame.h"

void performHomography(frame* prev_frame,frame* current_frame);

void display_warped_image(Mat homography,frame* prev_frame,frame* current_frame);

void cameraPoseFromHomography(const Mat& H, Mat& pose);

void calculatePoseEssentialMat(frame* prev_frame,frame* current_frame);

#endif /* defined(__odometry_depth_estimation_not_pixelwise__Homography__) */
