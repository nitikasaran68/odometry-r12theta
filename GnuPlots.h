//
//  GnuPlots.h
//  odometry code 6b
//
//  Created by Himani Arora on 19/02/16.
//  Copyright (c) 2016 Himani Arora. All rights reserved.
//

#ifndef odometry_code_6b_GnuPlots_h
#define odometry_code_6b_GnuPlots_h

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>


using namespace std;
using namespace cv;

inline void calculateViewVecGnu(float *pose, float& rx, float& ry, float&  rz)
{
    float c = cos(pose[1]);
    float s = sin(pose[1]);
    float r = pose[0];

    Mat SE3 = (Mat_<float>(4, 4) << c,0,-s,-(r*s),0,1,0,0,s,0,c,((r*c)-r),0,0,0,1);

    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    Eigen::MatrixXf SE3_R = SE3_Eigen.block(0, 0, 3, 3);
    
    rx=SE3_R(2,0);
    ry=SE3_R(2,1);
    rz=SE3_R(2,2);
    
    return;
    
}


inline void calculateCoordinateGnu(float *pose, float& px, float& py, float&  pz)
{
    float c = cos(pose[1]);
    float s = sin(pose[1]);
    float r = pose[0];

    Mat SE3 = (Mat_<float>(4, 4) << c,0,-s,-(r*s),0,1,0,0,s,0,c,((r*c)-r),0,0,0,1);

    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    Eigen::MatrixXf Rot = SE3_Eigen.block(0, 0, 3, 3);
    Eigen::MatrixXf Rot_trans = Rot.transpose();
    Eigen::MatrixXf Trans = SE3_Eigen.block(0, 3, 3, 1);
    Eigen::MatrixXf Points = -Rot_trans*Trans;
    
    px=Points(0,0);
    py=Points(1,0);
    pz=Points(2,0);
    
}


inline void plotViewVector(float* pose)
{
    printf("\n");
    system("/opt/local/bin/gnuplot");
    system("set term x11");
    
    float view_x, view_y, view_z;
    
    
    calculateViewVecGnu(pose, view_x, view_y, view_z );

    stringstream ss;
    string str = "set arrow from 0,0,0 to ";
    ss << str << (view_x) << "," << (view_y) << "," << (view_z);

    system("splot \"< echo '0 0 0’ \" ");
    cout << "\nString: " << ss.str();
    
}


inline void plotCamCoordinate(float* pose)
{
    float center_x, center_y, center_z;
    calculateCoordinateGnu(pose, center_x, center_y, center_z );
}


#endif
