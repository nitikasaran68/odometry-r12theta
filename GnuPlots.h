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
#include<ctime>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include<complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>


using namespace std;
using namespace cv;

inline void calculateViewVecGnu(float *pose, float& rx, float& ry, float&  rz)
{
    //Create matrix in OpenCV
    Mat se3=(Mat_<float>(4, 4) << 0,-pose[2],pose[1],pose[3], pose[2],0,-pose[0],pose[4], -pose[1],pose[0],0,pose[5],0,0,0,0);
    
    // Map the OpenCV matrix with Eigen:
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> se3_Eigen(se3.ptr<float>(), se3.rows, se3.cols);
    //cout<<"\nse3: \n"<<se3_Eigen;
    // Take exp in Eigen and store in new Eigen matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> SE3_Eigen = se3_Eigen.exp(); //4x4 pose of Other wrt This (eigen)
    //cout<<"\nafter exp: \n"<<SE3_Eigen;
    
    // create an OpenCV Mat header for the Eigen data:
    Mat SE3(4, 4, CV_32FC1, SE3_Eigen.data()); //4x4 pose of Other wrt This (open cv)
    
    // SE3_R=SE3_Eigen.block(0, 0, 3, 3);
    //cout<<"\nse3 r: \n"<<SE3_R;
    Eigen::MatrixXf SE3_R=SE3_Eigen.block(0, 0, 3, 3);
    
    rx=SE3_R(2,0);
    ry=SE3_R(2,1);
    rz=SE3_R(2,2);
    
    return;
    
}

inline void calculateCoordinateGnu(float *pose, float& px, float& py, float&  pz)
{
    Mat se3=(Mat_<float>(4, 4) << 0,-pose[2],pose[1],pose[3], pose[2],0,-pose[0],pose[4], -pose[1],pose[0],0,pose[5],0,0,0,0);
    
    // Map the OpenCV matrix with Eigen:
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> se3_Eigen(se3.ptr<float>(), se3.rows, se3.cols);
    //cout<<"\nse3: \n"<<se3_Eigen;
    // Take exp in Eigen and store in new Eigen matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> SE3_Eigen = se3_Eigen.exp(); //4x4 pose of Other wrt This (eigen)
    Eigen::MatrixXf Rot=SE3_Eigen.block(0, 0, 3, 3);
    Eigen::MatrixXf Rot_trans=Rot.transpose();
    Eigen::MatrixXf Trans=SE3_Eigen.block(0, 3, 3, 1);
    Eigen::MatrixXf Points=-Rot_trans*Trans;
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
    ss<<str<<(view_x)<<","<<(view_y)<<","<<(view_z);
    //system(ss.str());
    system("splot \"< echo '0 0 0’ \" ");
    cout<<"\nString: "<<ss.str();
    
}

inline void plotCamCoordinate(float* pose)
{
    float center_x, center_y, center_z;
    calculateCoordinateGnu(pose, center_x, center_y, center_z );
    
}




#endif
