//
//  EigenInitialization.h
//  odometry code 4
//
//  Created by Himani Arora on 04/09/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//

#ifndef __odometry_code_4__EigenInitialization__
#define __odometry_code_4__EigenInitialization__

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace Eigen;
using namespace std;


namespace util
{
    
    extern Mat K;
    extern Mat Kinv;
    
    extern Map<Matrix<float, Dynamic, Dynamic, RowMajor>> K_Eigen;
    extern Map<Matrix<float, Dynamic, Dynamic, RowMajor>> Kinv_Eigen;    
    
    extern float* Kinv_ptr;
    extern float ORIG_FX_INV ;
    extern float ORIG_CX_INV ;
    
    extern float* Kinv_ptr2;
    extern float ORIG_FY_INV ;
    extern float ORIG_CY_INV ;

}


#endif /* defined(__odometry_code_4__EigenInitialization__) */
