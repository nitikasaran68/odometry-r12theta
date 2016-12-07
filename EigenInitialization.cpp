//
//  EigenInitialization.cpp
//  odometry code 4
//
//  Created by Himani Arora on 04/09/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//

#include "EigenInitialization.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include "ExternVariable.h"

using namespace cv;
using namespace Eigen;
using namespace std;


namespace util
{
    
	Mat K=(Mat_<float>(3,3)<<ORIG_FX,0,ORIG_CX,0,ORIG_FY,ORIG_CY,0,0,1);
	Mat Kinv=K.inv();

	Map<Matrix<float, Dynamic, Dynamic, RowMajor>> K_Eigen(K.ptr<float>(), K.rows, K.cols);
	Map<Matrix<float, Dynamic, Dynamic, RowMajor>> Kinv_Eigen(Kinv.ptr<float>(), Kinv.rows, Kinv.cols);



	/*  float ORIG_FX_INV ;
	 float ORIG_FY_INV ;
	 float ORIG_CX_INV ;
	 float ORIG_CY_INV ; */


	float* Kinv_ptr=Kinv.ptr<float>(0);

	float ORIG_FX_INV = Kinv_ptr[0];
	float ORIG_CX_INV = Kinv_ptr[2];

	float* Kinv_ptr2=Kinv.ptr<float>(1);
	float ORIG_FY_INV = Kinv_ptr2[1];
	float ORIG_CY_INV = Kinv_ptr2[2]; 
    
}
