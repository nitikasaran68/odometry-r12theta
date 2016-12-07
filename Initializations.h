
#ifndef odometry_code_3_hello_h
#define odometry_code_3_hello_h

#include <cstdio>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "IterationFunc.h"


using namespace std;
using namespace cv;
using namespace Eigen;


//initialize frame and depth matrices
/*
ImageClass current_frame;//stores current frame, invokes default constructor
ImageClass current_depth; //stores current depth map,invokes default constructor

ImageClass prev_frame; //stores prev frame, invokes default constructor
ImageClass prev_depth; //stores prev depth map, invokes default constructor
 */

Mat frame(orig_rows,orig_cols,CV_8UC3);
Mat depth(orig_rows,orig_cols,CV_8UC3);

int nRows, nCols;
int MAX_ITER;




#endif
