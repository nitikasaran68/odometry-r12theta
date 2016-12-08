
#pragma once
#ifndef odometry_code_3_ImageClass_h
#define odometry_code_3_ImageClass_h

#include <cstdio>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "ExternVariable.h"

using namespace std;
using namespace cv;

class ImageClass
{
    
public:
    
    Mat image; //image frame/depth matrix
    
    Mat pyramid[util::MAX_PYRAMID_LEVEL]; //image pyramid matrix array from 0 to (Max_pyramid_level-1)
    
    //Constructor ---
    
    ImageClass(void); //default constructor
    
    ImageClass(Mat img); //parameterized constructor
    
    //Destructor ---
    //using default
    
    //Returns number of rows in image at a particular pyramid level
    int GetNumRows(int level);
    
    //Returns number of columns in image at a particular pyramid level
    int GetNumCols(int level);
    
    //Checks whether memory allocated for image is continous or not ---
    bool CheckContinuous(void);
    
    //Returns pointer to image row ---
    uchar* GetPtr2Row(int);
    
    //Makes deep copy of pyramids and frame in another ImageClass object ---
    void ClonePyramid(ImageClass& dest);
    
    //Get pyramid image at particular level
    Mat GetPyramidImage(int level);
    
    //captures frame from video and stores it
    void FrameCapture (VideoCapture vidcap, string type );
    
    
};
#endif
