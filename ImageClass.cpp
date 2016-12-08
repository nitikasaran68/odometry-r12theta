
#include <cstdio>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "ImageClass.h"
#include "ExternVariable.h"

using namespace std;
using namespace cv;


/*
 FUNCTION: IMAGE CLASS (CONSTRUCTOR)
 
 PURPOSE: DEFAULT CONSTRUCTOR
 INPUT : NONE
 OUTPUT : NONE
 */
ImageClass::ImageClass()
{
    //cout<<"\nDefault Constructor";
    //image=Mat::zeros(ORIG_ROWS, ORIG_COLS, CV_8UC1);
}



/*
 FUNCTION: IMAGE CLASS (CONSTRUCTOR)
 
 PURPOSE: INITIALIZES CLASS PRIVATE PARAMETERS
 INPUT : MAT IMAGE FRAME/DEPTH
 OUTPUT : NONE
 */
ImageClass::ImageClass(Mat img)
{
    
   // cout<<"\nUser defined Constructor";
    cvtColor(img, image, CV_BGR2GRAY); //Convert img to Greyscale and stor in image
    pyramid[0] = image;
    
}



/*
 FUNCTION: GET NUM ROWS
 
 PURPOSE: RETURNS NUMBER OF ROWS IN IMG AT PARTICULAR PYRAMID LEVEL
 INPUT : NONE
 OUTPUT: NUMBER OF ROWS
 */
int ImageClass::GetNumRows(int level)
{   return pyramid[level].rows ;
}



/*
 FUNCTION: GET NUM COLS
 
 PURPOSE: RETURNS NUMBER OF COLUMNS IN IMG AT PARTICULAR PYRAMID LEVEL
 INPUT: NONE
 OUTPUT: NUMBER OF ROWS
 */
int ImageClass::GetNumCols(int level)
{   return pyramid[level].cols;
}



/*
 FUNCTION: CHECK CONTINUOUS
 
 PURPOSE: CHECKS IF IMG MATRIX IS CONTINUOUS
 INPUT: NONE
 OUTPUT: TRUE/FALSE
 */
bool ImageClass::CheckContinuous(void)
{
    return image.isContinuous();
}




/*
 FUNCTION: GET PTR 2 ROW
 
 PURPOSE: RETURNS POINTER TO THE DESIRED ROW
 INPUT: ROW NUMBER (MIN=0)
 OUTPUT: POINTER TO ROW
 */
uchar* ImageClass::GetPtr2Row(int r)
{
    return image.ptr<uchar>(r);
    
}



/*
 FUNCTION: CLONE PYRAMID
 
 PURPOSE: COPIES CURRENT OBJECT IMAGE AND PYRAMID TO INPUT OBJECT IMAGE AND PYRAMID
 INPUT: OBJECT OF CLASS TYPE IMAGE CLASS
 OUTPUT: NONE
 */
void ImageClass::ClonePyramid(ImageClass& img)
{
    
    img.image=this->image.clone();
    img.pyramid[0] = img.image.clone(); //Not cloning!, using referance to image

    img.pyramid[1] = this->pyramid[1].clone();
    img.pyramid[2] = this->pyramid[2].clone();
    img.pyramid[3] = this->pyramid[3].clone();
    img.pyramid[4] = this->pyramid[4].clone();
    
}



/*
 FUNCTION: GET PYRAMID IMAGE
 
 PURPOSE: RETURNS PYRAMID IMAGE FOR INPUT LEVEL
 INPUT: PYRAMID LEVEL
 OUTPUT: PYRAMID IMAGE MATRIX */

Mat ImageClass::GetPyramidImage(int level)
{
    return pyramid[level];
}



/*
 FUNCTION: FRAME CAPTURE
 
 PURPOSE: CAPTURES FRAME FROM VIDEO, CONVERTS TO GRAYSCALE AND FORMS IMAGE PYRAMIDS
 INPUT: VIDEOCAPTURE OBJECT
 OUTPUT: NONE
 */
void ImageClass::FrameCapture (VideoCapture vidcap, string type )
{
    vidcap >> image; // get a new frame from bgr
   
    // ConvertImageToGray(image);
    cvtColor(image, image, CV_BGR2GRAY);
    
    if( type=="Depth")
        image=image/20; //adjusting values for depth

    //PopulatePyramid();
    pyramid[0] = image.clone();  
    
    pyrDown(pyramid[0], pyramid[1]);
    pyrDown(pyramid[1], pyramid[2]);
    pyrDown(pyramid[2], pyramid[3]);
    pyrDown(pyramid[3], pyramid[4]);
  
}

