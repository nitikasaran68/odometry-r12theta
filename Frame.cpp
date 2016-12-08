
//
//Frame.cpp
//

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include "Frame.h"
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "ExternVariable.h"
#include "DisplayFunc.h"
#include "UserDefinedFunc.h"
#include "EigenInitialization.h"

//using namespace cv;

static int numberOfInstances;
using namespace std;
using namespace cv;

frame::frame() //default constructor
{
    PRINTF("\nDeafult Constructor for Frame Id: %d", frameId)
}


//constructor that is actually used. It reads incoming image and stores it
frame::frame(VideoCapture vidcap_rgb)
{

    //frame iId allocated, starts from frameId = 1
    frameId=++numberOfInstances; 
    
    
    PRINTF("\nConstructing Frame with Frame ID: %d", frameId);
    
    deptharrfirstcall=false;
    deptharrpyrfirstcall=false;
    
    no_points_substantial_grad=0;

    Mat big_image;
    vidcap_rgb>>big_image;
    
   //image re-sized here only to reduce computation. This is not related to pyramid levels.
   //image after re-sizing here becomes the max resolution image that will be used thereafter
   resize(big_image, rgb_image, Size(),util::RESIZE_FACTOR,util::RESIZE_FACTOR);
    
    //make rgb image as B&W
    //comment out if your input iamge is already B&W
    cvtColor(big_image, big_image, CV_BGR2GRAY);
    
   //undistorts image. Undsitortion parameters are hard-coded here
   if(util::FLAG_DO_UNDISTORTION)
   {
    
        Mat cam_K=(Mat_<float>(3,3)<<util::ORIG_FX*util::INTRINSIC_FACTOR, 0.0f, util::ORIG_CX*util::DIM_FACTOR, 0.0f, util::ORIG_FY*util::INTRINSIC_FACTOR, util::ORIG_CY*util::DIM_FACTOR, 0.0f, 0.0f ,1.0f);
        Mat cam_distort=(Mat_<float>(1,4)<<-0.288283f, 0.146546f, 0.003800f, -0.001690f, -0.132134f); //hard-coded parameters
        
        Mat big_image2;
        //printf("\nsize original row,col= %d,%d", image.rows, image.cols);
        
        Mat cam_Knew=getOptimalNewCameraMatrix(cam_K, cam_distort, big_image.size(), double(0.0f));
        
        //Mat image_undistort;
        undistort(big_image, big_image2, cam_K, cam_distort , cam_Knew);
        
        
        resize(big_image2,image, Size(),util::RESIZE_FACTOR,util::RESIZE_FACTOR);
        //imshow("Current Frame", image);
        //waitKey(2000);
           
       
   }
  //  image=big_image.clone();

    
    else
    {
        resize(big_image, image, Size(),util::RESIZE_FACTOR,util::RESIZE_FACTOR);
    }
    /*
    pyrDown(big_image2, big_image2);
    pyrDown(big_image2, image);
    */
    
/*
    vidcap_rgb >>image;    
   cvtColor(image, image, CV_BGR2GRAY);
*/

    width=image.cols;
    height=image.rows;
    
    //printf("\nSIZE rows %d, cols %d", height, width);
    pyrLevel=0;
    
    currentCols=width;
    currentRows=height; 

    //initialize to 0 
    poseWrtOrigin[0]=0.0f; 
    poseWrtOrigin[1]=0.0f;
    poseWrtOrigin[2]=0.0f;
    
    
    poseWrtWorld[0]=0.0f;
    poseWrtWorld[1]=0.0f;
    poseWrtWorld[2]=0.0f;
    
    rescaleFactor=1.0f;
    
    //make image pyramids
    constructImagePyramids();
    
    
    calculateGradient();
    buildMaxGradients();
    
    //initializing depth map to 0
    depth=Mat::zeros(height,width,CV_32FC1); 
    
    depth_pyramid[0]=Mat::zeros(height, width, CV_32FC1);
    depth_pyramid[1]=Mat::zeros(height>>1, width>>1, CV_32FC1);
    depth_pyramid[2]=Mat::zeros(height>>2, width>>2, CV_32FC1);
    depth_pyramid[3]=Mat::zeros(height>>3, width>>3, CV_32FC1);
    
    weight_pyramid[0]=Mat::zeros(height, width, CV_32FC1);
    weight_pyramid[1]=Mat::zeros(height>>1, width>>1, CV_32FC1);
    weight_pyramid[2]=Mat::zeros(height>>2, width>>2, CV_32FC1);
    weight_pyramid[3]=Mat::zeros(height>>3, width>>3, CV_32FC1);
    
    numWeightsAdded[0]=0;
    numWeightsAdded[1]=0;
    numWeightsAdded[2]=0;
    numWeightsAdded[3]=0;

}

/*
frame::frame(Mat imagecap, int input_frameId)
{
    
    frameId=input_frameId;
    
    
    PRINTF("\nConstructing Frame with Frame ID: %d", frameId);
    
    deptharrfirstcall=false;
    deptharrpyrfirstcall=false;
    
    no_points_substantial_grad=0;
   
   
    image=imagecap;
    
    width=image.cols;
    height=image.rows;
    
    //printf("\nSIZE rows %d, cols %d", height, width);
    pyrLevel=0;
    
    currentCols=width;
    currentRows=height;
    poseWrtOrigin[0]=0.0f; //initialize to 0
    poseWrtOrigin[1]=0.0f;
    poseWrtOrigin[2]=0.0f;
    poseWrtOrigin[3]=0.0f;
    poseWrtOrigin[4]=0.0f;
    poseWrtOrigin[5]=0.0f;
    
    
    poseWrtWorld[0]=0.0f;
    poseWrtWorld[1]=0.0f;
    poseWrtWorld[2]=0.0f;
    poseWrtWorld[3]=0.0f;
    poseWrtWorld[4]=0.0f;
    poseWrtWorld[5]=0.0f;
    
    rescaleFactor=1.0f;
    
    //make image pyramids
    constructImagePyramids();
    
    //parentKeyframeId=frameId;
    
    calculateGradient();
    buildMaxGradients();
    
    depth=Mat::zeros(height,width,CV_32FC1); //initializing depth map to 0
    
    depth_pyramid[0]=Mat::zeros(height, width, CV_32FC1);
    depth_pyramid[1]=Mat::zeros(height>>1, width>>1, CV_32FC1);
    depth_pyramid[2]=Mat::zeros(height>>2, width>>2, CV_32FC1);
    depth_pyramid[3]=Mat::zeros(height>>3, width>>3, CV_32FC1);
    depth_pyramid[4]=Mat::zeros(height>>4, width>>4, CV_32FC1);
}
*/ 


frame::frame(VideoCapture vidcap_rgb,VideoCapture vidcap_depth)
{
    
    
    frameId=++numberOfInstances;
    
    vidcap_rgb >> image; // get a new frame from bgr
    
    // ConvertImageToGray(image);
    cvtColor(image, image, CV_BGR2GRAY);
    
    Mat depth2= Mat::zeros(util::ORIG_ROWS, util::ORIG_COLS, CV_32FC1);
    
    vidcap_depth >> depth2; // get a new frame from bgr
    
    // ConvertImageToGray(image);
    cvtColor(depth2, depth2, CV_BGR2GRAY);
    
    depth2.convertTo(depth, CV_32FC1);
    
    depth=depth/20.0f;
    
    width=image.cols;
    height=image.rows;
    
    pyrLevel=0;
    
    
    currentCols=width;
    currentRows=height;

    //initialize to 0
    poseWrtOrigin[0]=0.0f; 
    poseWrtOrigin[1]=0.0f;
    poseWrtOrigin[2]=0.0f;
    
 
    //make image pyramids
    constructImagePyramids();
   // constructDepthPyramids();
    
    
    calculateGradient();
    buildMaxGradients();
    
}




void frame::constructImagePyramids()
{   
    PRINTF("\nConstructing Image Pyramids for Frame Id: %d", frameId);

    image_pyramid[0] = image.clone();
    
    pyrDown(image_pyramid[0], image_pyramid[1]);
    pyrDown(image_pyramid[1], image_pyramid[2]);
    pyrDown(image_pyramid[2], image_pyramid[3]);
    //pyrDown(image_pyramid[3], image_pyramid[4]);
}


void frame::calculateGradient()
{
    PRINTF("\nConstructing Gradient function 2 for frame: %d and pyramid level: %d", frameId, pyrLevel);
    
    //float kernelarr = { -1, 0, 1 };
    /*
    Mat kernelx = (Mat_<float>(1, 3)<< -1.0f,0,1.0f);
    // Gradient X
    
    //Sobel(src, dst, ddepth=16S, dx=1, dy=0, ksize=1 (for no smoothing), scale=1,delta=0, borderType=BORDER_DEFAULT )∂
    filter2D(image_pyramid[pyrLevel], gradientx, CV_32FC1, kernelx); 
    
    //float kernelarr = { -1, 0, 1 };
    Mat kernely = (Mat_<float>(3,1)<< -1.0f,0,1.0f);
    //Gradient Y
    //Sobel(src, dst, ddepth=16S, dx=0, dy=1, ksize=1 (for no smoothing), scale=1,delta=0, borderType=BORDER_DEFAULT )∂
    filter2D(image_pyramid[pyrLevel], gradienty, CV_32FC1, kernely); 
     */
    
    gradientx=Mat::zeros(currentRows, currentCols, CV_32FC1);
    gradienty=Mat::zeros(currentRows, currentCols, CV_32FC1);

    
    uchar* img_ptr=image_pyramid[pyrLevel].ptr<uchar>(0);
    uchar* img_ptr_top=image_pyramid[pyrLevel].ptr<uchar>(0);
    uchar* img_ptr_bottom=image_pyramid[pyrLevel].ptr<uchar>(0);

    float* gradx_ptr=gradientx.ptr<float>(0);
    float* grady_ptr=gradienty.ptr<float>(0);
    
    
    int y,x;
    
    //CASE 1
    
    for(y=1; y<currentRows-1; y++) // for internal elements with border size =1
    {
        img_ptr=image_pyramid[pyrLevel].ptr<uchar>(y);
        gradx_ptr=gradientx.ptr<float>(y);
        grady_ptr=gradienty.ptr<float>(y);
        img_ptr_top=image_pyramid[pyrLevel].ptr<uchar>(y-1);
        img_ptr_bottom=image_pyramid[pyrLevel].ptr<uchar>(y+1);

        for(x=1; x<currentCols-1; x++)
        {
            gradx_ptr[x]=0.5f*(float(img_ptr[x+1])-float(img_ptr[x-1]));
            grady_ptr[x]=0.5f*(float(img_ptr_bottom[x])-float(img_ptr_top[x]));
        }
    }
        
        //CASE 2
        y=0; //top row
        img_ptr=image_pyramid[pyrLevel].ptr<uchar>(y);
        gradx_ptr=gradientx.ptr<float>(y);
        grady_ptr=gradienty.ptr<float>(y);
        img_ptr_bottom=image_pyramid[pyrLevel].ptr<uchar>(y+1);
    
        x=0; //left top corner element
        gradx_ptr[x]=(float(img_ptr[x+1])-float(img_ptr[x]));//
        grady_ptr[x]=(float(img_ptr_bottom[x])-float(img_ptr[x]));//

        for(x=1; x<currentCols-1; x++) //top row , starting from 2nd element upto 2nd last
        {   gradx_ptr[x]=0.5f*(float(img_ptr[x+1])-float(img_ptr[x-1]));
            grady_ptr[x]=(float(img_ptr_bottom[x])-float(img_ptr[x]));//
        }
        
        x=currentCols-1; //right top corner element
        gradx_ptr[x]=(float(img_ptr[x])-float(img_ptr[x-1]));//
        grady_ptr[x]=(float(img_ptr_bottom[x])-float(img_ptr[x]));//
        
        //CASE 3
        y=currentRows-1; //bottom row
        img_ptr=image_pyramid[pyrLevel].ptr<uchar>(y);
        gradx_ptr=gradientx.ptr<float>(y);
        grady_ptr=gradienty.ptr<float>(y);
        img_ptr_top=image_pyramid[pyrLevel].ptr<uchar>(y-1);
        
        x=0; //left bottom corner element
        gradx_ptr[x]=(float(img_ptr[x+1])-float(img_ptr[x]));//
        grady_ptr[x]=(float(img_ptr[x])-float(img_ptr_top[x]));//
        
        for(x=1; x<currentCols-1; x++) //bottom row , starting from 2nd element upto 2nd last
        {   gradx_ptr[x]=0.5f*(float(img_ptr[x+1])-float(img_ptr[x-1]));
            grady_ptr[x]=(float(img_ptr[x])-float(img_ptr_top[x]));//
        }
        
        x=currentCols-1; //right bottom corner element
        gradx_ptr[x]=(float(img_ptr[x])-float(img_ptr[x-1]));//
        grady_ptr[x]=(float(img_ptr[x])-float(img_ptr_top[x]));//
        
        //CASE 4
        
        for(y=1; y<currentRows-1;y++) //left border and right border, starting from 2nd element upto 2nd last
        {
            img_ptr=image_pyramid[pyrLevel].ptr<uchar>(y);
            
            gradx_ptr=gradientx.ptr<float>(y);
            grady_ptr=gradienty.ptr<float>(y);
            
            img_ptr_top=image_pyramid[pyrLevel].ptr<uchar>(y-1);
            img_ptr_bottom=image_pyramid[pyrLevel].ptr<uchar>(y+1);
            
            x=0;    //left border
            gradx_ptr[x]=(float(img_ptr[x+1])-float(img_ptr[x]));//
            grady_ptr[x]=0.5f*(float(img_ptr_bottom[x])-float(img_ptr_top[x]));
            
            x=currentCols-1; //right border
            gradx_ptr[x]=(float(img_ptr[x])-float(img_ptr[x-1]));//
            grady_ptr[x]=0.5f*(float(img_ptr_bottom[x])-float(img_ptr_top[x]));
            
            
            
        }
       
  //  printf("\nlevel %d, currentRows %d, currentCols %d", pyrLevel, currentRows, currentCols);
   // cout<<"GRAD x \n"<<gradientx;
   // cout<<"\n\n\n\nGRAD y\n"<<gradienty;
    //waitKey(0);
    
}

void frame::getImgSize(int& row, int& col,int level)
{
    PRINTF("\nCalculating Image Size for Frame Id: %d and Pyramid Level: %d", frameId, level);
    row=height/pow(2, level);
    col=width/pow(2, level);

}

void frame::calculateNonZeroDepthPts()
{
    PRINTF("\nCalculating Mask and Non-Zero Depth Points for Frame Id: %d", frameId);
    mask=depth_pyramid[pyrLevel]>0.0f;
    no_nonZeroDepthPts=countNonZero(mask);
    //cout<<"\nNON ZERO: "<<no_nonZeroDepthPts;
}

void frame::initializePose()
{
    PRINTF("\nInitializing Pose Wrt Origin for Frame Id: %d", frameId);
    poseWrtOrigin[0]=0.0f;
    poseWrtOrigin[1]=0.0f;
    poseWrtOrigin[2]=0.0f;

}

//updates size-related image varaibles that change when pyramid levels change
void frame::updationOnPyrChange(int level,bool isPrevious)
{
    PRINTF("\nUpdating parameters on Pyramid change for Frame Id: %d, level: %d", frameId, level);
    pyrLevel=level;
    
    currentRows=height/pow(2, level);
    currentCols=width/pow(2, level);

    if(isPrevious)calculateNonZeroDepthPts(); //calculating new mask
    calculateGradient();

}

void frame::calculatePoseWrtOrigin(frame *prev_image, float *poseChangeWrtPrevframe,bool frmhomo)
{
    //PRINTF("\nCalculating Pose Wrt Origin for Frame Id: %d using pose change Wrt Prev Frame with Frame Id: %d", frameId, prev_image->frameId);
    //PRINTF("\nother_frame->poseWrtOrigin: %f ", prev_image->poseWrtOrigin[0] );
    
    if(!frmhomo)
    {
     
        concatenateRelativePose(poseChangeWrtPrevframe, prev_image->poseWrtOrigin, poseWrtOrigin);
            
        
        // poseWrtOrigin[0]=prev_image->poseWrtOrigin[0]+poseChangeWrtPrevframe[0];
        // poseWrtOrigin[1]=prev_image->poseWrtOrigin[1]+poseChangeWrtPrevframe[1];
        // poseWrtOrigin[2]=prev_image->poseWrtOrigin[2]+poseChangeWrtPrevframe[2];
        // poseWrtOrigin[3]=prev_image->poseWrtOrigin[3]+poseChangeWrtPrevframe[3];
        // poseWrtOrigin[4]=prev_image->poseWrtOrigin[4]+poseChangeWrtPrevframe[4];
        // poseWrtOrigin[5]=prev_image->poseWrtOrigin[5]+poseChangeWrtPrevframe[5];
           
    }

    else
    { 
        poseWrtOrigin[0]=prev_image->poseWrtOrigin[0]+poseChangeWrtPrevframe[0];
        poseWrtOrigin[1]=prev_image->poseWrtOrigin[1]+poseChangeWrtPrevframe[1];
        poseWrtOrigin[2]=prev_image->poseWrtOrigin[2]+poseChangeWrtPrevframe[2];
    }
    
    return;
}


void frame::calculatePoseWrtWorld(frame *prev_image, float *poseChangeWrtPrevframe,bool frmhomo)
{
    //PRINTF("\nCalculating Pose Wrt Origin for Frame Id: %d using pose change Wrt Prev Frame with Frame Id: %d", frameId, prev_image->frameId);
    //PRINTF("\nother_frame->poseWrtOrigin: %f ", prev_image->poseWrtOrigin[0] );
    
    if(!frmhomo){
        // poseWrtWorld[0]=prev_image->poseWrtWorld[0]+poseChangeWrtPrevframe[0];
        // poseWrtWorld[1]=prev_image->poseWrtWorld[1]+poseChangeWrtPrevframe[1];
        // poseWrtWorld[2]=prev_image->poseWrtWorld[2]+poseChangeWrtPrevframe[2];
        // poseWrtWorld[3]=prev_image->poseWrtWorld[3]+poseChangeWrtPrevframe[3];
        // poseWrtWorld[4]=prev_image->poseWrtWorld[4]+poseChangeWrtPrevframe[4];
        // poseWrtWorld[5]=prev_image->poseWrtWorld[5]+poseChangeWrtPrevframe[5];
        
        concatenateRelativePose(poseChangeWrtPrevframe, prev_image->poseWrtWorld, poseWrtWorld);
    }
    
    else
    {
        poseWrtWorld[0]=prev_image->poseWrtWorld[0]+poseChangeWrtPrevframe[0];
        poseWrtWorld[1]=prev_image->poseWrtWorld[1]+poseChangeWrtPrevframe[1];
        poseWrtWorld[2]=prev_image->poseWrtWorld[2]+poseChangeWrtPrevframe[2];
    }
    
    return;
}


//USE THIS!!!
//calculates pose of other frame w.r.t this frame using their absolute poses. Stores result in variables of this frame to be accessed later
void frame::calculateSE3poseOtherWrtThis(frame *other_frame) // this to other 
{
    PRINTF("\nCalculating SE3 pose for This Frame with Id: %d Wrt to Other Frame with Id: %d", frameId, other_frame->frameId);
    float poseWrtThis[3];
    
    concatenateOriginPose(other_frame->poseWrtOrigin, poseWrtOrigin, poseWrtThis);

    float c = cos(poseWrtThis[2]);
    float s = sin(poseWrtThis[2]);
    float r1 = poseWrtThis[0];
    float r2 = poseWrtThis[1]; 
    
    Mat SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    // updating pose matrices

    SE3poseOtherWrtThis = SE3_Eigen;
    SE3poseOtherWrtThis_r = SE3poseOtherWrtThis.block(0, 0, 3, 3);  //Rotation 3X3
    SE3poseOtherWrtThis_t = SE3poseOtherWrtThis.block(0, 3, 3, 1);  //Translation 3X1
    
    SE3poseThisWrtOther = SE3poseOtherWrtThis.inverse();
    SE3poseThisWrtOther_r = SE3poseThisWrtOther.block(0, 0, 3, 3);
    SE3poseThisWrtOther_t = SE3poseThisWrtOther.block(0, 3, 3, 1);

    K_SE3poseThisWrtOther_r = util::K_Eigen*SE3poseThisWrtOther_r;
    K_SE3poseThisWrtOther_t = util::K_Eigen*SE3poseThisWrtOther_t;
    
    K_SE3poseOtherWrtThis_r = util::K_Eigen*SE3poseOtherWrtThis_r;
    K_SE3poseOtherWrtThis_t = util::K_Eigen*SE3poseOtherWrtThis_t;
    
}




//not sure if accurate. It is not really used
void frame::calculateSim3poseOtherWrtThis(float scale_factor)
{
    PRINTF("\nCalculating Sim3 pose for This Frame with Id: %d Wrt to Other Frame with recale factor: %f", frameId, scale_factor);

    rescaleFactor = scale_factor; 
    
    Eigen::Matrix3f scale_mat;
    scale_mat<<scale_factor, 0, 0, 0, scale_factor, 0, 0, 0, scale_factor;
    
    SE3poseThisWrtOther_r = scale_mat*SE3poseThisWrtOther_r;    
    SE3poseOtherWrtThis_r = SE3poseThisWrtOther_r.inverse();
    
    K_SE3poseThisWrtOther_r = util::K_Eigen*SE3poseThisWrtOther_r;
    K_SE3poseOtherWrtThis_r = util::K_Eigen*SE3poseOtherWrtThis_r;
    
    //translation remains the same
    
    SE3poseThisWrtOther<<SE3poseThisWrtOther_r(0,0), SE3poseThisWrtOther_r(0,1), SE3poseThisWrtOther_r(0,2), SE3poseThisWrtOther_t(0,0),SE3poseThisWrtOther_r(1,0), SE3poseThisWrtOther_r(1,1), SE3poseThisWrtOther_r(1,2), SE3poseThisWrtOther_t(1,0), SE3poseThisWrtOther_r(2,0), SE3poseThisWrtOther_r(2,1), SE3poseThisWrtOther_r(2,2), SE3poseThisWrtOther_t(2,0), 0 , 0 , 0 , 1;
    
    SE3poseOtherWrtThis=SE3poseThisWrtOther.inverse();
    
   // K_SE3poseOtherWrtThis=util::K_Eigen*SE3poseOtherWrtThis;
   // K_SE3poseThisWrtOther=util::K_Eigen*SE3poseThisWrtOther;
    
}


//converts se3 poseWrtWorld to SE3 form to get R and T matrices that are w.r. World
void frame::calculateRandT()
{
    float c = cos(poseWrtWorld[2]);
    float s = sin(poseWrtWorld[2]);
    float r1 = poseWrtWorld[0];
    float r2 = poseWrtWorld[1];

    Mat SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Pose(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    Eigen::Matrix3f scale_mat;
    scale_mat<<rescaleFactor, 0, 0, 0, rescaleFactor, 0, 0, 0, rescaleFactor;
    
    this->SE3_Pose = SE3_Pose.block(0, 0, 4, 4);
    
    SE3_R = SE3_Pose.block(0, 0, 3, 3);
    //cout<<"\nse3 r: \n"<<SE3_R;
    SE3_T = SE3_Pose.block(0, 3, 3, 1);
   // cout<<"\nse3 t: \n"<<SE3_T;
    Sim3_R = scale_mat*SE3_R;
    //cout<<"\nsim3 r: "<<Sim3_R;
    
}


void frame::calculateRelativeRandT(float* relPose)
{

    float c = cos(relPose[2]);
    float s = sin(relPose[2]);
    float r1 = relPose[0];
    float r2 = relPose[1];

    Mat SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Pose(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    Eigen::Matrix3f scale_mat;
    scale_mat<<rescaleFactor, 0, 0, 0, rescaleFactor, 0, 0, 0, rescaleFactor;
    
    this->SE3_Pose=SE3_Pose.block(0, 0, 4, 4);
    
    SE3_R=SE3_Pose.block(0, 0, 3, 3);
    //cout<<"\nse3 r: \n"<<SE3_R;
    SE3_T=SE3_Pose.block(0, 3, 3, 1);
    // cout<<"\nse3 t: \n"<<SE3_T;
    Sim3_R=scale_mat*SE3_R;

}

//concatenates poses using relative poses
void frame::concatenateRelativePose(float *src_1wrt2, float *src_2wrt3, float *dest_1wrt3)
{
    float c = cos(src_1wrt2[2]);
    float s = sin(src_1wrt2[2]);
    float r1 = src_1wrt2[0];
    float r2 = src_1wrt2[1];

    Mat src_1wrt2_SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> src_1wrt2_SE3Eigen(src_1wrt2_SE3.ptr<float>(), src_1wrt2_SE3.rows, src_1wrt2_SE3.cols);

    c = cos(src_2wrt3[2]);
    s = sin(src_2wrt3[2]);
    r1 = src_2wrt3[0];
    r2 = src_2wrt3[1];

    Mat src_2wrt3_SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> src_2wrt3_SE3Eigen(src_2wrt3_SE3.ptr<float>(), src_2wrt3_SE3.rows, src_2wrt3_SE3.cols);

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dest_1wrt3_SE3Eigen = src_1wrt2_SE3Eigen * src_2wrt3_SE3Eigen;
        
    dest_1wrt3[0] = src_1wrt2[0];
    dest_1wrt3[1] = src_2wrt3[1];
    dest_1wrt3[2] = src_1wrt2[2] + src_2wrt3[2];

    c = cos(dest_1wrt3[2]);
    s = sin(dest_1wrt3[2]);
    r1 = dest_1wrt3[0];
    r2 = dest_1wrt3[1];

    Mat dest_1wrt3_SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> dest_1wrt3_SE3Eigen2(dest_1wrt3_SE3.ptr<float>(), dest_1wrt3_SE3.rows, dest_1wrt3_SE3.cols);

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> diff = dest_1wrt3_SE3Eigen2 - dest_1wrt3_SE3Eigen;
    float e = diff.norm();
    float d = dest_1wrt2_SE3Eigen2.norm()


    if(e>0){
        printf("CONCATENATE REL ERROR: %f%%\n",(e/D)*100);
    }

}


//concatenates poses using world poses
void frame::concatenateOriginPose(float *src_1wrt0, float *src_2wrt0, float *dest_1wrt2)
{

    float c = cos(src_1wrt0[2]);
    float s = sin(src_1wrt0[2]);
    float r1 = src_1wrt0[0];
    float r2 = src_1wrt0[1];

    Mat src_1wrt0_SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> src_1wrt0_SE3Eigen(src_1wrt0_SE3.ptr<float>(), src_1wrt0_SE3.rows, src_1wrt0_SE3.cols);

    c = cos(src_2wrt0[2]);
    s = sin(src_2wrt0[2]);
    r1 = src_2wrt0[0];
    r2 = src_2wrt0[1];

    Mat src_2wrt0_SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> src_2wrt0_SE3Eigen(src_2wrt0_SE3.ptr<float>(), src_2wrt0_SE3.rows, src_2wrt0_SE3.cols);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> src_0wrt2_SE3Eigen = src_2wrt0_SE3Eigen.inverse();

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dest_1wrt2_SE3Eigen = src_1wrt0_SE3Eigen * src_0wrt2_SE3Eigen;
        
    dest_1wrt2[0] = src_1wrt0[0];
    dest_1wrt2[1] = src_2wrt0[0];
    dest_1wrt2[2] = src_1wrt0[2] - src_2wrt0[2];

    c = cos(dest_1wrt2[2]);
    s = sin(dest_1wrt2[2]);
    r1 = dest_1wrt2[0];
    r2 = dest_1wrt2[1];

    Mat dest_1wrt2_SE3=(Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> dest_1wrt2_SE3Eigen2(dest_1wrt2_SE3.ptr<float>(), dest_1wrt2_SE3.rows, dest_1wrt2_SE3.cols);

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> diff = dest_1wrt2_SE3Eigen2 - dest_1wrt2_SE3Eigen;
    float e = diff.norm();
    float d = dest_1wrt2_SE3Eigen2.norm()

    if(e>0){
        printf("CONCATENATE ORIGIN ERROR: %f%%\n",(e/d)*100);
    }
    
}


void frame::calculateInvLiePose(float *pose)
{
    float c = cos(pose[2]);
    float s = sin(pose[2]);
    float r1 = pose[0];
    float r2 = pose[1];

    Mat SE3 = (Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> SE3_Eigen_inv = SE3_Eigen.inverse();
    
    float temp = pose[0];
    pose[0] = pose[1];
    pose[1] = temp;
    pose[2] = -pose[2];

    c = cos(pose[2]);
    s = sin(pose[2]);
    r1 = pose[0];
    r2 = pose[1];

    SE3 = (Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen_inv2(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> diff = SE3_Eigen_inv2 - SE3_Eigen_inv;
    float e = diff.norm();
    float d = SE3_Eigen_inv2.norm()

    if(e>0){
        printf("INVERSE ERROR: %f%%\n",(e/d)*100);
    }

}


void frame::calculateInvLiePose(float *posesrc, float *posedest)
{
    float c = cos(posesrc[2]);
    float s = sin(posesrc[2]);
    float r1 = posesrc[0];
    float r2 = posesrc[1];

    Mat SE3 = (Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> SE3_Eigen_inv = SE3_Eigen.inverse();
    
    posedest[0] = posesrc[1];
    posedest[1] = posesrc[0];
    posedest[2] = -posesrc[2];

    c = cos(posedest[2]);
    s = sin(posedest[2]);
    r1 = posedest[0];
    r2 = posedest[1];

    SE3 = (Mat_<float>(4, 4) << c,0,-s,-(r1*s),0,1,0,0,s,0,c,((r1*c)-r2),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen_inv2(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> diff = SE3_Eigen_inv2 - SE3_Eigen_inv;
    float e = diff.norm();
    float d = SE3_Eigen_inv2.norm()

    if(e>0){
        printf("INVERSE ERROR: %f%%\n",(e/d)*100);
    }
    
}


//finds maximum gradient, used in depth est.
void frame::buildMaxGradients()
{
    PRINTF("\nCalculating Max Gradients for frame Id: %d", frameId);
    // 1. write abs gradients in real data.
    no_points_substantial_grad=0;
    
    maxAbsGradient=Mat::zeros(height,width,CV_32FC1);
    
    
    Mat sqrGradX;
    Mat sqrGradY;
    multiply(gradientx,gradientx,sqrGradX);
    multiply(gradienty,gradienty,sqrGradY);
    
    add(sqrGradX, sqrGradY, maxAbsGradient);
    sqrt(maxAbsGradient,maxAbsGradient);
    
    /*
    float* maxabs_grad_pt=maxAbsGradient.ptr<float>(0);
    
    float* gradx_ptr=gradientx.ptr<float>(0);
    
    float* grady_ptr=gradienty.ptr<float>(0);
    
    for(int y=0; y< util::ORIG_ROWS;y++)
    {
        maxabs_grad_pt=maxAbsGradient.ptr<float>(y);
        
        gradx_ptr=gradientx.ptr<float>(y);
        
        grady_ptr=gradienty.ptr<float>(y);
        
        for(int x=0;x<util::ORIG_COLS;x++)
        {
            maxabs_grad_pt[x]=(gradx_ptr[x]*gradx_ptr[x])+(grady_ptr[x]*grady_ptr[x]);
            maxabs_grad_pt[x]=sqrt(maxabs_grad_pt[x]);
        }
    }
    */

    Mat maxGradTemp=Mat::zeros(height,width,CV_32FC1);
    
    // 2. smear up/down direction into temp buffer
    float* maxgrad_centre_pt=maxAbsGradient.ptr<float>(0);
    float* maxgrad_up_pt=maxAbsGradient.ptr<float>(0);
    float* maxgrad_down_pt=maxAbsGradient.ptr<float>(0);
    float* maxgrad_t_pt=maxGradTemp.ptr<float>(0);
    
    for (int y=1;y<height-1; y++)
    {
        maxgrad_centre_pt=maxAbsGradient.ptr<float>(y);
        maxgrad_up_pt=maxAbsGradient.ptr<float>(y-1);
        maxgrad_down_pt=maxAbsGradient.ptr<float>(y+1);
        maxgrad_t_pt=maxGradTemp.ptr<float>(y);
        for (int x=0; x<width; x++)
        {
            float g1=max(maxgrad_centre_pt[x],maxgrad_up_pt[x]);
            maxgrad_t_pt[x]=max(g1,maxgrad_down_pt[x]);
        }

    }
    
    // 2. smear left/right direction into real data
    for (int y=1;y<height-1; y++)
    {
        maxgrad_centre_pt=maxAbsGradient.ptr<float>(y);
        
        maxgrad_t_pt=maxGradTemp.ptr<float>(y);
        for (int x=1; x<width-1; x++)
        {
            float g1=max(maxgrad_t_pt[x-1],maxgrad_t_pt[x]);
            maxgrad_centre_pt[x]=max(g1,maxgrad_t_pt[x+1]);
            if(maxgrad_centre_pt[x]>=util::MIN_ABS_GRAD_DECREASE)
                no_points_substantial_grad++;
        }
        
    }
    
    //cout<<"\n\n\n"<<maxAbsGradient;

    /* // 2. smear up/down direction into temp buffer
    maxgrad_pt = data.maxGradients[level] + width+1;
    maxgrad_pt_max = data.maxGradients[level] + width*(height-1)-1;
    float* maxgrad_t_pt = maxGradTemp + width+1;
    for(;maxgrad_pt<maxgrad_pt_max; maxgrad_pt++, maxgrad_t_pt++)
    {
        float g1 = maxgrad_pt[-width];
        float g2 = maxgrad_pt[0];
        if(g1 < g2) g1 = g2;
        float g3 = maxgrad_pt[width];
        if(g1 < g3)
            *maxgrad_t_pt = g3;
        else
            *maxgrad_t_pt = g1;
    }
    
    float numMappablePixels = 0;
    // 2. smear left/right direction into real data
    maxgrad_pt = data.maxGradients[level] + width+1;
    maxgrad_pt_max = data.maxGradients[level] + width*(height-1)-1;
    maxgrad_t_pt = maxGradTemp + width+1;
    for(;maxgrad_pt<maxgrad_pt_max; maxgrad_pt++, maxgrad_t_pt++)
    {
        float g1 = maxgrad_t_pt[-1];
        float g2 = maxgrad_t_pt[0];
        if(g1 < g2) g1 = g2;
        float g3 = maxgrad_t_pt[1];
        if(g1 < g3)
        {
            *maxgrad_pt = g3;
            if(g3 >= MIN_ABS_GRAD_CREATE)
                numMappablePixels++;
        }
        else
        {
            *maxgrad_pt = g1;
            if(g1 >= MIN_ABS_GRAD_CREATE)
                numMappablePixels++;
        }
    }
    
    if(level==0)
        this->numMappablePixels = numMappablePixels;
    
    FrameMemory::getInstance().returnBuffer(maxGradTemp);
    
    data.maxGradientsValid[level] = true;*/
}


//weights of pixels for a kf are the average of weights estimated during pose est. of each frame on this kf
void frame::finaliseWeights()
{
    
    for(int level=util::MAX_PYRAMID_LEVEL-1;level>=0;level--)
    {
        //printf("\nFinalising weights for frame id: %d with weight count: %d at level: %d", frameId, numWeightsAdded[level], level);
        
        if(numWeightsAdded[level] > 0)
        {
            weight_pyramid[level]=weight_pyramid[level]/numWeightsAdded[level];
            //DisplayWeightsPixelWise(weight_pyramid[level], this, "Averaged Weights", frameId);
        }
        else
        {
            printf("\nWeights cannot be averaged!!! ");
        }

    }

    return;
}


//for saving mat object of type float to text file
void frame::saveMatAsText(Mat save_mat, string name, string save_mat_path)
{
    int id;
    
    if(util::FLAG_ALTERNATE_GN_RA)
        id=(frameId+util::BATCH_START_ID-1);
    else
        id=frameId;
    
    stringstream ss;
    string type = ".txt";

    ss << save_mat_path << "/" << id << "_" << name << type;
    string filename = ss.str();
    ss.str("");
    ofstream txt_file;
    txt_file.open(filename);
    
    cout<<"\nSaving mat image... " << filename;
    
    //printf("\nrows: %d, cols: %d",save_mat.rows,save_mat.cols);
    
    float* save_mat_ptr;
    for(int y=0;y<save_mat.rows;y++)
    {
        save_mat_ptr=save_mat.ptr<float>(y);
        
        for(int x=0; x < save_mat.cols ;x++)
        {
            txt_file << save_mat_ptr[x] << " ";
        }

        txt_file << "\n";
    }
    
    txt_file.close();
    
    return;
}


//requires an initialized make_mat object with predefined rows, cols, type(float)
void frame::makeMatFromText(Mat make_mat, string name, string read_txt_path)
{
    int id;
    if(util::FLAG_ALTERNATE_GN_RA)
        id=(frameId+util::BATCH_START_ID-1);
    else
        id=frameId;
    
    stringstream ss;
    string type = ".txt";
    ss<<read_txt_path << "/" << id << "_" << name << type;

    string filename = ss.str();
    ss.str("");
    ifstream txt_file;
    txt_file.open(filename);
    
    cout << "\nMaking mat image... " << filename;
        
    float* make_mat_ptr;
    for(int y=0;y<make_mat.rows;y++)
    {
        make_mat_ptr=make_mat.ptr<float>(y);
        for(int x=0; x < make_mat.cols; x++)
        {
            txt_file >> make_mat_ptr[x];
        }
    }
    
    txt_file.close();
    
    return;

    
}


//requires an initialized make_mat object with predefined rows, cols, type(float)
void frame::makeArrayFromText(float* make_arr, string name, string read_txt_path, int pyr_level)
{
    int id;
    if(util::FLAG_ALTERNATE_GN_RA)
        id=(frameId+util::BATCH_START_ID-1);
    else
        id=frameId;
    
    stringstream ss;
    string type = ".txt";

    ss<<read_txt_path << "/" << id << "_" << name << type;
    string filename = ss.str();
    ss.str("");
    ifstream txt_file;
    txt_file.open(filename);
    
    cout << "\nMaking Array... " << filename;
    
    int arr_size=0;
    switch (pyr_level)
    {
        case 0:
            arr_size=util::ORIG_COLS*util::ORIG_ROWS;
            break;
        case 1:
            arr_size=(util::ORIG_ROWS >> 1)*(util::ORIG_COLS >> 1);
            break;
        case 2:
            arr_size=(util::ORIG_ROWS >> 2)*(util::ORIG_COLS >> 2);
            break;
        case 3:
            arr_size=(util::ORIG_ROWS >> 3)*(util::ORIG_COLS >> 3);
            break;
    }
    
    //printf("\nrows: %d, cols: %d",save_mat.rows,save_mat.cols);
    
    for(int i=0; i < arr_size; i++)
    {
        txt_file >> make_arr[i];
        
    }
    
    txt_file.close();
    
    return;
    
    
}

void frame::saveArrayAsText(float* save_arr, string name, string save_arr_path, int pyr_level)
{
    int id;

    if(util::FLAG_ALTERNATE_GN_RA)
        id=(frameId+util::BATCH_START_ID-1);
    else
        id=frameId;
    
    stringstream ss;
    string type = ".txt";

    ss << save_arr_path << "/" << id << "_" << name << type;
    string filename = ss.str();
    ss.str("");
    ofstream txt_file;
    txt_file.open(filename);
    
   cout<<"\nSaving array... "<<filename;
    
    //printf("\nrows: %d, cols: %d",save_mat.rows,save_mat.cols);
    
    int arr_size=0;
    switch (pyr_level)
    {
        case 0:
            arr_size=util::ORIG_COLS*util::ORIG_ROWS;
            break;
        case 1:
            arr_size=(util::ORIG_ROWS >> 1)*(util::ORIG_COLS >> 1);
            break;
        case 2:
            arr_size=(util::ORIG_ROWS >> 2)*(util::ORIG_COLS >> 2);
            break;
        case 3:
            arr_size=(util::ORIG_ROWS >> 3)*(util::ORIG_COLS >> 3);
            break;
    }

    
    for(int i=0;i<arr_size;i++)
    {
        txt_file << save_arr[i] << " ";
    }
    
    txt_file.close();
    
    return;

    
}


void frame::makeDepthHypothesisFromText(depthhypothesis* make_depth_hypo, string name, string read_txt_path)
{
 
    stringstream ss;
    string type = ".txt";
    ss << read_txt_path << "/" << frameId << "_" << name << type;
    string filename = ss.str();
    ss.str("");
    ifstream txt_file;
    txt_file.open(filename);
        
    float depth_val;
    
    for(int y=0;y<util::ORIG_ROWS;y++)
    {
        for(int x=0;x<util::ORIG_COLS;x++)
        {
            depthhypothesis* ptr = make_depth_hypo + util::ORIG_COLS * y + x;
            txt_file >> depth_val;
            ptr->invDepth=1/depth_val;
            if(depth_val<=0)
                ptr->isValid=0;
        }
        
    }
    
    txt_file.close();
    
    return;

}

