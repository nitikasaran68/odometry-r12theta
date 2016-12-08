
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <cstdio>
#include <ctime>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "ExternVariable.h"
#include <cmath>

#include "UserDefinedFunc.h"
#include "IterationFunc.h"
#include "DisplayFunc.h"


using namespace std;
using namespace cv;

/*
 FUNCTION: CALCULATE WORLD POINTS
 
 PURPOSE: CALCULATE WORLD POINTS AND SAVES INTENSITY VALUE OF IMAGE AT NON ZERO DEPTH POINTS
 INPUT: SAVE IMAGE MATRIX, WORLD POINT MATRIX, MASK, ORIGINAL IMAGE, DEPTH, NUMBER OF ROWS IN IMAGE MATRIX, NUMBER OD COLUMNS , NUMBER OF NON ZERO DEPTH POINTS, CURRENT PYRAMID LEVEL
 OUTPUT: NONE
*/
void CalculateWorldPoints(Mat& saveimg, Mat& worldpoint, frame* image_frame)
{
    PRINTF("\nCalculating World Points for previous image: %d", image_frame->frameId);
    //*******INITIALIZE POINTERS*******//
    
    int pyrlevel=image_frame->pyrLevel;
    int nRows=image_frame->currentRows;
    int nCols=image_frame->currentCols;
    
    //worldpoint pointers
    float* worldpoint_ptr0; //pointer to access row 0
    float* worldpoint_ptr1; //pointer to access row 1
    float* worldpoint_ptr2; //pointer to access row 2
    uchar* img_ptr;
    float* saveimg_ptr;
    uchar* mask_ptr;
    float* depth_ptr;
    
    worldpoint_ptr0=worldpoint.ptr<float>(0); //initialize to row0
    worldpoint_ptr1=worldpoint.ptr<float>(1); //initialize to row1
    worldpoint_ptr2=worldpoint.ptr<float>(2); //initialize to row2
    
    saveimg_ptr=saveimg.ptr<float>(0); //initialize to row 0 (single row Mat)
    
    //*******CALCULATE RESIZED INTRINSIC PARAMETERS*******//
    
    vector<float> resized_intrinsic= GetIntrinsic(image_frame->pyrLevel);
    float resized_fx=resized_intrinsic[0];
    float resized_fy=resized_intrinsic[1];
    float resized_cx=resized_intrinsic[2];
    float resized_cy=resized_intrinsic[3];
    
    //*******INITIALIZE COUNTERS*******//
    
    int world_rows0=0; //counter to access columns in row 0
    int world_rows1=0; //counter to access columns in row 1
    int world_rows2=0; //counter to access columns in row 2
    int saveimg_row=0; //counter to store in ro0 of saveimg
    
    int i,j;//loop variables
    for(i = 0; i < nRows; ++i)
    {
        
        //get pointer for current row i;
        img_ptr = image_frame->image_pyramid[pyrlevel].ptr<uchar>(i);
        mask_ptr=image_frame->mask.ptr<uchar>(i);
        depth_ptr = image_frame->depth_pyramid[pyrlevel].ptr<float>(i);
        
        for (j = 0; j < nCols; ++j)
            
        {   //check for zero depth point
            if(mask_ptr[j]==0)
            continue;  //skip loop for non zero depth points
            
            //save image intensity at non zero depth point
            saveimg_ptr[saveimg_row++]=img_ptr[j];
            
            //populating world point with homogeneous coordinate ( X ;Y ;depth ; 1)
            worldpoint_ptr0[world_rows0++]=(j-resized_cx)*depth_ptr[j]/resized_fx; //X-coordinate
            worldpoint_ptr1[world_rows1++]=(i-resized_cy)*depth_ptr[j]/resized_fy; //Y-coordinate
            worldpoint_ptr2[world_rows2++]=depth_ptr[j]; //depth of point at (u,v)
            //worldpoint_ptr3[world_rows3++]=1; //1 for making homogeneous
            
        }
    }

}




/*
 FUNCTION: CALCULATE WARPED POINTS

 PURPOSE: CALCULATES THE POSE MATRIX AND WARPED POINTS FOR THE POSE
 INPUT: POSE VECTOR, WORLDPOINT MATRIX, NUMBER OF NON ZERO DEPTH POINTS, PYRAMID LEVEL
 OUTPUT: WARPED POINT MATRIX
*/

Mat CalculateWarpedPoints (float *pose, Mat worldpoint, frame* image_frame)
{
    PRINTF("\nCalculating warped points for previous frame: %d ", image_frame->frameId);
    //*******CALCULATE POSE MATRIX *******//
    
    int nonZeroPts=image_frame->no_nonZeroDepthPts;
    //Create matrix in OpenCV
    Mat se3=(Mat_<float>(4, 4) << 0,-pose[2],pose[1],pose[3], pose[2],0,-pose[0],pose[4], -pose[1],pose[0],0,pose[5],0,0,0,0);
    
    // Map the OpenCV matrix with Eigen:
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> se3_Eigen(se3.ptr<float>(), se3.rows, se3.cols);
    
    // Take exp in Eigen and store in new Eigen matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> SE3_Eigen = se3_Eigen.exp();
   
    // create an OpenCV Mat header for the Eigen data:
    Mat SE3(4, 4, CV_32FC1, SE3_Eigen.data());
    
    //cout<<"\n\nSE3"<<SE3;
    
    //stores warped points
    Mat warpedpoint(2,nonZeroPts, CV_32FC1);
    
    //cout<<"\n\nSE3:  "<<SE3;

    //*******INITIALIZE POINTERS*******//
    
    //warpedpoint pointers
    float* warpedpoint_ptrx; //pointer to access row 0=> x image coordinate
    float* warpedpoint_ptry; //pointer to access row 1=> y image coordinate
    //worldpoint pointers
    float* worldpoint_ptrX; //pointer to access row 0=> X
    float* worldpoint_ptrY; //pointer to access row 1=> Y
    float* worldpoint_ptrZ; //pointer to access row 2=> Z
    
    warpedpoint_ptrx=warpedpoint.ptr<float>(0); //initialize to row0
    warpedpoint_ptry=warpedpoint.ptr<float>(1); //initialize to row1
    
    worldpoint_ptrX=worldpoint.ptr<float>(0); //initialize to row0
    worldpoint_ptrY=worldpoint.ptr<float>(1); //initialize to row1
    worldpoint_ptrZ=worldpoint.ptr<float>(2); //initialize to row2
    
    //*******CALCULATE RESIZED INTRINSIC PARAMETERS*******//
    
    vector<float> resized_intrinsic= GetIntrinsic(image_frame->pyrLevel);
    float resized_fx=resized_intrinsic[0];
    float resized_fy=resized_intrinsic[1];
    float resized_cx=resized_intrinsic[2];
    float resized_cy=resized_intrinsic[3];
    
    //*******STORE SE3 PARAMETERS*******//
  
    float* SE3_ptr;
    float SE3_vec[12];  //r11, r12, r13, t1, r21 r22, r23, t2, r31, r32, r33, t3
    int vec_counter=0;
    int i; //loop variables
    
    for(i=0; i<3; ++i)
    {
        SE3_ptr=SE3.ptr<float>(i); //get pointer to first row of SE3
        
        SE3_vec[vec_counter++]=SE3_ptr[0];
        SE3_vec[vec_counter++]=SE3_ptr[1];
        SE3_vec[vec_counter++]=SE3_ptr[2];
        SE3_vec[vec_counter++]=SE3_ptr[3];

    }
    
   // cout<<"\n\nSE3 VECTOR  "<<SE3_vec[0]<<" : "<< SE3_vec[1]<<" : "<<SE3_vec[2]<<" : "<<SE3_vec[3]<<" : "<<SE3_vec[4]<<" : "<<SE3_vec[5]<<" : "<<SE3_vec[6]<<" : "<<SE3_vec[7]<<" : "<<SE3_vec[8]<<" : "<<SE3_vec[9]<<" : "<<SE3_vec[10]<<" : "<<SE3_vec[11];
   
  //enter loop to calculate warped points
    int j;
    
    for( j = 0; j < nonZeroPts; ++j)
    {
        
        
        if (SE3_vec[1]==0)
        {
        
        warpedpoint_ptrx[j]=((((SE3_vec[0]*worldpoint_ptrX[j])+(SE3_vec[1]*worldpoint_ptrY[j])+(SE3_vec[2]*worldpoint_ptrZ[j])+(SE3_vec[3]))/((SE3_vec[8]*worldpoint_ptrX[j])+(SE3_vec[9]*worldpoint_ptrY[j])+(SE3_vec[10]*worldpoint_ptrZ[j])+(SE3_vec[11])))*resized_fx)+resized_cx;
        
        warpedpoint_ptry[j]=((((SE3_vec[4]*worldpoint_ptrX[j])+(SE3_vec[5]*worldpoint_ptrY[j])+(SE3_vec[6]*worldpoint_ptrZ[j])+(SE3_vec[7]))/((SE3_vec[8]*worldpoint_ptrX[j])+(SE3_vec[9]*worldpoint_ptrY[j])+(SE3_vec[10]*worldpoint_ptrZ[j])+(SE3_vec[11])))*resized_fy)+resized_cy;
            
        }
        
        else
        {
        
        warpedpoint_ptrx[j]=float((((float(SE3_vec[0]*worldpoint_ptrX[j])+float(SE3_vec[1]*worldpoint_ptrY[j])+float(SE3_vec[2]*worldpoint_ptrZ[j])+float(SE3_vec[3]))/(float(SE3_vec[8]*worldpoint_ptrX[j])+float(SE3_vec[9]*worldpoint_ptrY[j])+float(SE3_vec[10]*worldpoint_ptrZ[j])+float(SE3_vec[11])))*float(resized_fx))+float(resized_cx));
        
        
        warpedpoint_ptry[j]=float((((float((SE3_vec[4]*worldpoint_ptrX[j]))+float((SE3_vec[5]*worldpoint_ptrY[j]))+float((SE3_vec[6]*worldpoint_ptrZ[j]))+float((SE3_vec[7])))/(float((SE3_vec[8]*worldpoint_ptrX[j]))+float((SE3_vec[9]*worldpoint_ptrY[j]))+float((SE3_vec[10]*worldpoint_ptrZ[j]))+float((SE3_vec[11]))))*float(resized_fy))+float(resized_cy));
            
        }
    
    }
    

    return warpedpoint;
    
}




/*
 FUNCTION: CALCULATE WARPED IMAGE
 
 PUROSE: CALCULATES THE INTERPOLATED WAARPED IMAGE POINTS
 INPUT: WARPEDPOINT MATRIX, ORIGINAL IMAGE MATRIX, NUMBER OF ROWS, NUMBER OF COLUMNS, NUMBER OF NON ZERO DEPTH POINTS
 OUTPUT: WARPED IMAGE MATRIX
 */

Mat CalculateWArpedImage(Mat warpedpoint,frame* current_frame,frame* prev_frame)
{
    PRINTF("\nCalculating Warped Image for previous frame: %d, current frame: %d", prev_frame->frameId, current_frame->frameId);
    int nonZeroPts=prev_frame->no_nonZeroDepthPts;
    int pyrlevel=prev_frame->pyrLevel;

    //declaring warped img
    //Mat warpedimg(1,no_nonzero_mask, CV_32FC1);
    
    Mat warpedimg=Mat::zeros(1,nonZeroPts,CV_32FC1);

    //*******INITIALIZE POINTERS*******//
    
    //initialize pointers
    uchar* img_ptr0; //pointer0 to access original image
    uchar* img_ptr1; //pointer1 to acess original image
    float* warpedimg_ptr; //pointer to store in warped image
    float* warpedpoint_ptrx; //pointer to acess warped point x => column
    float* warpedpoint_ptry; //pointer to access warped point y=> row
    
    warpedpoint_ptrx=warpedpoint.ptr<float>(0); //initialize to row0 to acess x coordinate
    warpedpoint_ptry=warpedpoint.ptr<float>(1); //initialize to row1 to acess y coordinate
    warpedimg_ptr=warpedimg.ptr<float>(0); //initialize to store in row0 of warped image
    
    //*******DECLARE VARIABLES*******//
    
    //cout<<"\n\nWARPED POINTS   "<<warpedpoint;
    
    
    int j; //for loop variables
    float yx[2]; //to store warped points for current itertion
    float wt[2]; //to store weight
    float y,x; //to store x and y coordinate to acess original image
    uchar pixVal1, pixVal2; //to store intensity value
    float interTop, interBtm;
    
    int nCols=prev_frame->currentCols-1; //maximum valid x coordinate
    int nRows=prev_frame->currentRows-1; //maximum valid y coordinate
 
    for(j=0; j<nonZeroPts; j++ )
    {
        yx[0]=warpedpoint_ptry[j]; //store current warped point y
        yx[1]=warpedpoint_ptrx[j]; //store warped point x
        
        wt[0]=yx[0]-floor(yx[0]); //weight for y
        wt[1]=yx[1]-floor(yx[1]); //weight for x
        
        //Case 1
        y=floor(yx[0]); //floor of y
        x=floor(yx[1]); //floor of x
  
        //cout<<"\n\n"<<int(x);
        if ((x<0)||(x>nCols)||(y<0)||(y>nRows))
            pixVal1=0; //if outside boundary pixel value is 0
        else
        {
            img_ptr0=current_frame->image_pyramid[pyrlevel].ptr<uchar>(y); //initialize image pointer0 to row floor(y)
            pixVal1=img_ptr0[int(x)]; //move pointer to get value at pixel (floor(x), floor(y))
        }
        
        
        
        x=yx[1]; //warped point x

        if ((x<0)||(x>nCols)||(y<0)||(y>nRows))
            pixVal2=0; //if outside boundary pixel value is 0
        else
        {
            img_ptr0=current_frame->image_pyramid[pyrlevel].ptr<uchar>(y); //initialize image pointer0 to row floor(y)
            pixVal2=img_ptr0[int(ceil(x))]; //move pointer to get value at pixel (ceil(x), floor(y))
        }

        interTop=((1-wt[1])*pixVal1)+(wt[1]*pixVal2);


        //Case 2
        y=yx[0]; //warped point y
       
        x=floor(yx[1]); //floor of x
        
        if ((x<0)||(x>nCols)||(y<0)||(y>nRows))
            pixVal1=0; //if outside boundary pixel value is 0
        else
        {
            img_ptr1=current_frame->image_pyramid[pyrlevel].ptr<uchar>(ceil(y)); //initialize image pointer1 to row ceil(y)
            pixVal1=img_ptr1[int(x)]; //move pointer to get value at pixel (floor(x), ceil(y))
        }

        x=yx[1]; //warped point x

        if ((x<0)||(x>nCols)||(y<0)||(y>nRows))
            pixVal2=0; //if outside boundary pixel value is 0
        else
        {
            img_ptr1=current_frame->image_pyramid[pyrlevel].ptr<uchar>(ceil(y)); //initialize image pointer1 to row ceil(y)
            pixVal2=img_ptr1[int(ceil(x))]; //move pointer to get value at pixel (ceil(x), ceil(y))
        }

        interBtm=((1-wt[1])*pixVal1)+(wt[1]*pixVal2);

        warpedimg_ptr[j]=((1-wt[0])*interTop)+(wt[0]*interBtm); //calculate interpolated value to get warped image intensity

        }

    return warpedimg;

}


/* 
 FUNCTION: UPDATE POSE
 
 PURPOSE: CALCULATE RESIDUAL,  UPDATE CURRENT ESTIMATE OF POSE VECTOR AND CHECK CONDITION FOR TERMINATION OF ITERATION LOOP
 INPUT: SAVED ORIGINAL IMAGE OF NON ZERO DEPTH POINTS, WARPED IMAGE , STEEPEST DESCENT MATRIX, POSE POINTER, NUMBER OF NON ZERO DEPTH POINTS, 
        WEIGHT POINTER OF POSE PARAMETERS, THRESHOLD VALUE TO TERMINATE LOOP
 OUTPUT: RETURNS -1 IF LOOP NEEDS TO BE TERMINATED OTHERWISE 0
 */

int UpdatePose(Mat& residual, Mat saveimg, Mat warpedimg, Mat steepdesc, Mat hessianinv, float* pose, int no_nonzero_mask)
{
    PRINTF("\nUpdating pose ");
    residual=warpedimg-saveimg; //calculate residual

    //cout<<"\n\n\n"<<residual;
    
    Mat sd_param= residual*steepdesc; //calculate steepest descnet parameters
    
    //cout<<steepdesc;
    Mat deltapose= ((hessianinv*(sd_param.t())).t()); //calculate delta pose (change in pose)
    deltapose=-deltapose;
    
    float* deltapose_ptr=deltapose.ptr<float>(0); //initialize pointer to delta pose
    
    //PRINTF("\nDelta Pose: %f , %f , %f , %f , %f , %f", deltapose_ptr[0],deltapose_ptr[1],deltapose_ptr[2],deltapose_ptr[3],deltapose_ptr[4],deltapose_ptr[5]);
    
    //cout<<"\nWeighted Pose in func "<<abs(deltapose_ptr[0]*weight[0])<<" , "<<abs(deltapose_ptr[1]*weight[1])<<" , "<<abs(deltapose_ptr[2]*weight[2])<<" , "<<abs(deltapose_ptr[3]*weight[3])<<" , "<<abs(deltapose_ptr[4]*weight[4])<<" , "<<abs(deltapose_ptr[5]*weight[5])<<" , ";
    
    
    //calculate weighted pose value
    float weighted_pose = abs(deltapose_ptr[0]*util::weight[0])+abs(deltapose_ptr[1]*util::weight[1])+abs(deltapose_ptr[2]*util::weight[2])+abs(deltapose_ptr[3]*util::weight[3])+abs(deltapose_ptr[4]*util::weight[4])+abs(deltapose_ptr[5]*util::weight[5]);
    
    PRINTF("Total Weighted Pose: %f ",weighted_pose);
    
    //check condition for termination
    if( weighted_pose < util::threshold1)
    {
        PRINTF("\nWeighted pose below threshold! Terminating");
     return -1;  //terminate iteration loop
    }

    
    //update current estimate of pose
    pose[0]+=real(deltapose_ptr[0]);
    pose[1]+=real(deltapose_ptr[1]);
    pose[2]+=real(deltapose_ptr[2]);
    pose[3]+=real(deltapose_ptr[3]);
    pose[4]+=real(deltapose_ptr[4]);
    pose[5]+=real(deltapose_ptr[5]);
   
   // cout<<"\n\nPose in func "<<pose[0]<<" , "<<pose[1]<<" , "<<pose[2]<<" , "<<pose[3]<<" , "<<pose[4]<<" , "<<pose[5]<<" , ";
    //cout<<"\nDelta pose: "<<deltapose;
    return 0;
    
}




