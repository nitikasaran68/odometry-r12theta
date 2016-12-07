
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <cstdio>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "ExternVariable.h"

#include "UserDefinedFunc.h"
#include "DisplayFunc.h"


using namespace std;
using namespace cv;

/*
 FUNCTION: FORM IMAGE MASK
 
 PURPOSE: FORMS MASK WITH NON-ZERO DEPTH POINTS=1, ZERO DEPTH POINTS=255
 INPUT: DEPTH IMAGE
 OUTPUT: MASK MATRIX
 */
Mat FormImageMask(Mat depth_img)
{
     PRINTF("\n !!!  Calculating Image Mask  !!!");
    return (depth_img>0.0f);
    
}



/*
 FUNCTION: FIND GRADIENT
 
 PURPOSE: FIND IMAGE GRADIENT IN A PARTICULAR DIRECTION
 INPUT: SOURCE MATRIX, DESTINATION MATRIX, DIRECTION (x OR y)
 OUTPUT: DESTINATION MATRIX
 */
void CalculateGradient(Mat Source,Mat& dstx, Mat& desty ) //////dont use!!!
{

    PRINTF("\nCalculating Gradient function1");
    //float kernelarr = { -1, 0, 1 };
    Mat kernelx = (Mat_<float>(1, 3)<< -1,0,1);
    // Gradient X
    //Sobel(src, dst, ddepth=16S, dx=1, dy=0, ksize=1 (for no smoothing), scale=1,delta=0, borderType=BORDER_DEFAULT )∂
    filter2D(Source, dstx, CV_32FC1, kernelx);
    
    //float kernelarr = { -1, 0, 1 };
    Mat kernely = (Mat_<float>(3,1)<< -1,0,1);
    //Gradient Y
    //Sobel(src, dst, ddepth=16S, dx=0, dy=1, ksize=1 (for no smoothing), scale=1,delta=0, borderType=BORDER_DEFAULT )∂
    filter2D(Source, desty, CV_32FC1, kernely);
}


/*
 FUNCTION: CALCULATE STEEPEST DESCENT
 
 PURPOSE: RETURNS STEEPEST DESCENT MATRIX FOR A FRAME AT A PARTICULAR PYRAMID LEVEL AND POPULATES WORLD POINT MATRIX WITH IMAGE COORDINATES         AND DEPTH
 INPUTS: DEPTH MATRIX, MASK OF DEPTH MATRIX, GRADIENT IN X DIRECTION OF FRAME, GRADIENT IN Y DIRECTION OF FRAME, WORLD POINT MATRIX
         NUMBER OF ROWS, NUMBER OF COLUMNS, PYRAMID LEVEL
 OUTPUT: STEEPEST DESCENT MATRIX 
 */

Mat CalculateSteepestDescent(frame* image_frame)
{
    PRINTF("\nCalculating Steepest Descent for frame: %d ", image_frame->frameId);
    int nRows=image_frame->currentRows;
    int nCols=image_frame->currentCols;
    
    //*******INITIALIZE VAR*******//
  
    Mat jacobian_top(image_frame->no_nonZeroDepthPts,6, CV_32FC1);
    Mat jacobian_bottom(image_frame->no_nonZeroDepthPts,6, CV_32FC1);
    
    //cout<<"\n\n\nGRADIENT POINTER:  "<<(gradx.type()==CV_32FC1);
    
    
    //calculate resized intrinsic focal parameters for pyrlevel
    vector<float> resized_intrinsic= GetIntrinsic(image_frame->pyrLevel);
    float resized_fx=resized_intrinsic[0];
    float resized_fy=resized_intrinsic[1];
    float resized_cx=resized_intrinsic[2];
    float resized_cy=resized_intrinsic[3];
    
    
    //*******INITIALIZE POINTERS*******//
    
    //pointers to access elements
    float* depth_ptr;
    float* gradx_ptr;
    float* grady_ptr;
    float* jacob_top_ptr;
    float* jacob_bottom_ptr;
    uchar* mask_ptr;
    
    int jac_rows;
    //check if matrix stored continuously
    if(jacobian_top.isContinuous() & jacobian_bottom.isContinuous())
        jac_rows= -1;
    else
         jac_rows= 0;
    
    int jac_counter =0;
    
    jacob_top_ptr = jacobian_top.ptr<float>(0);
    jacob_bottom_ptr = jacobian_bottom.ptr<float>(0);
    
   //*******LOOP TO CALCULATE JACOBIAN*******//
    
    int i,j; //loop variables
    for( i = 0; i < nRows; ++i)
    {
        
        depth_ptr = image_frame->depth_pyramid[image_frame->pyrLevel].ptr<float>(i);
        
        gradx_ptr = image_frame->gradientx.ptr<float>(i);
        grady_ptr = image_frame->gradienty.ptr<float>(i);
        
        mask_ptr=image_frame->mask.ptr<uchar>(i);
        
        for ( j = 0; j < nCols; ++j)
        {
            
            if(mask_ptr[j]==0)
                continue;  //skip loop for non zero depth points
            
            //Calculate value of Jacobian for a point
            
            jacob_bottom_ptr[jac_counter]  = grady_ptr[j]*(-(resized_fy + (pow((-resized_cy + i),2) / resized_fy)));
            jacob_top_ptr[jac_counter++] = gradx_ptr[j]*(-((-resized_cy + i)*(-resized_cx + j)) / resized_fy);
            
            jacob_bottom_ptr[jac_counter]=grady_ptr[j]*(((-resized_cy + i)*(-resized_cx + j)) / resized_fx);
            jacob_top_ptr[jac_counter++] = gradx_ptr[j]*(resized_fx + (pow((-resized_cx + j),2) / resized_fx));
            
            jacob_bottom_ptr[jac_counter]=grady_ptr[j]*((resized_fy*(-resized_cx + j)) / resized_fx);
            jacob_top_ptr[jac_counter++]  = gradx_ptr[j]*(-(resized_fx*(-resized_cy + i) / resized_fy));
            
            jacob_bottom_ptr[jac_counter]=0;
            jacob_top_ptr[jac_counter++]  =gradx_ptr[j]*( resized_fx*(pow(depth_ptr[j],-1)));
            
            jacob_bottom_ptr[jac_counter]= grady_ptr[j]*(resized_fy*(pow(depth_ptr[j],-1)));
            jacob_top_ptr[jac_counter++]  = 0;
            
            jacob_bottom_ptr[jac_counter]= grady_ptr[j]*(-(-resized_cy + i) *(pow(depth_ptr[j],-1)));
            jacob_top_ptr[jac_counter++]  = gradx_ptr[j]*(-(-resized_cx + j) *(pow(depth_ptr[j],-1)));
            
            
            if(jac_rows>-1)
            {   jac_rows=jac_rows+1;
                jacob_bottom_ptr=jacobian_bottom.ptr<float>(jac_rows);
                jacob_top_ptr=jacobian_top.ptr<float>(jac_rows);
                jac_counter=0;
            }
            
        }
    }
    //*******CALCULATE STEEPEST DESCENT*******//
    
    Mat steepest_desc(image_frame->no_nonZeroDepthPts,6, CV_32FC1);
    steepest_desc=jacobian_top+jacobian_bottom;
    
    return steepest_desc;
   
}



/*
 FUNCTION: CALCULATE HESSIAN INVERSE
 
 PURPOSE: RETURNS HESSIAN INVERSE MATRIX FOR A FRAME
 INPUTS: STEEPEST DESCENT  MATRIX
 OUTPUTS: HESSIAN INVERSE MATRIX
*/

Mat CalculateHessianInverse(Mat steepest_desc)
{
    PRINTF("\nCalculating Hessian "); 
    Mat hessian =steepest_desc.t()*steepest_desc;
    return hessian.inv();
}



/*
 FUNCTION: GET INTRINSIC
 
 PURPOSE: RETURN THE RESIZED INTRINSIC MATRIX [fx, fy, cx, cy]
 INPUT: PYRAMID LEVEL
 OUTPUT: POINTER TO FIRST ELEMENT OF INTRINSIC MATRIX
*/

vector<float> GetIntrinsic(int pyrlevel)
{
    PRINTF("\nCalculating Intrinsic parameters for level: %d", pyrlevel);
    vector<float> resized_intrinsic(4);

    //calculating resized intrinsic parameters  fx, fy, cx, cy
    resized_intrinsic[0] = (util::ORIG_FX / pow(2, pyrlevel));
    resized_intrinsic[1] = (util::ORIG_FY / pow(2, pyrlevel));
    resized_intrinsic[2] = (util::ORIG_CX / pow(2, pyrlevel));
    resized_intrinsic[3] = (util::ORIG_CY / pow(2, pyrlevel));
    
    //storing in resized_intrinsic=[fx, fy, cx, cy];
   // Mat resized_intrinsic = (Mat_<float>(1,4)<<resized_fx, resized_fy, resized_cx, resized_cy);
    
    //return pointer to the first element
    
    return resized_intrinsic;
}


Mat CalculateTransformationMatrix(float *pose){


    //Create matrix in OpenCV
    Mat se3=(Mat_<float>(4, 4) << 0,-pose[2],pose[1],pose[3], pose[2],0,-pose[0],pose[4], -pose[1],pose[0],0,pose[5],0,0,0,0);
    
    // Map the OpenCV matrix with Eigen:
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> se3_Eigen(se3.ptr<float>(), se3.rows, se3.cols);
    
    // Take exp in Eigen and store in new Eigen matrix
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> SE3_Eigen = se3_Eigen.exp();
    
    // create an OpenCV Mat header for the Eigen data:
    Mat SE3(4, 4, CV_32FC1, SE3_Eigen.data());
    return SE3;
}






