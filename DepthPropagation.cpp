//
//  DepthPropagation.cpp
//  odometry
//
//  Created by Himanshu Aggarwal on 8/29/15.
//  Copyright (c) 2015 Himanshu Aggarwal. All rights reserved.
//
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include "DepthPropagation.h"
#include "DisplayFunc.h"
#include "ExternVariable.h"
#include "EigenInitialization.h"


using namespace std;
using namespace cv;


//constructor
depthMap::depthMap()
{
    
    PRINTF("\nConstructing the empty Depth Map ");
    
    for(int i=0;i<util::ORIG_COLS*util::ORIG_ROWS;i++)
    validityIntegralBuffer[i]=0;
    
    //initializes pointers
    depthvararrptr[0]=depthvararrpyr0;
    depthvararrptr[1]=depthvararrpyr1;
    depthvararrptr[2]=depthvararrpyr2;
    depthvararrptr[3]=depthvararrpyr3;
    //depthvararrptr[4]=depthvararrpyr4;
    
    deptharrptr[0]=deptharrpyr0;
    deptharrptr[1]=deptharrpyr1;
    deptharrptr[2]=deptharrpyr2;
    deptharrptr[3]=deptharrpyr3;
    //deptharrptr[4]=deptharrpyr4;
    
}

//updates depth map parameters whenever there is a new current frame
void depthMap::formDepthMap(frame* image_frame)
{
    PRINTF("\nForming the Depth Map for frame Id: %d", image_frame->frameId);
    
    currentFrame=image_frame;
    currentFrame->isKeyframe=false;
    
    size=util::ORIG_ROWS*util::ORIG_COLS;
    
    if(image_frame->frameId == 1) //so that only calls for the first frame
    {
        keyFrame=image_frame; //first frame is also the kf
        keyFrame->isKeyframe=true;
        keyFrame->rescaleFactor=1.0f; //initially scale of kf is 1
        initializeRandomly(); //calls random initialization of depth map
        //image_frame->constructDepthPyramids();
       
    }
    //currentFrame->calculateSE3poseOtherWrtThis(keyFrame); shifted to update key frame
    //keyFrame->calculateSE3poseOtherWrtThis(currentFrame); dont calculate!!
    
    currentFrame->parentKeyframeId=keyFrame->frameId;
    currentFrame->rescaleFactor=keyFrame->rescaleFactor;

    
}


//for resetting dpeth hypothesis
void depthMap::resetHypothesis(int idx)
{
    PRINTF("Resetting hypothesis for current frame Id: %d, pixel Id: %d",currentFrame->frameId, idx);
    
    currentDepthHypothesis[idx].nUsed=0;
    currentDepthHypothesis[idx].oldestFrameId=currentFrame->frameId;
    currentDepthHypothesis[idx].successful=0;

}

//not used
void depthMap::refineDepth( vector<frame*> frameptrvector)
{
    PRINTF("\nRefining Depth Map");
    //selectReferanceFrame(frameptrvector,int pixId);
    
}


//initialization function for the first frame
//supports random initialization as well as initializing depth values from a saved depth matrix
void depthMap::initializeRandomly()
{
    
   Mat save_depth=Mat(util::ORIG_ROWS,util::ORIG_COLS,CV_32FC1); //for saving depth if initialized from file
   float save_array[util::ORIG_COLS*util::ORIG_ROWS]; //for saving variance if initialized from file
    
    //if replicate depth flag is on, does not do random initialization. Instead initializes from file
    if(util::FLAG_REPLICATE_NEW_DEPTH)
    {
        //make depth mats from text file
        currentFrame->makeMatFromText(save_depth, "Depth", util::SAVED_MATS_PATH);
        //make depth var arrays from text file
        currentFrame->makeArrayFromText(save_array, "DepthVarArr_pyr0", util::SAVED_MATS_PATH, 0);
    }

    float* saved_depth_ptr;
    float* imgMaxGrad=currentFrame->maxAbsGradient.ptr<float>(0);

    int random_init=0; //count for points initialized
    
    for(int y=1;y<util::ORIG_ROWS-1;y++)
    {
        imgMaxGrad=currentFrame->maxAbsGradient.ptr<float>(y); //pointer to row of maxGradient
        saved_depth_ptr=save_depth.ptr<float>(y);

        for(int x=1;x<util::ORIG_COLS-1;x++)
        {
            //depthhypothesis* target = currentDepthHypothesis+x+util::ORIG_COLS*y;
            depthhypothesis* target= &currentDepthHypothesis[x+util::ORIG_COLS*y]; //target pixel
            
            //<change>
//            if(y>=util::YMAX)
//            {
//                target->isValid=false;
//                target->blacklisted=0;
//                continue;
//            }
            
            if(util::FLAG_REPLICATE_NEW_DEPTH) //if initialize from file
            {
                if(saved_depth_ptr[x]==0) //if depth is zero, point is invalid
                {
                    target->isValid=false;
                    target->blacklisted=0;
                    continue;
                }
                
                //reaches here for non-zero depth
                target->invDepth=1/saved_depth_ptr[x]; //depth is inversed
                target->variance=save_array[x+util::ORIG_COLS*y];
                
                //smoothed var and depth is same
                target->varianceSmoothed=target->variance;
                target->invDepthSmoothed=target->invDepth;
                
                target->validity_counter=20; //from LSD
                target->isValid=true;
                target->blacklisted=0; //from LSD
                
                random_init++;
                continue;

            }

            //reaches here for random initialization
            
            if(imgMaxGrad[x] > 1.0*util::MIN_ABS_GRAD_CREATE) //checks gradient
            {
                //this if-else not needed as will never be evaluated as true
                if(util::FLAG_REPLICATE_NEW_DEPTH)
                {
                    if(saved_depth_ptr[x]==0)
                    {
                        target->isValid=false;
                        target->blacklisted=0;
                        continue;
                    }
                    target->invDepth=1/saved_depth_ptr[x];
                    target->variance=save_array[x+util::ORIG_COLS*y];
                    target->varianceSmoothed=target->variance;
                }
                else //always come here when gradient is above the threshold
                {
                    //random initialization
                    target->invDepth=0.5f + 1.0f * ((rand() % 100001) / 100000.0f);
                    target->variance=util::VAR_RANDOM_INIT_INITIAL; //constants
                    target->varianceSmoothed=util::VAR_RANDOM_INIT_INITIAL;

                }
                
                target->invDepthSmoothed=target->invDepth;
                target->validity_counter=20;
                target->isValid=true;
                target->blacklisted=0;
                
                random_init++;
    
            }
            else //when gradient is below threshold, point will not be initialized
            {
                target->isValid=false;
                target->blacklisted=0;
            }
           

        }
    }
    
   // cout<<"\nRANDOM INIT :"<<random_init;
    
}


//this function observes the depth and decides whether to refine the estimate or create a new one
//each pixel is treated independently, therefore it supports multiple threads
//takes in a ymin and ymax which forms the boundary of the section of image on which it acts
//the thread_num specifies the current thread that is calling this function. For single thread, thread_num is 0
void depthMap::observeDepthRow(int ymin,int ymax, int thread_num)
{
    PRINTF("\nObserving Depth Row for current frame Idx: %d", currentFrame->frameId);
    //printf("\nObserve depth row YMIN: %d  YMAX: %d",ymin,ymax);
    // Variables for debug purpose
    clock_t start, end;
    
    start = clock();
    
    int successes_create=0; //pixels whose depth estimate is initialized successfully
    int successes_update=0; //pixels whose depth estimate is updated successfully
    int return_val;
    //
    //    int ret_10=0;
    //    int ret_20=0;
    //    int ret_true0=0;
    //    int ret_1=0;
    //    int ret_2=0;
    //
    //    int ret_3=0;
    //    int ret_4=0;
    //    int ret_5=0;
    //    int ret_6=0;
    //    int ret_true=0;
    //
    float* imgFrameMaxGrad;
    
    
    
    //printf("current frame: height:%d,width:%d",util::ORIG_ROWS,util::ORIG_COLS);
    
    int idx=0;
    
    for(int y=ymin;y<ymax; y++)
    {
        imgFrameMaxGrad=keyFrame->maxAbsGradient.ptr<float>(y);
        
        for(int x=3;x<util::ORIG_COLS-3;x++) //column boundary width is 3 pixels
        {
            idx=x+y*util::ORIG_COLS; //idx is used to go to the current pixel
            
            depthhypothesis* target = &currentDepthHypothesis[idx]; //target pixel
            bool hasHypothesis = target->isValid;//using only non zero depth points
            
            // ======== 1. check absolute grad =========
            
            if(hasHypothesis && imgFrameMaxGrad[x]< util::MIN_ABS_GRAD_DECREASE) //if pixel is valid but bgradient below threshold
            {
                target->isValid = false; //invalidate it
                //PRINTF("\nmax gradient : %f,target rejected",imgFrameMaxGrad[x]);
                //cout<<"\nmax gradient < decrease ; target rejected :"<<imgFrameMaxGrad[x];
                continue;
            }
            
            if(imgFrameMaxGrad[x] < util::MIN_ABS_GRAD_CREATE || target->blacklisted < util::MIN_BLACKLIST)
            {
                //do nothing
                
                //cout<<"\n\nmax gradient < decrease ; target rejected :"<<imgFrameMaxGrad[x]<<"  , blacklisted :"<<target->blacklisted;
                continue;
            }
            
            
            if(!hasHypothesis) //if pixel is not valid but is above gradient threshold, create an estimate
            {
                
                return_val=observeDepthCreate(x, y, idx); //does not have hypothesis
                
                if(return_val==1)
                    successes_create++;
                
                //                switch(return_val)
                //                {
                //                    case -1: ret_10++;
                //                        break;
                //                    case -2: ret_20++;
                //                        break;
                //                    case 1: ret_true0++;
                //                        break;
                //                }
                
            }
            else //if pixel is valid and is above gradient threshold, update its depth estimate
            {
                
                return_val= observeDepthUpdate(x, y, idx);
                
                if(return_val==1)
                    successes_update++;
                
                //                switch(return_val)
                //                {
                //                    case -1: ret_1++;
                //                        break;
                //                    case -2: ret_2++;
                //                        break;
                //                    case -3: ret_3++;
                //                        break;
                //                    case -4: ret_4++;
                //                        break;
                //                    case -5: ret_5++;
                //                        break;
                //                    case -6: ret_6++;
                //                        break;
                //                    case 1: ret_true++;
                //                        break;
                //                }
                
                
            }
            
            
        }
    }
    end=clock();
    
    //cout <<"\nTime observe depth row  parallel: "<<int(util::FLAG_DO_PARALLEL_DEPTH_ESTIMATION)<<"  "<<float( end - start) / CLOCKS_PER_SEC<<" "<<ymin<<" "<<ymax<<endl;
    //printf("\nObserve depth row3 YMIN: %d  YMAX: %d",ymin,ymax);
    /*
     cout<<"\n\nsuccess_create_calls :"<<successes_create;
     cout<<"\n return -1 = "<<ret_10;
     cout<<"\n return -2 = "<<ret_20;
     cout<<"\n return TRUE = "<<ret_true0;
     
     cout<<"\n\nsuccess_update_calls :"<<successes_update;
     cout<<"\n return -1 = "<<ret_1;
     cout<<"\n return -2 = "<<ret_2;
     cout<<"\n return -3 = "<<ret_3;
     cout<<"\n return -4 = "<<ret_4;
     cout<<"\n return -5 = "<<ret_5;
     cout<<"\n return -6 = "<<ret_6;
     cout<<"\n return TRUE = "<<ret_true;
     */
    
    return;
}


//this function creates a depth estimate if the epipolar and stereo constraints are satisfied
int depthMap::observeDepthCreate(const int &x, const int &y, const int &idx)
{
    PRINTF("\nCreating Depth for current frame Id: %d, pixel Id: %d", currentFrame->frameId, idx);
    
    depthhypothesis* target = &currentDepthHypothesis[idx];

  //  Frame* refFrame = activeKeyFrameIsReactivated ? newest_keyFrame : oldest_keyFrame;
    /*
    if(refFrame->getTrackingParent() == activeKeyFrame)
    {
        //bool* wasGoodDuringTracking = refFrame->refPixelWasGoodNoCreate();
        //if(wasGoodDuringTracking != 0 && !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) + (util::ORIG_COLS >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)])
        //need to add condition to check if x and y lie out of image plane
        {
            if(plotStereoImages)
                debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,0); // BLUE for SKIPPED NOT GOOD TRACKED
            return false;
        }
    }
     */
    
    //checks epipolar constraints
    float epx, epy;
    bool isGood = makeAndCheckEPL(x, y, &epx, &epy); // ! returns 1 if x and y are good and epipolar line created
    if(!isGood) return -1; //epl fails
    //if(enablePrintDebugInfo) stats->num_observe_create_attempted++;
    
    //checks stereo constraints
    float new_u = x;
    float new_v = y;
    float result_idepth, result_var, result_eplLength;
    float error = doLineStereo(new_u,new_v,epx,epy,0.0f, 1.0f, 1.0f/util::MIN_DEPTH,result_idepth, result_var, result_eplLength);
    
    //cout<<"\nObserve depth CREATE: stereo error= "<<error;
    
    if(error == -3 || error == -2)
    {
        target->blacklisted--;
        //if(enablePrintDebugInfo) stats->num_observe_blacklisted++;
    }
    if(error < 0 || result_var > util::MAX_VAR)
        return -2;
    
    
    // reaches here when stereo successful
    //add hypothesis
    result_idepth = UNZERO(result_idepth);
    target->invDepth=result_idepth;
    target->variance=result_var;
    target->invDepthSmoothed=-1;
    target->varianceSmoothed=-1;
    target->validity_counter=util::VALIDITY_COUNTER_INITIAL_OBSERVE;
    target->isValid=true;
    target->blacklisted=0;
    
    //*target = DepthMapPixelHypothesis(result_idepth,result_var,VALIDITY_COUNTER_INITIAL_OBSERVE);
   // if(plotStereoImages)
   //     debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,255,255); // white for GOT CREATED
    //if(enablePrintDebugInfo) stats->num_observe_created++;
    
    return 1; //returns one is new depth has been created
    
}

//checks espipolar constraints
bool depthMap::makeAndCheckEPL(const int x, const int y, float* pepx, float* pepy)
{
    int idx = x+y*util::ORIG_COLS;
    
    static int length=0;
    static int grad=0;
    static int angle=0;
    static int num_true=0;
    

    //PRINTF("\nMaking and Checking Epipolar Line for potential Referance frame for current Frame Id: %d, pixel Id: %d", currentFrame->frameId, idx);

    float epx = - util::ORIG_FX * currentFrame->SE3poseOtherWrtThis_t(0,0) +currentFrame->SE3poseOtherWrtThis_t(2,0)*(x - util::ORIG_CX);
    float epy = - util::ORIG_FY * currentFrame->SE3poseOtherWrtThis_t(1,0) +currentFrame->SE3poseOtherWrtThis_t(2,0)*(y - util::ORIG_CY);
    
      //  cout<<"\n exp : epy = "<<epx<<" : "<<epy;
        
    if(isnan(epx+epy))
    {   //PRINTF("\nNAN")
        //cout<<"\nNan";
        return false;
    }
    
    // ======== check epl length =========
    float eplLengthSquared = epx*epx+epy*epy;
    
    if(eplLengthSquared < util::MIN_EPL_LENGTH_SQUARED)
    {
        //PRINTF("\neplLengthSquared %f", eplLengthSquared);
        // if(enablePrintDebugInfo) stats->num_observe_skipped_small_epl++;
      //cout<<"\neplLengthSquared : "<<eplLengthSquared;
        length++;
        return false;
    }
    

    uchar* img_ptr_centre=keyFrame->image.ptr<uchar>(y);
    uchar* img_ptr_up=keyFrame->image.ptr<uchar>(y-1);
    uchar* img_ptr_down=keyFrame->image.ptr<uchar>(y+1);

    // ===== check epl-grad magnitude ======
        
    float gx=img_ptr_centre[x+1]-img_ptr_centre[x-1];
    float gy=img_ptr_down[x]-img_ptr_up[x];
        
    float eplGradSquared = gx * epx + gy * epy;
   //cout<<" \n eplGradSquared: "<<eplGradSquared*eplGradSquared;

    eplGradSquared = eplGradSquared*eplGradSquared / eplLengthSquared;	// square and norm with epl-length
    
    if(eplGradSquared < util::MIN_EPL_GRAD_SQUARED)
    {
        //PRINTF("\nepl-grad magnitude");
        //if(enablePrintDebugInfo) stats->num_observe_skipped_small_epl_grad++;
       /*
      cout<<"\n exp : epy = "<<epx<<" : "<<epy;
      cout<<" |  gx : gy = "<<gx<<" : "<<gy; */
     //cout<<"\nepl-grad magnitude=  "<<eplGradSquared;
        grad++;
        return false;
    }
        
    // ===== check epl-grad angle ======
    if(eplGradSquared / (gx*gx+gy*gy) < util::MIN_EPL_ANGLE_SQUARED)
    {
        //PRINTF("\nepl-grad angle");
        //if(enablePrintDebugInfo) stats->num_observe_skipped_small_epl_angle++;
      //cout<<"\nepl-grad angle= "<<eplGradSquared/(gx*gx+gy*gy);
        angle++;
        return false;
    }
        
    // ===== DONE - return "normalized" epl =====
    float fac = util::GRADIENT_SAMPLE_DIST / sqrt(eplLengthSquared);
    *pepx = epx * fac;
    *pepy = epy * fac;
    //cout<<"\nTRUE, pepx : pepy"<<*pepx<<" , "<<*pepy;
     
    //PRINTF("\nEpl within limits,pepx: %f, pepy: %f",*pepx,*pepy);
    num_true++;
    return true;
    
}


// find pixel in image (do stereo along epipolar line).
// mat: NEW image
// KinvP: point in OLD image (Kinv * (u_old, v_old, 1)), projected
// trafo: x_old = trafo * x_new; (from new to old image)
// realVal: descriptor in OLD image.
// returns: result_idepth : point depth in new camera's coordinate system
// returns: result_u/v : point's coordinates in new camera's coordinate system
// returns: idepth_var: (approximated) measurement variance of inverse depth of result_point_NEW
// returns error if sucessful; -1 if out of bounds, -2 if not found, -3 for big error and -4 for invalid epipolar line/ points
//need gradient, pose, refernce images
float depthMap::doLineStereo( const float u, const float v, const float epxn, const float epyn,  const float min_idepth, const float prior_idepth, float max_idepth, float &result_idepth, float &result_var, float &result_eplLength)
{
    //PRINTF("\nCalculating Line Stereo for selected Keyframe with Id: %d using current Frame Id as reference frame: %d, pixel Id: ?", keyFrame->frameId , currentFrame->frameId);
    //if(enablePrintDebugInfo) stats->num_stereo_calls++;
    
    Eigen::Vector3f KinvP = Eigen::Vector3f(util::ORIG_FX_INV*u+util::ORIG_CX_INV,util::ORIG_FY_INV*v+util::ORIG_CY_INV,1.0f);
    Eigen::Vector3f pInf = currentFrame->K_SE3poseThisWrtOther_r* KinvP; // ! point at infinty ? !
    Eigen::Vector3f pReal = pInf / prior_idepth + currentFrame->K_SE3poseThisWrtOther_t;
    
    float rescaleFactor =pReal[2]* prior_idepth;
    
    float firstX = u - 2*epxn*rescaleFactor;
    float firstY = v - 2*epyn*rescaleFactor;
    float lastX = u + 2*epxn*rescaleFactor;
    float lastY = v + 2*epyn*rescaleFactor;
    
    
    // util::ORIG_COLS - 2 and util::ORIG_ROWS - 2 comes from the one-sided gradient calculation at the bottom
    if (firstX <= 0 || firstX >= util::ORIG_COLS - 2
        || firstY <= 0 || firstY >= util::ORIG_ROWS - 2
        || lastX <= 0 || lastX >= util::ORIG_COLS - 2
        || lastY <= 0 || lastY >= util::ORIG_ROWS - 2)
    {
        return -1;
       
    }
    
    //printf("\nrescale Factor as calculated in do line stereo %f",rescaleFactor);
    if(!(rescaleFactor > 0.7f && rescaleFactor < 1.4f))
    {
        //if(enablePrintDebugInfo) stats->num_stereo_rescale_oob++;
        return -1;
      
    }
    
    // calculate values to search for on keyframe
    float realVal_p1 = keyFrame->getInterpolatedElement(u + epxn*rescaleFactor, v + epyn*rescaleFactor);
    float realVal_m1 = keyFrame->getInterpolatedElement(u - epxn*rescaleFactor, v - epyn*rescaleFactor);
    float realVal = keyFrame->getInterpolatedElement(u, v);
    float realVal_m2 = keyFrame->getInterpolatedElement(u - 2*epxn*rescaleFactor, v - 2*epyn*rescaleFactor);
    float realVal_p2 = keyFrame->getInterpolatedElement(u + 2*epxn*rescaleFactor, v + 2*epyn*rescaleFactor);
    
    Eigen::Vector3f pClose = pInf + currentFrame->K_SE3poseThisWrtOther_t*max_idepth;
    // if the assumed close-point lies behind the
    // image, have to change that.
    if(pClose[2] < 0.001f)
    {
        max_idepth = (0.001f-pInf[2]) / currentFrame->K_SE3poseThisWrtOther_t(2,0);
        pClose = pInf + currentFrame->K_SE3poseThisWrtOther_t*max_idepth;
    }
    pClose = pClose / pClose[2]; // pos in new image of point (xy), assuming max_idepth
    
    
    Eigen::Vector3f pFar = pInf + currentFrame->K_SE3poseThisWrtOther_t*min_idepth;
    
    
    // if the assumed far-point lies behind the image or closter than the near-point,
    // we moved past the Point it and should stop.
    if(pFar[2] < 0.001f || max_idepth < min_idepth)
    {
       // if(enablePrintDebugInfo) stats->num_stereo_inf_oob++;
        //cout<<"\npfar[2] : "<<pFar[2]<<" ,max_idepth : min_idepth = "<<max_idepth<<" : "<<min_idepth;
        return -1;
    }
    pFar = pFar / pFar[2]; // pos in new image of point (xy), assuming min_idepth
    
    
    // check for nan due to eg division by zero.
    if(isnan((float)(pFar[0]+pClose[0])))
    {    //cout<<"\nNan";
        return -4;
    }//  if pFar and pClose if NaN
    
    // calculate increments in which we will step through the epipolar line.
    // they are sampleDist (or half sample dist) long
    float incx = pClose[0] - pFar[0];  
    float incy = pClose[1] - pFar[1];
    float eplLength = sqrt(incx*incx+incy*incy);
    if(!eplLength > 0 || std::isinf(eplLength))
    
    {
       // cout<<"\nelp length not greater than 0";
        return   -4;
    }// not valid epipolar length that is <0 or inf
    
    if(eplLength > util::MAX_EPL_LENGTH_CROP)
    {
        pClose[0] = pFar[0] + incx*util::MAX_EPL_LENGTH_CROP/eplLength;
        pClose[1] = pFar[1] + incy*util::MAX_EPL_LENGTH_CROP/eplLength;
    }
    
    incx *= util::GRADIENT_SAMPLE_DIST/eplLength;
    incy *= util::GRADIENT_SAMPLE_DIST/eplLength;
    
    
    // extend one sample_dist to left & right.
    pFar[0] -= incx;
    pFar[1] -= incy;
    pClose[0] += incx;
    pClose[1] += incy;
    
    
    // make epl long enough (pad a little bit).
    if(eplLength < util::MIN_EPL_LENGTH_CROP)
    {
        float pad = (util::MIN_EPL_LENGTH_CROP - (eplLength)) / 2.0f;
        pFar[0] -= incx*pad;
        pFar[1] -= incy*pad;
        
        pClose[0] += incx*pad;
        pClose[1] += incy*pad;
    }
    
    // if inf point is outside of image: skip pixel.
    if(
       pFar[0] <= util::SAMPLE_POINT_TO_BORDER ||
       pFar[0] >= util::ORIG_COLS- util::SAMPLE_POINT_TO_BORDER ||
       pFar[1] <= util::SAMPLE_POINT_TO_BORDER ||
       pFar[1] >= util::ORIG_ROWS- util::SAMPLE_POINT_TO_BORDER)
    {
        //if(enablePrintDebugInfo) stats->num_stereo_inf_oob++;
        //cout<<"\npfar[0] : pfar[1] = "<<pFar[0]<<" : "<<pFar[1];
        return -1;     // if infinity point is outside image => out of bounds(oob) => skip , return -1
    }
    
    // if near point is outside: move inside, and test length again.
    if(
       pClose[0] <= util::SAMPLE_POINT_TO_BORDER ||
       pClose[0] >= util::ORIG_COLS- util::SAMPLE_POINT_TO_BORDER ||
       pClose[1] <= util::SAMPLE_POINT_TO_BORDER ||
       pClose[1] >= util::ORIG_ROWS- util::SAMPLE_POINT_TO_BORDER)
    {
        if(pClose[0] <= util::SAMPLE_POINT_TO_BORDER)
        {
            float toAdd = (util::SAMPLE_POINT_TO_BORDER - pClose[0]) / incx;
            pClose[0] += toAdd * incx;
            pClose[1] += toAdd * incy;
        }
        else if(pClose[0] >= util::ORIG_COLS- util::SAMPLE_POINT_TO_BORDER)
        {
            float toAdd = (util::ORIG_COLS- util::SAMPLE_POINT_TO_BORDER - pClose[0]) / incx;
            pClose[0] += toAdd * incx;
            pClose[1] += toAdd * incy;
        }
        
        if(pClose[1] <= util::SAMPLE_POINT_TO_BORDER)
        {
            float toAdd = (util::SAMPLE_POINT_TO_BORDER - pClose[1]) / incy;
            pClose[0] += toAdd * incx;
            pClose[1] += toAdd * incy;
        }
        else if(pClose[1] >= util::ORIG_ROWS- util::SAMPLE_POINT_TO_BORDER)
        {
            float toAdd = (util::ORIG_ROWS- util::SAMPLE_POINT_TO_BORDER - pClose[1]) / incy;
            pClose[0] += toAdd * incx;
            pClose[1] += toAdd * incy;
        }
        
        
        
        // get new epl length
        float fincx = pClose[0] - pFar[0];
        float fincy = pClose[1] - pFar[1];
        float newEplLength = sqrt(fincx*fincx+fincy*fincy);
        
        
        // test again
        if(
           pClose[0] <= util::SAMPLE_POINT_TO_BORDER ||
           pClose[0] >= util::ORIG_COLS- util::SAMPLE_POINT_TO_BORDER ||
           pClose[1] <= util::SAMPLE_POINT_TO_BORDER ||
           pClose[1] >= util::ORIG_ROWS- util::SAMPLE_POINT_TO_BORDER ||
           newEplLength < 8.0f
           )
        {
            //if(enablePrintDebugInfo) stats->num_stereo_near_oob++;
           //cout<<"\npClose[0] : pclose[1] = "<<pClose[0]<<" : "<<pClose[1]<<" , newepllength : "<<newEplLength;
            return -1;  // if still oob and new epl length is less,  returns -1
        }
        
        
    }
    
    
    // from here on:
    // - pInf: search start-point
    // - p0: search end-point
    // - incx, incy: search steps in pixel
    // - eplLength, min_idepth, max_idepth: determines search-resolution, i.e. the result's variance.
    
    float cpx = pFar[0];
    float cpy =  pFar[1];
    
    float val_cp_m2 = currentFrame->getInterpolatedElement(cpx-2.0f*incx, cpy-2.0f*incy);
    float val_cp_m1 = currentFrame->getInterpolatedElement(cpx-incx, cpy-incy);
    float val_cp = currentFrame->getInterpolatedElement(cpx, cpy);
    float val_cp_p1 =currentFrame->getInterpolatedElement(cpx+incx, cpy+incy);
    float val_cp_p2;
    
    
    
    /*
     * Subsequent exact minimum is found the following way:
     * - assuming lin. interpolation, the gradient of Error at p1 (towards p2) is given by
     *   dE1 = -2sum(e1*e1 - e1*e2)
     *   where e1 and e2 are summed over, and are the residuals (not squared).
     *
     * - the gradient at p2 (coming from p1) is given by
     * 	 dE2 = +2sum(e2*e2 - e1*e2)
     *
     * - linear interpolation => gradient changes linearely; zero-crossing is hence given by
     *   p1 + d*(p2-p1) with d = -dE1 / (-dE1 + dE2).
     *
     *
     *
     * => I for later exact min calculation, I need sum(e_i*e_i),sum(e_{i-1}*e_{i-1}),sum(e_{i+1}*e_{i+1})
     *    and sum(e_i * e_{i-1}) and sum(e_i * e_{i+1}),
     *    where i is the respective winning index.
     */
    
    
    // walk in equally sized steps, starting at depth=infinity.
    int loopCounter = 0;
    float best_match_x = -1;
    float best_match_y = -1;
    float best_match_err = 1e50;
    float second_best_match_err = 1e50;
    
    // best pre and post errors.
    float best_match_errPre=NAN, best_match_errPost=NAN, best_match_DiffErrPre=NAN, best_match_DiffErrPost=NAN;
    bool bestWasLastLoop = false;
    
    float eeLast = -1; // final error of last comp.
    
    // alternating intermediate vars
    float e1A=NAN, e1B=NAN, e2A=NAN, e2B=NAN, e3A=NAN, e3B=NAN, e4A=NAN, e4B=NAN, e5A=NAN, e5B=NAN;
    
    int loopCBest=-1, loopCSecond =-1;  // ! loop counter corresponding to best mactch and second best match !
    while(((incx < 0) == (cpx > pClose[0]) && (incy < 0) == (cpy > pClose[1])) || loopCounter == 0)
    {
        // interpolate one new point
        //Interpolation needed!!
        val_cp_p2=currentFrame->getInterpolatedElement(cpx+2*incx, cpy+2*incy);
        
        
        // hacky but fast way to get error and differential error: switch buffer variables for last loop.
        float ee = 0;
        if(loopCounter%2==0)
        {
            // calc error and accumulate sums.
            e1A = val_cp_p2 - realVal_p2;ee += e1A*e1A;
            e2A = val_cp_p1 - realVal_p1;ee += e2A*e2A;
            e3A = val_cp - realVal;      ee += e3A*e3A;
            e4A = val_cp_m1 - realVal_m1;ee += e4A*e4A;
            e5A = val_cp_m2 - realVal_m2;ee += e5A*e5A;
        }
        else
        {
            // calc error and accumulate sums.
            e1B = val_cp_p2 - realVal_p2;ee += e1B*e1B;
            e2B = val_cp_p1 - realVal_p1;ee += e2B*e2B;
            e3B = val_cp - realVal;      ee += e3B*e3B;
            e4B = val_cp_m1 - realVal_m1;ee += e4B*e4B;
            e5B = val_cp_m2 - realVal_m2;ee += e5B*e5B;
        }
        
        
        // do I have a new winner??
        // if so: set.
        if(ee < best_match_err)
        {
            // put to second-best
            second_best_match_err=best_match_err;
            loopCSecond = loopCBest;
            
            // set best.
            best_match_err = ee;
            loopCBest = loopCounter;
            
            best_match_errPre = eeLast;
            best_match_DiffErrPre = e1A*e1B + e2A*e2B + e3A*e3B + e4A*e4B + e5A*e5B;
            best_match_errPost = -1;
            best_match_DiffErrPost = -1;
            
            best_match_x = cpx;
            best_match_y = cpy;
            bestWasLastLoop = true;
        }
        // otherwise: the last might be the current winner, in which case i have to save these values.
        else
        {
            if(bestWasLastLoop)
            {
                best_match_errPost = ee;
                best_match_DiffErrPost = e1A*e1B + e2A*e2B + e3A*e3B + e4A*e4B + e5A*e5B;
                bestWasLastLoop = false;
            }
            
            // collect second-best:
            // just take the best of all that are NOT equal to current best.
            if(ee < second_best_match_err)
            {
                second_best_match_err=ee;
                loopCSecond = loopCounter;
            }
        }
        
        
        // shift everything one further.
        eeLast = ee;
        val_cp_m2 = val_cp_m1; val_cp_m1 = val_cp; val_cp = val_cp_p1; val_cp_p1 = val_cp_p2;
        
       // if(enablePrintDebugInfo) stats->num_stereo_comparisons++;
        
        cpx += incx;
        cpy += incy;
        
        loopCounter++;
        
        //cout<<"\n VAL : "<<val_cp_p1<<" , "<<val_cp_m1<<" , "<<val_cp<<" , "<<val_cp_m2<<" , "<<val_cp_p2;
        //cout<<"\nerror: "<<best_match_err;

    }
    
    // if error too big, will return -3, otherwise -2.
    if(best_match_err > 4.0f*(float)util::MAX_ERROR_STEREO)
    {
       // if(enablePrintDebugInfo) stats->num_stereo_invalid_bigErr++;
       //cout<<"\nerror too big";
        return -3;  // if best match error too big returns -3
    }
    
    
    // check if clear enough winner
    if(abs(loopCBest - loopCSecond) > 1.0f && util::MIN_DISTANCE_ERROR_STEREO * best_match_err > second_best_match_err)
    {
         //if(enablePrintDebugInfo) stats->num_stereo_invalid_unclear_winner++;
        //cout<<"\ninvalid unclear winner => RETURN -2";
        return -2;   // ! return -2 if not clear winner
    }
    
    bool didSubpixel = false;
    bool useSubpixelStereo = true;
    if(useSubpixelStereo)  // ! ???? !
    {
        // ================== compute exact match =========================
        // compute gradients (they are actually only half the real gradient)
        float gradPre_pre = -(best_match_errPre - best_match_DiffErrPre);
        float gradPre_this = +(best_match_err - best_match_DiffErrPre);
        float gradPost_this = -(best_match_err - best_match_DiffErrPost);
        float gradPost_post = +(best_match_errPost - best_match_DiffErrPost);
        
        // final decisions here.
        bool interpPost = false;
        bool interpPre = false;
       
        
        // if one is oob: return false.
        if(1 && (best_match_errPre < 0 || best_match_errPost < 0))
        {
            //stats->num_stereo_invalid_atEnd++;
            //do nothing
            ;
        }
        
        
        // - if zero-crossing occurs exactly in between (gradient Inconsistent),
        
        else if((gradPost_this < 0) ^ (gradPre_this < 0))
        {
            
            // return exact pos, if both central gradients are small compared to their counterpart.
           // if(enablePrintDebugInfo && (gradPost_this*gradPost_this > 0.1f*0.1f*gradPost_post*gradPost_post ||
            ; //                             gradPre_this*gradPre_this > 0.1f*0.1f*gradPre_pre*gradPre_pre))
           //     stats->num_stereo_invalid_inexistantCrossing++;
        }
        
        // if pre has zero-crossing
        else if((gradPre_pre < 0) ^ (gradPre_this < 0))
        {
            // if post has zero-crossing
            if((gradPost_post < 0) ^ (gradPost_this < 0))
            {
                ; //if(enablePrintDebugInfo) stats->num_stereo_invalid_twoCrossing++;
            }
            else
                interpPre = true;
        }
        
        // if post has zero-crossing
        else if((gradPost_post < 0) ^ (gradPost_this < 0))
        {
            interpPost = true;
        }
        
        // if none has zero-crossing
        else
        {
          //  if(enablePrintDebugInfo) stats->num_stereo_invalid_noCrossing++;
        }
        
        
        // DO interpolation!
        // minimum occurs at zero-crossing of gradient, which is a straight line => easy to compute.
        // the error at that point is also computed by just integrating.
        
        if(interpPre)
        {
            float d = gradPre_this / (gradPre_this - gradPre_pre);
            best_match_x -= d*incx;
            best_match_y -= d*incy;
            best_match_err = best_match_err - 2*d*gradPre_this - (gradPre_pre - gradPre_this)*d*d;
            //if(enablePrintDebugInfo) stats->num_stereo_interpPre++;
            didSubpixel = true;
            
        }
        else if(interpPost)
        {
            float d = gradPost_this / (gradPost_this - gradPost_post);
            best_match_x += d*incx;
            best_match_y += d*incy;
            best_match_err = best_match_err + 2*d*gradPost_this + (gradPost_post - gradPost_this)*d*d;
            //if(enablePrintDebugInfo) stats->num_stereo_interpPost++;
            didSubpixel = true;
        }
        else
        {
           // if(enablePrintDebugInfo) stats->num_stereo_interpNone++;
        }
    }
    
    
    // sampleDist is the distance in pixel at which the realVal's were sampled
    float sampleDist = util::GRADIENT_SAMPLE_DIST*rescaleFactor;
    
    float gradAlongLine = 0;
    float tmp = realVal_p2 - realVal_p1;  gradAlongLine+=tmp*tmp;
    tmp = realVal_p1 - realVal;  gradAlongLine+=tmp*tmp;
    tmp = realVal - realVal_m1;  gradAlongLine+=tmp*tmp;
    tmp = realVal_m1 - realVal_m2;  gradAlongLine+=tmp*tmp;
    
    gradAlongLine /= sampleDist*sampleDist;
    
    // check if interpolated error is OK. use evil hack to allow more error if there is a lot of gradient.
    if(best_match_err > (float)util::MAX_ERROR_STEREO + sqrtf( gradAlongLine)*20)
    {
        //if(enablePrintDebugInfo) stats->num_stereo_invalid_bigErr++;
       //cout<<"\nStereo invalid, error big ";
        return -3;    // if big error, return -3
    }
    
    
    // ================= calc depth (in KF) ====================
    // * KinvP = Kinv * (x,y,1); where x,y are pixel coordinates of point we search for, in the KF.
    // * best_match_x = x-coordinate of found correspondence in the reference frame.
    
    float idnew_best_match;	// depth in the new image
    float alpha; // d(idnew_best_match) / d(disparity in pixel) == conputed inverse depth derived by the pixel-disparity.
    if(incx*incx>incy*incy)
    {

        float oldX = util::ORIG_FX_INV*best_match_x+util::ORIG_CX_INV;
        float nominator = (oldX*currentFrame->SE3poseThisWrtOther_t(2,0) - currentFrame->SE3poseThisWrtOther_t(0,0));
        float dot0 = KinvP.dot(currentFrame->SE3poseThisWrtOther_r.row(0));
        float dot2 = KinvP.dot(currentFrame->SE3poseThisWrtOther_r.row(2));
        
        idnew_best_match = (dot0 - oldX*dot2) / nominator;
        alpha = incx*util::ORIG_FX_INV*(dot0*currentFrame->SE3poseThisWrtOther_t(2,0) - dot2*currentFrame->SE3poseThisWrtOther_t(0,0)) / (nominator*nominator);
        
    }
    else
    {
        float oldY = util::ORIG_FY_INV*best_match_y+util::ORIG_CY_INV;
        
        float nominator = (oldY*currentFrame->SE3poseThisWrtOther_t(2,0) - currentFrame->SE3poseThisWrtOther_t(1,0));
        float dot1 = KinvP.dot(currentFrame->SE3poseThisWrtOther_r.row(1));
        float dot2 = KinvP.dot(currentFrame->SE3poseThisWrtOther_r.row(2));
        
        idnew_best_match = (dot1 - oldY*dot2) / nominator;
        alpha = incy*util::ORIG_FX_INV*(dot1*currentFrame->SE3poseThisWrtOther_t(2,0) - dot2*currentFrame->SE3poseThisWrtOther_t(1,0)) / (nominator*nominator);
        
    }
    

    if(idnew_best_match < 0)
    {
        //if(enablePrintDebugInfo) stats->num_stereo_negative++;
        
        //do not allow for right now
        //if(!allowNegativeIdepths)
       // cout<<"\nStereo negative  RETURN -2";
            return -2;   // depth in new image is negative, return -2
    }
    
    //if(enablePrintDebugInfo) stats->num_stereo_successfull++;    // ! if control reaches here, successful point
    
    // ================= calc var (in NEW image) ====================
    
    // calculate error from photometric noise
    float photoDispError = 4.0f * util::CAMERA_PIXEL_NOISE / (gradAlongLine + util::DIVISION_EPS);
    
    float trackingErrorFac =  0.25f*1.0f;// 0.25f*(1.0f+keyFrame->initialTrackedResidual);/// check from logs
    
    // calculate error from geometric noise (wrong camera pose / calibration)

    Eigen::Vector2f gradsInterp;
    gradsInterp[0]= keyFrame->getInterpolatedElement(u, v,"gradx");// to be checked
    gradsInterp[1]=keyFrame->getInterpolatedElement(u, v,"grady");
    float geoDispError = (gradsInterp[0]*epxn + gradsInterp[1]*epyn) + util::DIVISION_EPS;
    geoDispError = trackingErrorFac*trackingErrorFac*(gradsInterp[0]*gradsInterp[0] + gradsInterp[1]*gradsInterp[1]) / (geoDispError*geoDispError);
    
    //geoDispError *= (0.5 + 0.5 *result_idepth) * (0.5 + 0.5 *result_idepth);
    
    // final error consists of a small constant part (discretization error),
    // geometric and photometric error.
    result_var = alpha*alpha*((didSubpixel ? 0.05f : 0.5f)*sampleDist*sampleDist +  geoDispError + photoDispError);	// square to make variance
    
/*
    if(plotStereoImages)
    {
        if(rand()%5==0)
        {
            //if(rand()%500 == 0)
            //	printf("geo: %f, photo: %f, alpha: %f\n", sqrt(geoDispError), sqrt(photoDispError), alpha, sqrt(result_var));
            
            
            //int idDiff = (keyFrame->pyramidID - keyFrame->id);
            //cv::Scalar color = cv::Scalar(0,0, 2*idDiff);// bw
            
            //cv::Scalar color = cv::Scalar(sqrt(result_var)*2000, 255-sqrt(result_var)*2000, 0);// bw
            
            //			float eplLengthF = std::min((float)MIN_EPL_LENGTH_CROP,(float)eplLength);
            //			eplLengthF = std::max((float)MAX_EPL_LENGTH_CROP,(float)eplLengthF);
            //
            //			float pixelDistFound = sqrtf((float)((pReal[0]/pReal[2] - best_match_x)*(pReal[0]/pReal[2] - best_match_x)
            //					+ (pReal[1]/pReal[2] - best_match_y)*(pReal[1]/pReal[2] - best_match_y)));
            //
            float fac = best_match_err / ((float)util::MAX_ERROR_STEREO + sqrtf( gradAlongLine)*20);
            
            cv::Scalar color = cv::Scalar(255*fac, 255-255*fac, 0);// bw
            
            
     
             if(rescaleFactor > 1)
             color = cv::Scalar(500*(rescaleFactor-1),0,0);
             else
             color = cv::Scalar(0,500*(1-rescaleFactor),500*(1-rescaleFactor));
     
            
            ???
            //cv::line(debugImageStereoLines,cv::Point2f(pClose[0], pClose[1]),cv::Point2f(pFar[0], pFar[1]),color,1,8,0);  // ! look up !
        }
    }
*/
    
    result_idepth = idnew_best_match;
    
    result_eplLength = eplLength;
    
    return best_match_err;  // returns best match error if successful
}


int depthMap::observeDepthUpdate(const int &x, const int &y, const int &idx)
{
    PRINTF("\nUpdating Depth for current frame Id: %d, pixel Id: %d", currentFrame->frameId, idx);
    depthhypothesis* target = &currentDepthHypothesis[idx];

    //frame* refFrame;
   /*
    if(!activeKeyFrameIsReactivated)
    {
        if((int)target->nextStereoFrameMinID - keyFrameByID_offset >= (int)keyFrameByID.size())
        {
            if(plotStereoImages)
                debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,255,0);	// GREEN FOR skip
            
            if(enablePrintDebugInfo) stats->num_observe_skip_alreadyGood++;
            return false;
        }
        
        if((int)target->nextStereoFrameMinID - keyFrameByID_offset < 0)
            refFrame = oldest_keyFrame;
        else
            refFrame = keyFrameByID[(int)target->nextStereoFrameMinID - keyFrameByID_offset];
    }
    else
        refFrame = newest_keyFrame;*/
    /*
    if(keyFrame->getTrackingParent() == activeKeyFrame)
    {
        bool* wasGoodDuringTracking = refFrame->refPixelWasGoodNoCreate();
        if(wasGoodDuringTracking != 0 && !wasGoodDuringTracking[(x >> SE3TRACKING_MIN_LEVEL) + (util::ORIG_COLS >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)])
        {
            if(plotStereoImages)
                debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,0); // BLUE for SKIPPED NOT GOOD TRACKED
            return false;
        }
    }
     */
    
    float epx, epy;
    bool isGood = makeAndCheckEPL(x, y, &epx, &epy);
    if(!isGood)   return -5; //return false;
    
    // which exact point to track, and where from.
    float sv = sqrt(target->varianceSmoothed);
    float min_idepth = target->invDepthSmoothed - sv*util::STEREO_EPL_VAR_FAC;
    float max_idepth = target->invDepthSmoothed + sv*util::STEREO_EPL_VAR_FAC;
    if(min_idepth < 0)
        min_idepth = 0;
    if(max_idepth > 1/util::MIN_DEPTH)
        max_idepth = 1/util::MIN_DEPTH;
    
    //stats->num_observe_update_attempted++;
    
    float result_idepth, result_var, result_eplLength;
    
    
    
    float error = doLineStereo(x,y,epx,epy,min_idepth, target->invDepthSmoothed ,max_idepth,result_idepth, result_var, result_eplLength);
    
   // cout<<"\nObserve depth UPDATE: stereo error= "<<error;
    
     //cout<<"\nprev idepth  "<<target->invDepth<<";"<<"new idepth estimate  "<<result_idepth<<endl;
    
    float diff = result_idepth - target->invDepthSmoothed;
    
    // if oob: (really out of bounds)
    if(error == -1)
    {
        // do nothing, pixel got oob, but is still in bounds in original. I will want to try again.
       // if(enablePrintDebugInfo) stats->num_observe_skip_oob++;
        //if(plotStereoImages)
         //   debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,255);	// RED FOR OOB
        return -1;
        //return false;
    }
    
    // if just not good for stereo (e.g. some inf / nan occured; has inconsistent minimum; ..)
    else if(error == -2)
    {
       // if(enablePrintDebugInfo) stats->num_observe_skip_fail++;
        //if(plotStereoImages)
        //    debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,0,255);	// PURPLE FOR NON-GOOD
        
        target->validity_counter -= util::VALIDITY_COUNTER_DEC;
        if(target->validity_counter < 0)
            target->validity_counter = 0;
        
       // target->nextStereoFrameMinID = 0;
        
        target->variance *= util::FAIL_VAR_INC_FAC;
        if(target->variance > util::MAX_VAR)
        {
            target->isValid = false;
            target->blacklisted--;
        }
        return -2;
        //return false;
    }
    
    // if not found (error too high)
    else if(error == -3)
    {
        //if(enablePrintDebugInfo) stats->num_observe_notfound++;
        //if(plotStereoImages)
          //  debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,0);	// BLACK FOR big not-found
       
        return -3;
       // return false;
    }
    
    else if(error == -4)
    {
        //if(plotStereoImages)
            //debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,0);	// BLACK FOR big arithmetic error
        
        return -4;
        //return false;
    }
    
    // if inconsistent
    else if(util::DIFF_FAC_OBSERVE*diff*diff > result_var + target->varianceSmoothed)
    {
        //if(enablePrintDebugInfo) stats->num_observe_inconsistent++;
        //if(plotStereoImages)
        //debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(255,255,0);	// Turkoise FOR big inconsistent
    
        target->variance *= util::FAIL_VAR_INC_FAC;
        if(target->variance > util::MAX_VAR)
            target->isValid = false;
        return -6;
        //return false;
    }
    
    
    else
    {
        // one more successful observation!
        //if(enablePrintDebugInfo) stats->num_observe_good++;
        //if(enablePrintDebugInfo) stats->num_observe_updated++;
        // do textbook ekf update:
        // increase var by a little (prediction-uncertainty)
        float id_var = target->variance*util::SUCC_VAR_INC_FAC;
        
        // update var with observation
        float w = result_var / (result_var + id_var);
        float new_idepth = (1-w)*result_idepth + w*target->invDepth;
        target->invDepth = UNZERO(new_idepth);
        
        // variance can only decrease from observation; never increase.
        id_var = id_var * w;
        if(id_var < target->variance)
            target->variance = id_var;
        
        // increase validity!
        target->validity_counter += util::VALIDITY_COUNTER_INC;
        
        
        float* maxgrad_ptr=keyFrame->maxAbsGradient.ptr<float>(y);
        //float* grady_ptr=keyFrame->gradienty.ptr<float>(0);
       
        
        float absGrad =maxgrad_ptr[x];  //keyFrameMaxGradBuf[idx];
        
        if(target->validity_counter > util::VALIDITY_COUNTER_MAX+absGrad*(util::VALIDITY_COUNTER_MAX_VARIABLE)/255.0f)
            target->validity_counter = util::VALIDITY_COUNTER_MAX+absGrad*(util::VALIDITY_COUNTER_MAX_VARIABLE)/255.0f;
        
        /*
        
        // increase Skip!
        if(result_eplLength < util::MIN_EPL_LENGTH_CROP)
        {
            float inc = activeKeyFrame->numFramesTrackedOnThis / (float)(activeKeyFrame->numMappedOnThis+5);
            if(inc < 3) inc = 3;
            
            inc +=  ((int)(result_eplLength*10000)%2);
            
            if(enablePrintDebugInfo) stats->num_observe_addSkip++;
            
            if(result_eplLength < 0.5*MIN_EPL_LENGTH_CROP)
                inc *= 3;
            target->nextStereoFrameMinID = refFrame->id() + inc;
        }
         */
        // if(plotStereoImages)
            //debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = cv::Vec3b(0,255,255); // yellow for GOT UPDATED
               
        return 1; 
        //return true;
    }
}



void depthMap::propagateDepth(frame* new_keyframe)
{
    PRINTF("\nPropagating depth from prev frame Id: %d to current frame Id: %d", keyFrame->frameId, currentFrame->frameId);
    //depthhypothesis* target = &currentDepthHypothesis[idx];
    
    
    // wipe depthmap
    for(depthhypothesis* pt = otherDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS-1; pt >= otherDepthHypothesis; pt--)
    {
        //PRINTF("%f",pt->invDepth);
        pt->isValid = false;
        pt->blacklisted = 0;
    }
    

    // re-usable values.
   // SE3 oldToNew_SE3 = se3FromSim3(new_keyframe->pose->thisToParent_raw).inverse();
    
    new_keyframe->calculateSE3poseOtherWrtThis(keyFrame);// parent id to be checked
    Eigen::MatrixXf oldToNew_SE3=new_keyframe->SE3poseThisWrtOther; //New Wrt Old
    Eigen::VectorXf trafoInv_t=new_keyframe->SE3poseThisWrtOther_t;
    Eigen::MatrixXf trafoInv_R=new_keyframe->SE3poseThisWrtOther_r;
    
    
    float* newKFMaxGrad_ptr = new_keyframe->maxAbsGradient.ptr<float>(0);
    uchar* sourceImg_ptr=keyFrame->image.ptr<uchar>(0);
    
    // go through all pixels of OLD image, propagating forwards.
    for(int y=0;y<util::ORIG_ROWS;y++)
    {
        newKFMaxGrad_ptr=new_keyframe->maxAbsGradient.ptr<float>(y);
        sourceImg_ptr=keyFrame->image.ptr<uchar>(y);
        
        for(int x=0;x<util::ORIG_COLS;x++)
        {
            depthhypothesis* source = currentDepthHypothesis + x + y*util::ORIG_COLS;
            
            if(!source->isValid)
            {
               // PRINTF("\nNot Valid!!!");
                continue;
            }
            
            
//            if(source->invDepth<0 || source->invDepthSmoothed<0)
//            {
//                printf("\nx: %d, y: %d, valid source_idepth: %f, valid source_idepth_smooth: %f", x, y, source->invDepth, source->invDepthSmoothed);
//                waitKey(0);
//            }
            
            //if(enablePrintDebugInfo) runningStats.num_prop_attempts++;
            
            Eigen::Vector3f pn = (trafoInv_R * Eigen::Vector3f(x*util::ORIG_FX_INV + util::ORIG_CX_INV,y*util::ORIG_FY_INV + util::ORIG_CY_INV,1.0f)) / source->invDepthSmoothed + trafoInv_t;
            

            float new_idepth = 1.0f / pn[2];
            
            
//            if(new_idepth<0)
//            {   printf("\nx: %d, y: %d, new_idepth: %f, source_invdepth: %f, source_invdepth_smooth: %f", x, y, new_idepth, source->invDepth, source->invDepthSmoothed);
//                cout<<"\ntrafoInv_R: \n"<<trafoInv_R;
//                cout<<"\nDeterminant of trafoInv_R: "<<trafoInv_R.determinant();
//                cout<<"\ntrafoInv_t: \n"<<trafoInv_t;
//                cout<<"\n pn: \n"<<pn;
//                cout<<"\n (trafoInv_R * Eigen::Vector3f(x*util::ORIG_FX_INV + util::ORIG_CX_INV,y*util::ORIG_FY_INV + util::ORIG_CY_INV,1.0f)): "<<(trafoInv_R * Eigen::Vector3f(x*util::ORIG_FX_INV + util::ORIG_CX_INV,y*util::ORIG_FY_INV + util::ORIG_CY_INV,1.0f));
//                
//                
//                printf("\nnew idepth: %f ", new_idepth);
//                
//                waitKey(0);
//            }
            
            
            
            
            //if(new_idepth<0 || source->invDepthSmoothed<0)
              
            
            //PRINTF("\nSource inverse depth : %f",source->invDepth);
            float u_new = pn[0]*new_idepth*util::ORIG_FX + util::ORIG_CX;
            float v_new = pn[1]*new_idepth*util::ORIG_FY + util::ORIG_CY;
            
            //PRINTF("\np0= %f ; p1= %f ; p2= %f", pn[0], pn[1], pn[2]);
            PRINTF("\nu= %f ; v= %f ", u_new, v_new);
            
            // check if still within image, if not: DROP.
            if(!(u_new > 2.1f && v_new > 2.1f && u_new < util::ORIG_COLS-3.1f && v_new < util::ORIG_ROWS-3.1f))
            {
               // if(enablePrintDebugInfo) runningStats.num_prop_removed_out_of_bounds++;
                //PRINTF("\nOut of bounds!!");
                continue;
            }
            
            int newIDX = (int)(u_new+0.5f) + ((int)(v_new+0.5f))*util::ORIG_COLS; //sub pixel ?
            float destAbsGrad = newKFMaxGrad_ptr[x];
            
            /*
            if(trackingWasGood != 0)
            {
                if(!trackingWasGood[(x >> SE3TRACKING_MIN_LEVEL) + (width >> SE3TRACKING_MIN_LEVEL)*(y >> SE3TRACKING_MIN_LEVEL)]
                   || destAbsGrad < MIN_ABS_GRAD_DECREASE)
                {
                    if(1) runningStats.num_prop_removed_colorDiff++;
                    continue;
                }
            }
            else*/
            
            float sourceColor =sourceImg_ptr[x];
            float destColor = new_keyframe->getInterpolatedElement(u_new, v_new);
            float residual = destColor - sourceColor;
            
            if(residual*residual / (util::MAX_DIFF_CONSTANT + util::MAX_DIFF_GRAD_MULT*destAbsGrad*destAbsGrad) > 1.0f || destAbsGrad < util::MIN_ABS_GRAD_DECREASE)
                {
                    //if(enablePrintDebugInfo) runningStats.num_prop_removed_colorDiff++;
                    //PRINTF("\nRemoved- Color Difference");
                    continue;
                }
            
            depthhypothesis* targetBest = otherDepthHypothesis +  newIDX;
            
            // large idepth = point is near = large increase in variance.
            // small idepth = point is far = small increase in variance.
            float idepth_ratio_4 = new_idepth / source->invDepthSmoothed;
            idepth_ratio_4 *= idepth_ratio_4;
            idepth_ratio_4 *= idepth_ratio_4;
            
            float new_var =idepth_ratio_4*source->invDepth;
            
            /*
            if((source->invDepth<0 ) ||(source->invDepthSmoothed<0 ))
            {    printf("\nx: %d, y: %d, source_idepth: %f, source_idepth_smooth: %f", x, y, source->invDepth,source->invDepthSmoothed);
                waitKey(0);
            }
            */
            
          
            
            
            // check for occlusion
            if(targetBest->isValid)
            {
                /*if(targetBest->invDepth<0 )
                {    printf("\nx: %d, y: %d, target_inv_depth: %f", x, y, targetBest->invDepth);
                    waitKey(0);
                }*/
                
                
                // if they occlude one another, one gets removed.
                float diff = targetBest->invDepth - new_idepth;
                if(util::DIFF_FAC_PROP_MERGE*diff*diff > new_var + targetBest->variance)
                {
                    if(new_idepth < targetBest->invDepth)
                    {
                        //if(enablePrintDebugInfo) runningStats.num_prop_occluded++;
                        //PRINTF("\nOccluded-1");
                        continue;
                    }
                    else
                    {
                       // if(enablePrintDebugInfo) runningStats.num_prop_occluded++;
                       // PRINTF("\nOccluded-2");
                        targetBest->isValid = false;
                    }
                }
            }
            
            if(!targetBest->isValid)
            {
                //if(enablePrintDebugInfo) runningStats.num_prop_created++;
                
                targetBest->invDepth=new_idepth;
                targetBest->variance=new_var;
                targetBest->varianceSmoothed=-1;
                targetBest->invDepthSmoothed=-1;
                targetBest->validity_counter=source->validity_counter;
                targetBest->isValid=true;
                targetBest->blacklisted=0;
               // PRINTF("\nFAIL");
//                if(new_var<0)
//                    printf("\nnew var: %f", new_var);
//                
//                if(new_idepth<0 )
//                {    printf("\nassigned inv_depth: %f", new_idepth);
//                    //waitKey(0);
//                }
            }
            
            
            else
            {
                //if(enablePrintDebugInfo) runningStats.num_prop_merged++;
                
                // merge idepth ekf-style
                float w = new_var / (targetBest->variance + new_var);
                float merged_new_idepth = w*targetBest->invDepth + (1.0f-w)*new_idepth;
                
                // merge validity
                int merged_validity = source->validity_counter + targetBest->validity_counter;
                if(merged_validity > util::VALIDITY_COUNTER_MAX+(util::VALIDITY_COUNTER_MAX_VARIABLE))
                    merged_validity = util::VALIDITY_COUNTER_MAX+(util::VALIDITY_COUNTER_MAX_VARIABLE);
                
                float temp_var=targetBest->variance; ////!!!!

                targetBest->invDepth=merged_new_idepth;
                targetBest->variance= 1.0f/(1.0f/temp_var + 1.0f/new_var);  //!!!!!
                targetBest->validity_counter=merged_validity;
               // targetBest->validity_counter=source->validity_counter;
                targetBest->isValid=true;
                targetBest->blacklisted=0;
                targetBest->invDepthSmoothed=-1.0f;
                targetBest->varianceSmoothed=-1.0f;
                //PRINTF("\nVALID");
               // *targetBest = DepthMapPixelHypothesis( merged_new_idepth,1.0f/(1.0f/targetBest->idepth_var + 1.0f/new_var),merged_validity);
//                 if(new_var<0 || temp_var<0)
//                     printf("\nnew var: %f, temp_var: %f", new_var, temp_var);
//                
//                if(new_idepth<0)
//                    printf("\nmerged idepth: %f", merged_new_idepth);
                
                
                /*if(targetBest->invDepth<0 || targetBest->variance<0)
                {    printf("\nx: %d, y: %d, already valid target_inv_depth: %f, var: %f", x, y, targetBest->invDepth, targetBest->variance);
                    waitKey(0);
                }
                */

            }
            
            
//            if(targetBest->invDepth<0 && targetBest->invDepth!=-1.0f)
//                printf("\nx: %d, y: %d, inv_depth: %f", x, y, targetBest->invDepth);
//             
            
            
        }
    }
    
    // swap!
    std::swap(currentDepthHypothesis, otherDepthHypothesis);
   

}


void depthMap::displayColourDepthMap(frame* image_frame, bool fromKeyFrameCreation)

{   static int count=0+util::BATCH_START_ID-1; //Need to check!!!!!!!!!!!!!!
    if(!fromKeyFrameCreation)
        count++;
    Mat colourMap(util::ORIG_ROWS,util::ORIG_COLS,CV_8UC1);
    float* depth_ptr= image_frame->depth.ptr<float>(0);
    uchar* color_ptr=colourMap.ptr<uchar>(0);
    
    // make mean inverse depth be one.
    /*
    float sumIdepth=0, numIdepth=0;
    for(int y=0;y<util::ORIG_ROWS;y++)
    {
        depth_ptr= image_frame->depth.ptr<float>(y);
        for(int x=0; x<util::ORIG_COLS;x++)
        {
            if(depth_ptr[x]==0)
                continue;
            sumIdepth += (1/depth_ptr[x]);
            numIdepth++;
        }
    }
    float rescaleFactor = numIdepth / sumIdepth;
     
    
    
    printf("\nRESCALE FAC: %f ", rescaleFactor);
     
    */

    //GREYSCALE
  
    
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        depth_ptr= image_frame->depth.ptr<float>(y);
        color_ptr=colourMap.ptr<uchar>(y);
        int j=0;
        for(int x=0; x<util::ORIG_COLS;x++)
        {
            
            color_ptr[j++]=uchar(depth_ptr[x]*100.f);//depth_ptr[x]
            
            //if(int(depth_ptr[x]*rescaleFactor*120.f)>color_ptr[j-1])
            //if((depth_ptr[x]>2.0f || depth_ptr[x]<0.5f) && depth_ptr[x]!=0)
            //    printf("\nscaled_depth: %f, color_depth: %d", depth_ptr[x]*rescaleFactor, color_ptr[j-1] );
            if(depth_ptr[x]*100.f > 255)
                color_ptr[j-1]=255;

    
    /*
    
    Mat colourMap(util::ORIG_ROWS,util::ORIG_COLS,CV_8UC3);
    float* depth_ptr= image_frame->depth.ptr<float>(0);
    uchar* color_ptr=colourMap.ptr<uchar>(0);
    
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        depth_ptr= image_frame->depth.ptr<float>(y);
        color_ptr=colourMap.ptr<uchar>(y);
        int j=0;
        for(int x=0; x<util::ORIG_COLS;x++)
        {
            if (depth_ptr[x]==0)
            {
                color_ptr[j++]=255;//depth_ptr[x]
                color_ptr[j++]=255;
                color_ptr[j++]=255;
                continue;
            }
    
     */
            
            /*if(depth_ptr[x]<0.70f) //blue
            {     color_ptr[j++]=255;
                color_ptr[j++]=0;
                color_ptr[j++]=0;
            }
            
            else if(depth_ptr[x]<0.95f) //purple
            {     color_ptr[j++]=255;
                color_ptr[j++]=102;
                color_ptr[j++]=178;
            }
            
            else if(depth_ptr[x]<1.1f) //light blue
            {     color_ptr[j++]=255;
                  color_ptr[j++]=178;
                  color_ptr[j++]=102;
            }
            else if(depth_ptr[x]<1.25f) //blue + green
            {     color_ptr[j++]=255;
                color_ptr[j++]=255;
                color_ptr[j++]=0;
            }
            else if(depth_ptr[x]<1.40f) //green
            {     color_ptr[j++]=0;
                color_ptr[j++]=255;
                color_ptr[j++]=0;
            }
            else if(depth_ptr[x]<1.55) //green+ red
            {     color_ptr[j++]=0;
                  color_ptr[j++]=255;
                  color_ptr[j++]=255;
            }
            else if (depth_ptr[x]<1.7f) //orange
            {     color_ptr[j++]=0;
                color_ptr[j++]=128;
                color_ptr[j++]=255;
            }

            else if(depth_ptr[x]<1.85f)
            {     color_ptr[j++]=0;
                  color_ptr[j++]=0;
                  color_ptr[j++]=255;
            }
            
            else // brown
            {
                color_ptr[j++]=0;
                color_ptr[j++]=51;
                color_ptr[j++]=102;
            }*/
            
        /*
            if(depth_ptr[x]<0.85f) //blue
             {     color_ptr[j++]=255;
             color_ptr[j++]=0;
             color_ptr[j++]=0;
             }
             
             else if(depth_ptr[x]<1.5f) //green
             {     color_ptr[j++]=0;
             color_ptr[j++]=255;
             color_ptr[j++]=0;
             }
             
             else  //light blue
             {
            color_ptr[j++]=0;
             color_ptr[j++]=0;
             color_ptr[j++]=255;
             }
            
       */

            
         /*
            
            uchar temp=uchar (depth_ptr[x]* util::IDEPTH_MULIPLIER);
            color_ptr[j++]=255-temp;//depth_ptr[x]
            color_ptr[j++]=temp<125 ? 2*temp:255-temp;
            color_ptr[j++]=0+temp;
          */
            

        }
    }
    
    Mat colourMap_rgb;
    applyColorMap(colourMap, colourMap_rgb, COLORMAP_JET);
    
    uchar* colour_rgb=colourMap_rgb.ptr<uchar>(0);
    uchar* image_ptr=keyFrame->image.ptr<uchar>(0);
    uchar r,g,b;
    int j;
    
    for(int y=0;y<util::ORIG_ROWS;y++)
    {
        colour_rgb=colourMap_rgb.ptr<uchar>(y);
        image_ptr=keyFrame->image.ptr<uchar>(y);
        j=0;
        
        for(int x=0;x<3*util::ORIG_COLS;x=x+3)
        {
            b=colour_rgb[x];
            g=colour_rgb[x+1];
            r=colour_rgb[x+2];
            
            //cout<<"\n"<<int(b)<<" "<<int(g)<<" "<<int(r);
            
            if(b==128 & g==0 & r==0)
            {
                // colour_rgb[x]=0;
                colour_rgb[x]=image_ptr[j];
                colour_rgb[x+1]=image_ptr[j];
                colour_rgb[x+2]=image_ptr[j];
                
            }
            j++;
        }
    }
    
    
    
    

    
    /*
    uchar* colorrgb_ptr=colourMap_rgb.ptr<uchar>(0);
    uchar* img_ptr= image_frame->image.ptr<uchar>(0);
    
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        img_ptr= image_frame->image.ptr<uchar>(y);
        colorrgb_ptr=colourMap_rgb.ptr<uchar>(y);
        int j=0;
        for(int x=0; x<3*util::ORIG_COLS;x=x+3)
        {
            if (colorrgb_ptr[x]==255 && colorrgb_ptr[x+1]==0 && colorrgb_ptr[x+2]==0)
            {
                color_ptr[j++]=img_ptr[x];//depth_ptr[x]
                color_ptr[j++]=0;
                color_ptr[j++]=0;
                continue;
            }
        }
    }
*/

    if(util::FLAG_DISPLAY_DEPTH_MAP)
    {
    
    imshow("Colour_depth",colourMap_rgb);
    waitKey(1000);
    }
    
  //  cout<<"\n\ncounter: "<<count;
    
    if(util::FLAG_SAVE_DEPTH_MAP)
    {
        
        
    //save image
    if(!fromKeyFrameCreation)
    {
    
     stringstream ss;
     string str = "depthmap_";
     string type = ".jpg";
     
     ss<<save_depthmap_path<<str<<(count)<<type;
     string filename = ss.str();
     ss.str("");
     
     imwrite(filename, colourMap_rgb);
    }
    
    }
    
     
    
}



void depthMap::updateDepthImage(bool fromKeyFrameCreation)
{
    //printf("Updating depth..");
    //find rescale factor:
    float sumIdepth=0, numIdepth=0;
    for(depthhypothesis* source = currentDepthHypothesis; source < currentDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS; source++)
    {
        if(!source->isValid)
            continue;
        sumIdepth += source->invDepthSmoothed;
        numIdepth++;
    }
    float rescaleFactor = numIdepth / sumIdepth;

   // cout<<"\nRESCALE FACTOR FOR KF id "<<keyFrame->frameId<<"is: "<<rescaleFactor;

    
    

    depthhypothesis* pt = currentDepthHypothesis;
    float* depth_ptr= keyFrame->depth.ptr<float>(0);
    float* deptharr=deptharrpyr0;
    float* vararr=depthvararrpyr0;
    
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        depth_ptr= keyFrame->depth.ptr<float>(y);
        
        for(int x=0; x<util::ORIG_COLS; x++)
        {
            if(y<3||y>=util::ORIG_ROWS-3||x<3||x>=util::ORIG_COLS-3)
            {
                pt->isValid=false;
            }
            
            
            
            depthhypothesis* source = currentDepthHypothesis + x + y*util::ORIG_COLS;
            //if(source->isValid && ((source->invDepth<0) || (source->invDepthSmoothed<0)))
            {  //  printf("\n col X: %d row Y: %d inv depth: %f, inv depth smooth= %f", x,y,source->invDepth , source->invDepthSmoothed);
                //waitKey(0);
            }
            
            
            
            if(pt->isValid==true && (pt->invDepthSmoothed>=-0.05f) )
            {
                depth_ptr[x]=(1/(pt->invDepthSmoothed));
                *deptharr=(1/(pt->invDepthSmoothed));
                *vararr=pt->varianceSmoothed;
                
//                
//                if(pt->varianceSmoothed<0)
//                {
//                    printf("\nx: %d, y: %d, varianceSmoothed: %f ", x, y, pt->varianceSmoothed);
//                   // waitKey(0);
//                }
                
                //cout<<"disp updated target depth:"<<pt->invDepth<<endl;//int(depth_ptr[x])<<endl;}
                
                //printf("%f , ", pt->invDepthSmoothed);
            }
            else{
                 depth_ptr[x]=0.0f;
                 *deptharr=-1.0f;
                 *vararr=-1.0f;
            
            }
            //if(currentFrame->frameId>98)
            /*{
                if(x==188 && y==32)
                    printf("\npt->varianceSmoothed: %f", pt->varianceSmoothed);
            }*/
            
            if( pt < currentDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS)
            {    pt++;
                vararr++;
                deptharr++;
            }
            else
                break;
        }
        
    }
    
    
    
   // cout<<"\nDEPTH : "<<keyFrame->depth;
    buildInvVarDepth();
    mapDepthArr2Mat();
    displayColourDepthMap(keyFrame, fromKeyFrameCreation);
    //keyFrame->constructDepthPyramids();
    
    
}

void depthMap::fillDepthHoles(){
    
    buildValIntegralBuffer();
    
    // memory copy!
    memcpy(otherDepthHypothesis, currentDepthHypothesis, util::ORIG_COLS*util::ORIG_ROWS*sizeof(depthhypothesis));
    
    // =========== regularize fill holes
    
    float* keyFrameMaxGradBuf = keyFrame->maxAbsGradient.ptr<float>(0);
    
    for(int y=util::YMIN; y<util::YMAX; y++)
        
    {
        
        keyFrameMaxGradBuf=keyFrame->maxAbsGradient.ptr<float>(y);
        
        for(int x=3;x<util::ORIG_COLS-2;x++)
            
        {
            
            int idx = x+y*util::ORIG_COLS;
            
            depthhypothesis* dest = otherDepthHypothesis + idx;
            
            if(dest->isValid) continue;
            
            if(keyFrameMaxGradBuf[x]<util::MIN_ABS_GRAD_DECREASE) continue;
            
            
            
            int* io = validityIntegralBuffer + idx;
            
            //cout<<"\nvalidity value "<<*io;
            
            int val = io[2+2*util::ORIG_COLS] - io[2-3*util::ORIG_COLS] - io[-3+2*util::ORIG_COLS] + io[-3-3*util::ORIG_COLS];
            
            
            if((dest->blacklisted >= util::MIN_BLACKLIST && val > util::VAL_SUM_MIN_FOR_CREATE) || val > util::VAL_SUM_MIN_FOR_UNBLACKLIST)
                
            {
                float sumIdepthObs = 0, sumIVarObs = 0;
                
                int num = 0;
                
                depthhypothesis* s1max = otherDepthHypothesis + (x-2) + (y+3)*util::ORIG_COLS;
                
                for (depthhypothesis* s1 = otherDepthHypothesis + (x-2) + (y-2)*util::ORIG_COLS; s1 < s1max; s1+=util::ORIG_COLS)
                    
                    for(depthhypothesis* source = s1; source < s1+5; source++)
                        
                    {
                        
                        if(!source->isValid) continue;
                        
                        sumIdepthObs += source->invDepth /source->variance;
                        
                        sumIVarObs += 1.0f/source->variance;
                        
                        num++;
                        
                    }
                
                float idepthObs = sumIdepthObs / sumIVarObs;
                
                idepthObs = UNZERO(idepthObs);
                
                
                currentDepthHypothesis[idx].invDepth=idepthObs;
                
                currentDepthHypothesis[idx].variance=util::VAR_RANDOM_INIT_INITIAL;
                
                currentDepthHypothesis[idx].validity_counter=0;
                
                currentDepthHypothesis[idx].isValid=true;
                
                currentDepthHypothesis[idx].blacklisted=0;
                
                currentDepthHypothesis[idx].invDepthSmoothed=-1;
                currentDepthHypothesis[idx].varianceSmoothed=-1;
            }
        }
    }
    
//    
//    for(int y=0;y<util::ORIG_ROWS;y++)
//        
//    {
//        for(int x=0;x<util::ORIG_COLS;x++)
//            
//        {
//            
//            depthhypothesis* dest = currentDepthHypothesis + x + y*util::ORIG_COLS;
//            if(dest->varianceSmoothed<0 && dest->isValid && (dest->invDepthSmoothed>=-0.05f))
//                cout<<"\nbye! "<<dest->varianceSmoothed;
//        }
//    }
}


void depthMap::buildValIntegralBuffer()

{
    
    // ============ build inegral buffers
    
    int* validityIntegralBufferPT = validityIntegralBuffer+3*util::ORIG_COLS;
    
    depthhypothesis* ptSrc = currentDepthHypothesis+3*util::ORIG_COLS;
    
    for(int y=util::YMIN;y<util::YMAX;y++)
        
    {
        
        int validityIntegralBufferSUM = 0;
        for(int x=0;x<util::ORIG_COLS;x++)
            
        {
            
            if(ptSrc->isValid)
                validityIntegralBufferSUM += ptSrc->validity_counter;// sum of validity counter of values lying left to the current pixel.
            //cout<<"\n row: "<<y<<" col: "<<x<<" validity counter"<<ptSrc->validity_counter;
            *(validityIntegralBufferPT++) = validityIntegralBufferSUM;
            ptSrc++;
            
        }
        
    }
    
    
    /*validityIntegralBufferPT = validityIntegralBuffer;
    int* validityIntegralBufferPT_T = validityIntegralBuffer+util::ORIG_COLS;
    
    int wh = util::ORIG_COLS*util::ORIG_ROWS;
    for(int idx=util::ORIG_COLS;idx<wh;idx++)
        *(validityIntegralBufferPT_T++) += *(validityIntegralBufferPT++);*/
    
    
    
}



void depthMap::regularizeDepthMap(bool removeOcclusions) //set as default to FALSE
{
    memcpy(otherDepthHypothesis, currentDepthHypothesis, util::ORIG_COLS*util::ORIG_ROWS*sizeof(depthhypothesis));
    
    int validityTH = util::VAL_SUM_MIN_FOR_KEEP;// To be decided later
    
    const int regularize_radius = 2;
    
    const float regDistVar = util::REG_DIST_VAR;
    
    for(int y=util::YMIN;y<util::YMAX;y++)
        
    {
        for(int x=regularize_radius;x<util::ORIG_COLS-regularize_radius;x++)
            
        {
            
            depthhypothesis* dest = currentDepthHypothesis + x + y*util::ORIG_COLS;
            
            depthhypothesis* destRead = otherDepthHypothesis + x + y*util::ORIG_COLS;
            
            if(!destRead->isValid)
                continue;
            
            float sum=0, val_sum=0, sumIvar=0;//, min_varObs = 1e20;
            
            int numOccluding = 0, numNotOccluding = 0;
            
            for(int dx=-regularize_radius; dx<=regularize_radius;dx++)
                
                for(int dy=-regularize_radius; dy<=regularize_radius;dy++)
                    
                {
                    
                    depthhypothesis* source = destRead + dx + dy*util::ORIG_COLS;
                    
                    if(!source->isValid) continue;
                    
                    float diff =source->invDepth - destRead->invDepth;
                    
                    if(util::DIFF_FAC_SMOOTHING*diff*diff > source->variance + destRead->variance)
                        
                    {
                        if(removeOcclusions)
                            
                        {
                            if(source->invDepth > destRead->invDepth)
                                numOccluding++;
                        }
                        
                        continue;
                    }
                    
                    val_sum += source->validity_counter;
                    
                    if(removeOcclusions)
                        
                        numNotOccluding++;
                    
                    float distFac = (float)(dx*dx+dy*dy)*regDistVar;
                    
                    float ivar = 1.0f/(source->variance + distFac);
                    
                    sum += source->invDepth * ivar;
                    
                    sumIvar += ivar;
                    
                }
            
            if(val_sum < validityTH)
                
            {
                dest->isValid = false;
            
                dest->blacklisted--;
                continue;
                
            }
            
            if(removeOcclusions)
                
            {
                if(numOccluding > numNotOccluding)
                    
                {
                    
                    dest->isValid = false;
                    continue;
                    
                }
                
            }
            
            sum = sum / sumIvar;
            
            sum = UNZERO(sum);
            
            // update!
            
            dest->invDepthSmoothed = sum;
            
            dest->varianceSmoothed = 1.0f/sumIvar;
            
//            if(currentFrame->frameId==97 && sumIvar<0)
//                cout<<"\nsumIvar: "<<sumIvar;
            
            
            
        }
    }
    
//    for(int y=0;y<util::ORIG_ROWS;y++)
//        
//    {
//        for(int x=0;x<util::ORIG_COLS;x++)
//            
//        {
//            
//            depthhypothesis* dest = currentDepthHypothesis + x + y*util::ORIG_COLS;
//            if(dest->varianceSmoothed<0  && dest->isValid && (dest->invDepthSmoothed>=-0.05f))
//                cout<<"\nhi! "<<dest->varianceSmoothed;
//        }
//    }
    
    
    
}


void depthMap::makeInvDepthOne(bool calculate_on_current)
{
  
    
   if(calculate_on_current)
   {
    //printf("\nMaking inverse depth one for Keyframe: %d", keyFrame->frameId);

    // make mean inverse depth be one.
    float sumIdepth=0, numIdepth=0;
    for(depthhypothesis* source = currentDepthHypothesis; source < currentDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS; source++)
    {
        if(!source->isValid)
            continue;
        sumIdepth += source->invDepthSmoothed;
        numIdepth++;
    }
    float rescaleFactor = numIdepth / sumIdepth; //rescale factor = 1/(mean of inv depth)
    depthScale=rescaleFactor;
       
       keyFrame->rescaleFactor=rescaleFactor;
    
    //adding recale factor to global scale
    util::GLOABL_DEPTH_SCALE*=depthScale;
    //printf("\nAdded new scale: %f of keyframe id: %d. Now Global Scale : %f", depthScale, keyFrame->frameId, util::GLOABL_DEPTH_SCALE);

    
    float rescaleFactor2 = rescaleFactor*rescaleFactor;
    
    for(depthhypothesis* source = currentDepthHypothesis; source < currentDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS; source++)
    {
        if(!source->isValid)
            continue;
        source->invDepth *= rescaleFactor;
        source->invDepthSmoothed *= rescaleFactor;
        source->variance *= rescaleFactor2;
        source->varianceSmoothed *= rescaleFactor2;
        /*if(source->varianceSmoothed < 0)
        {
            printf("\nHelloo ");
            printf("\nrescalefactor2: %f", rescaleFactor);
            printf("\nsource->varianceSmoothed: %f", source->varianceSmoothed);
            waitKey(0);
        }
         */
    }
    //update SE3 into Sim3
   // keyFrame->calculateSim3poseOtherWrtThis(rescaleFactor);
   // cout<<"\nRESCALE FACTOR FOR KF id "<<keyFrame->frameId<<"is: "<<rescaleFactor;
    
    //printf("\nExiting ....");
   // printf("\nDepth rescale factor: %f", depthScale);
   }
    
    
    
    
    else
    {
        //printf("\nMaking inverse depth one for Keyframe: %d", keyFrame->frameId);
        
        // make mean inverse depth be one.
        float sumIdepth=0, numIdepth=0;
        for(depthhypothesis* source = otherDepthHypothesis; source < otherDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS; source++)
        {
            if(!source->isValid)
                continue;
            sumIdepth += source->invDepthSmoothed;
            numIdepth++;
        }
        float rescaleFactor = numIdepth / sumIdepth; //rescale factor = 1/(mean of inv depth)
        
        printf("\nRESCALE FACTOR: %f", rescaleFactor);
        
        //depthScale=rescaleFactor;
        //adding recale factor to global scale
        //util::GLOABL_DEPTH_SCALE*=depthScale;
        //printf("\nAdded new scale: %f of keyframe id: %d. Now Global Scale : %f", depthScale, keyFrame->frameId, util::GLOABL_DEPTH_SCALE);
        
        
        float rescaleFactor2 = rescaleFactor*rescaleFactor;
        
        for(depthhypothesis* source = otherDepthHypothesis; source < otherDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS; source++)
        {
            if(!source->isValid)
                continue;
            source->invDepth *= rescaleFactor;
            source->invDepthSmoothed *= rescaleFactor;
            source->variance *= rescaleFactor2;
            source->varianceSmoothed *= rescaleFactor2;
            /*if(source->varianceSmoothed < 0)
             {
             printf("\nHelloo ");
             printf("\nrescalefactor2: %f", rescaleFactor);
             printf("\nsource->varianceSmoothed: %f", source->varianceSmoothed);
             waitKey(0);
             }
             */
        }
        //update SE3 into Sim3
        // keyFrame->calculateSim3poseOtherWrtThis(rescaleFactor);
        // cout<<"\nRESCALE FACTOR FOR KF id "<<keyFrame->frameId<<"is: "<<rescaleFactor;
        
        //printf("\nExiting ....");
        // printf("\nDepth rescale factor: %f", depthScale);
    }
    
}

void depthMap::doRegularization(bool removeOcclusions )
{
    //printf("\nDoing Regularization");
    //printf("\nIN FILL DEPTH");
    fillDepthHoles();
    //printf("\nIn regularize depth map");
    regularizeDepthMap(removeOcclusions);
    
}

void depthMap::buildInvVarDepth()
{
    PRINTF("\nBuilding Inverse depth and variance arrays");

 for (int i=1;i<util::MAX_PYRAMID_LEVEL;i++)
 {
     int width=util::ORIG_COLS>>i;
     int height=util::ORIG_ROWS>>i;
     int sw=2*width;
     float* depthVarSource=depthvararrptr[i-1];
     float* depthVarDest=depthvararrptr[i];
     float* depthSource=deptharrptr[i-1];
     float* depthDest=deptharrptr[i];
     
        for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            int idx = 2*(x+y*sw);
            int idxDest = (x+y*width);
            
            float idepthSumsSum = 0;
            float ivarSumsSum = 0;
            int num=0;
            

            
            // build sums
            float ivar;
            float var = depthVarSource[idx];
            if(var > 0)
            {
                ivar = 1.0f / var;
                ivarSumsSum += ivar;
                idepthSumsSum += ivar * 1.0f/depthSource[idx];
                num++;
            }
            
            var = depthVarSource[idx+1];
            if(var > 0)
            {
                ivar = 1.0f / var;
                ivarSumsSum += ivar;
                idepthSumsSum += ivar * 1.0f/depthSource[idx+1];
                num++;
            }
            
            var = depthVarSource[idx+sw];
            if(var > 0)
            {
                ivar = 1.0f / var;
                ivarSumsSum += ivar;
                idepthSumsSum += ivar * 1.0f/depthSource[idx+sw];
                num++;
            }
            
            var = depthVarSource[idx+sw+1];
            if(var > 0)
            {
                ivar = 1.0f / var;
                ivarSumsSum += ivar;
                idepthSumsSum += ivar * 1.0f/depthSource[idx+sw+1];
                num++;
            }
            
            if(num > 0)
            {
                
                float depth = ivarSumsSum / idepthSumsSum;
                depthDest[idxDest] = depth;
                depthVarDest[idxDest] = num / ivarSumsSum;
            }
            else
            {
                depthDest[idxDest] = 0.0f;
                depthVarDest[idxDest] = -1.0f;
                
            }
            if(x==188 && y==32)
            {
            //if(depthVarDest[idxDest]<0 && depthVarDest[idxDest]!=-1.0f)
            if(currentFrame->frameId>98)
            {   // printf("\n\ndepthVarDest : %f, x: %d, y: %d, width: %d", depthVarDest[idxDest], x, y, width);
                //printf("\ndepthvararrptr[i]+x+width*y: %f", *(depthvararrptr[i]+x+width*y));
                //printf("\ndepthvararrptr[i-1]+x+width*y: %f", *(depthvararrptr[i-1]+x+sw*y));
            }
            }
            
            if(x==188 && y==32)
            {
             //   printf("\ndepthvararrptr[i-1]+x+width*y: %f", *(depthvararrptr[i-1]+x+sw*y));
            }
        }
    
     }
    
   }
}


void depthMap::mapDepthArr2Mat()
{
    PRINTF("\nMapping Depth Array to Matrix");
    for(int i=0;i<util::MAX_PYRAMID_LEVEL;i++){
        if(i==0){
            keyFrame->depth_pyramid[0]=keyFrame->depth;
            continue;
        }
        int width=util::ORIG_COLS>>i;
        int height=util::ORIG_ROWS>>i;
        float* depthDest=deptharrptr[i];
        float* depthVarDest=depthvararrptr[i];
    
        float* depth_ptr=keyFrame->depth_pyramid[i].ptr<float>(0);
        for(int y=0;y<height;y++)
        {
            depth_ptr=keyFrame->depth_pyramid[i].ptr<float>(y);
            for(int x=0;x<width;x++)
            {
                depth_ptr[x]=*(depthDest+x+y*width);
                
            }
    
        }
        //cout<<"\ndepth"<<keyFrame->depth_pyramid[i];
    }
}


void depthMap::finaliseKeyframe()
{
    PRINTF("\nFinalising Key Frame: %d", keyFrame->frameId);
    doRegularization();
    //printf("\nIn finalise keyframe");
    updateDepthImage();
    
}


void depthMap::createKeyFrame(frame* new_keyframe)
{
    //printf("\nCreating Key Frame with new keyFrame Id: %d ", new_keyframe->frameId);
    //printf("\nIn create key frame");
    
    new_keyframe->calculateSE3poseOtherWrtThis(keyFrame);
    Eigen::MatrixXf oldToNew_SE3 = new_keyframe->SE3poseThisWrtOther;  //new KF Wrt old KF
    

    //updateDepthImage();
    
    //printf("\nIn propagate");
    propagateDepth(new_keyframe);
    
    /*
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        
        for(int x=0; x<util::ORIG_COLS; x++)
        {
            
            depthhypothesis* source = currentDepthHypothesis + x + y*util::ORIG_COLS;
            if(source->isValid && ((source->invDepth<0) || (source->invDepthSmoothed<0)))
            {    printf("\nafter propagate : inv depth: %f, inv depth smooth= %f", source->invDepth , source->invDepthSmoothed);
                //waitKey(0);
            }

        }
    }
    */
    
    
    //updateDepthImage();
    
    //changing active KF in depth map
    keyFrame=new_keyframe;
    keyFrame->isKeyframe=true;
    
    
    
     //printf("\nIn regularize depth map");
    regularizeDepthMap(true);
    
    
    /*
    //updateDepthImage();
    
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        
        for(int x=0; x<util::ORIG_COLS; x++)
        {
            
            depthhypothesis* source = currentDepthHypothesis + x + y*util::ORIG_COLS;
            if(source->isValid && ((source->invDepth<0) || (source->invDepthSmoothed<0)))
            {    printf("\ninv depth: %f, inv depth smooth= %f, x= %d, y=%d", source->invDepth , source->invDepthSmoothed, x, y);
                //waitKey(0);
            }

        }
    }
    */
    
    
    
    //printf("\nIn doregularization");
    doRegularization(false); //Do occlusions = false
    //updateDepthImage();
    
    
   /* for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        
        for(int x=0; x<util::ORIG_COLS; x++)
        {
            
            depthhypothesis* source = currentDepthHypothesis + x + y*util::ORIG_COLS;
            if(source->isValid && ((source->invDepth<0) || (source->invDepthSmoothed<0)))
            {    printf("\nAFTER REG : inv depth: %f, inv depth smooth= %f", source->invDepth , source->invDepthSmoothed);
                //waitKey(0);
            }

        }
    }
    */
    
    
    
   // printf("\nIn make inverse one");
    makeInvDepthOne();
    
    /*
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        
        for(int x=0; x<util::ORIG_COLS; x++)
        {
            
            depthhypothesis* source = currentDepthHypothesis + x + y*util::ORIG_COLS;
            if(source->isValid && ((source->invDepth<0) || (source->invDepthSmoothed<0)))
            {    printf("\ninv depth: %f, inv depth smooth= %f", source->invDepth , source->invDepthSmoothed);
                //waitKey(0);
            }

        }
    }
    */
    
    

    updateDepthImage(true);
   // printf("\nAfter update depth");
    
    /*
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        
        for(int x=0; x<util::ORIG_COLS; x++)
        {
            
            depthhypothesis* source = currentDepthHypothesis + x + y*util::ORIG_COLS;
            if(source->isValid && ((source->invDepth<0) || (source->invDepthSmoothed<0)))
            {    printf("\ninv depth: %f, inv depth smooth= %f", source->invDepth , source->invDepthSmoothed);
                //waitKey(0);
            }

        }
    }
     */
    
    /*
    
    keyFrame->poseWrtWorld[0]=keyFrame->poseWrtOrigin[0]; //unscaled pose
    keyFrame->poseWrtWorld[1]=keyFrame->poseWrtOrigin[0];
    keyFrame->poseWrtWorld[2]=keyFrame->poseWrtOrigin[0];
    keyFrame->poseWrtWorld[3]=keyFrame->poseWrtOrigin[0];
    keyFrame->poseWrtWorld[4]=keyFrame->poseWrtOrigin[0];
    keyFrame->poseWrtWorld[5]=keyFrame->poseWrtOrigin[0];
    */
    
    keyFrame->poseWrtOrigin[0]=0.0f; //making origin again for KeyFrame
    keyFrame->poseWrtOrigin[1]=0.0f;
    keyFrame->poseWrtOrigin[2]=0.0f;
    // keyFrame->poseWrtOrigin[3]=0.0f;
    // keyFrame->poseWrtOrigin[4]=0.0f;
    // keyFrame-> poseWrtOrigin[5]=0.0f;

    
    
    
}

void depthMap::updateKeyFrame() //to be called before loading reference frame for observe depth row
{
    PRINTF("\nUpdating Key Frame: %d for current frame: %d", keyFrame->frameId, currentFrame->frameId);
    currentFrame->calculateSE3poseOtherWrtThis(keyFrame);
    currentFrame->calculateSim3poseOtherWrtThis(keyFrame->rescaleFactor);
    
}

float depthMap::calculate_no_of_Seeds(bool calculate_on_current)
{
    float count=0;
    
    if(calculate_on_current)
    {
    for(int i=0;i<util::ORIG_COLS*util::ORIG_ROWS;i++)
    {
        depthhypothesis* idx=&currentDepthHypothesis[i];
        count+=float(idx->isValid);
    }
    }
    
    else
        
    {
        for(int i=0;i<util::ORIG_COLS*util::ORIG_ROWS;i++)
        {
            depthhypothesis* idx=&otherDepthHypothesis[i];
            count+=float(idx->isValid);
        }
    }
    
    return count/(util::ORIG_COLS*util::ORIG_ROWS)*100;
}


void depthMap::checkHighVariancePatch()
{
    Mat highVarianceDetectImg=Mat::zeros(util::ORIG_ROWS, util::ORIG_COLS, CV_32FC1);
    float* highvar_ptr=highVarianceDetectImg.ptr<float>(0);
    depthhypothesis* pt;
    depthhypothesis* index_pt;
    int patchsize=3;
    
    for(int y=3;y<util::ORIG_ROWS;y++)
    {
        highvar_ptr=highVarianceDetectImg.ptr<float>(y);
        for(int x=3;x<util::ORIG_COLS;x++)
        {
            pt=currentDepthHypothesis+x+util::ORIG_COLS*y;
            if(!pt->isValid)
                continue;
            
            //calculate standard deviation about a patch of 5X5 with mean as centre
            float stdDev=0.0f;
            float mean_inv_smooth=0.0f;
            float mean_inv=0.0f;
            int num_valid=0;
            
            for(int y1=-patchsize;y1<=patchsize;y1++)
            {
                for(int x1=-patchsize;x1<=patchsize;x1++)
                {
                    index_pt=pt+x1+util::ORIG_COLS*y1;
                    if(!index_pt->isValid)
                        continue;
                    mean_inv_smooth+=index_pt->invDepthSmoothed;
                    mean_inv+=index_pt->invDepth;
                    num_valid++;
                }
            }
            mean_inv_smooth/=num_valid;
            mean_inv/=num_valid;
            
            //printf("\nAt (x= %d, y= %d), mean= %f", x,y,pt->invDepthSmoothed);
            
            for(int y1=-patchsize;y1<=patchsize;y1++)
            {
                for(int x1=-patchsize;x1<=patchsize;x1++)
                {
                    index_pt=pt+x1+util::ORIG_COLS*y1;
                    //printf("\nAt (x1= %d, y1= %d), invdepth= %f", x1,y1,index_pt->invDepthSmoothed);
                    if(!index_pt->isValid)
                        continue;
                    
                    stdDev+=((index_pt->invDepthSmoothed-mean_inv_smooth)*(index_pt->invDepthSmoothed-mean_inv_smooth));
                    //printf("\nLoop stdDev= %f", stdDev);
                    //num_valid++;
                }
            }
            stdDev/=num_valid; //patch of 25 points
            //printf("\nAfter divide by numvalid= %f", stdDev);
            stdDev=sqrt(stdDev);
            //printf("\nAt (x= %d, y= %d) , After sqrt final StdDev = %f", x, y, stdDev );
            
            
            
            if(stdDev>0.15f)
            {
                highvar_ptr[x]=1;
                //printf("\nModifying!");
                
                for(int y1=-patchsize;y1<=patchsize;y1++)
                {
                    for(int x1=-patchsize;x1<=patchsize;x1++)
                    {
                        index_pt=pt+x1+util::ORIG_COLS*y1;
                        if(!index_pt->isValid)
                            continue;
                        index_pt->invDepthSmoothed=mean_inv_smooth;
                        index_pt->invDepth=mean_inv;
                    }
                }
                
            }
            
            //waitKey(0);
        }
    }
    //waitKey(0);
    
    Mat colourMap_rgb;
    applyColorMap(highVarianceDetectImg, colourMap_rgb, COLORMAP_JET);
    imshow("High Variance Detect", colourMap_rgb);
}

void depthMap::scaleDepthMap()
{
    //printf("\nIn ScaleDepthMap for kf id: %d with Gloabl scale: %f", keyFrame->frameId, util::GLOABL_DEPTH_SCALE);
    float rescaleFactor=util::GLOABL_DEPTH_SCALE;
    float rescaleFactor2=util::GLOABL_DEPTH_SCALE*util::GLOABL_DEPTH_SCALE;
    
    for(depthhypothesis* source = currentDepthHypothesis; source < currentDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS; source++)
    {
        if(!source->isValid)
            continue;
        source->invDepth /= rescaleFactor;
        source->invDepthSmoothed /= rescaleFactor;
        source->variance /= rescaleFactor2;
        source->varianceSmoothed /= rescaleFactor2;

    }
    
    updateDepthImage();
}

void depthMap::observeDepthRowParallel()
{
    
    currentFrame->calculateSE3poseOtherWrtThis(keyFrame);
    
    util::measureTime time_depth;
    time_depth.startTimeMeasure();
    
    //Compute
    int y_increment=util::ORIG_ROWS/util::NUM_DEPTH_THREADS;
    //printf("\nDepth: Orig_rows: %d, and y_increment: %d", util::ORIG_ROWS, y_increment);

    if(util::FLAG_DO_PARALLEL_DEPTH_ESTIMATION)
    {
        
        boost::thread_group group;
        
        group.create_thread(boost::bind(&depthMap::observeDepthRow,this, util::YMIN,util::YMIN+y_increment, 1));
        group.create_thread(boost::bind(&depthMap::observeDepthRow,this, util::YMIN+y_increment,util::YMIN+2*y_increment, 2));
        group.create_thread(boost::bind(&depthMap::observeDepthRow,this, util::YMIN+2*y_increment,util::YMAX, 3));
        
        group.join_all();
        
    }
    else
        
    {   observeDepthRow(util::YMIN, util::YMAX, 0);
        
    }
    
    float elapsed=time_depth.stopTimeMeasure();
    //cout <<"\nTime observe depth row parallel: "<<int(util::FLAG_DO_PARALLEL_DEPTH_ESTIMATION)<<"  "<<elapsed<<endl;
    
}









