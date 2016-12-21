//
//  PixelWisePyramid.cpp
//  odometry code 5
//
//  Created by Himani Arora on 18/10/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//

#include "PixelWisePyramid.h"

#include "UserDefinedFunc.h"
#include "ExternVariable.h"
#include "DisplayFunc.h"

using namespace cv;

//constructor
PixelWisePyramid::PixelWisePyramid(frame* prevframe,frame* currentframe,float* pose,depthMap* currDepthMap){
    
    currentDepthMap=currDepthMap;
    prev_frame=prevframe; //ref frame or keyframe
    current_frame=currentframe;
    saveImg= Mat(1, prev_frame->no_nonZeroDepthPts, CV_32FC1); //save intensity of prev image at non zero depth points for current pyramid level
    
    pyrlevel=prev_frame->pyrLevel;
    nRows=prev_frame->currentRows;
    nCols=prev_frame->currentCols;
    
    //forms matrices that can be displayed, not needed otherwise
    display_2bewarpedimg=Mat::zeros(nRows,nCols,CV_8UC1);
    display_templateimg=Mat::zeros(nRows,nCols,CV_8UC1); //template image is the fixed ref image or the prev image in this case
    display_origres=Mat::zeros(nRows,nCols,CV_32FC1);//the initial residual before warping
    display_iterationres=Mat::zeros(nRows,nCols,CV_32FC1); //residual bewteen warped iamge at each iteration
    display_warpedimg=Mat::zeros(nRows,nCols,CV_32FC1);
    display_weightimg=Mat::zeros(nRows, nCols, CV_32FC1); //shows the color-coded weight of each pixel
    
    //here matrices are of size rows*cols, where zero depth points are included but are masked
    savedWarpedPointsX=Mat::zeros(nRows, nCols, CV_32FC1);
    savedWarpedPointsY=Mat::zeros(nRows, nCols, CV_32FC1);
    steepestDescent=Mat::zeros(2, nRows*nCols, CV_32FC1); //6xN
    weightedSteepestDescent=Mat::zeros(2, nRows*nCols, CV_32FC1); //6*N
    
    //motion prior, not used
    //complete implementation may not be present
    if(util::FLAG_USE_MOTION_PRIOR)
    {
        covarianceDiagonalWts[0]=0.0001f; //w1
        covarianceDiagonalWts[1]=0.0001f;
        covarianceDiagonalWts[2]=0.0001f;
        covarianceDiagonalWts[3]=1000.0f; //v1
        covarianceDiagonalWts[4]=1000.0f;
        covarianceDiagonalWts[5]=1000.0f;
        calCovarianceMatrixInv(covarianceDiagonalWts);
    }

    
}

PixelWisePyramid::~PixelWisePyramid()
{
    //printf("\ndestructing...");
    /*
    hessianInv.release();
    printf("\nDone!");
    sd_param.release();
    printf("\nDone!");
    covarianceMatrixInv.release();
    printf("\nDone!");
    //warpedGradientx.release();
    //printf("\nDone!");
    //warpedGradienty.release();
    //printf("\nDone!");
    saveImg.release();
    printf("\nDone!");
    motionPrior.release();
    printf("\nDone!");
    deltapose.release();
    printf("\nDone!");
    sd_param.release();
    printf("\nDone!");
    */
}

//puts the initial pose
void PixelWisePyramid::putPreviousPose(frame* tminus1_prev_frame)
{
    tminus1_prev_frame->concatenateOriginPose(tminus1_prev_frame->poseWrtWorld, prev_frame->poseWrtWorld, prevPose);
}

//motion prior, not used
void PixelWisePyramid::calCovarianceMatrixInv(float* covar_wts)
{
    Mat covarianceMatrix=Mat::zeros(2, 2, CV_32FC1);
    
    float* covarmat_ptr=covarianceMatrix.ptr<float>(0);
    for(int y=0; y<2; y++)
    {
        covarmat_ptr=covarianceMatrix.ptr<float>(y);
        covarmat_ptr[y]=covar_wts[y];
    }
    
    covarianceMatrixInv=covarianceMatrix.inv();
    
    //cout<<"\n\nCovar Mat Inv: \n "<<covarianceMatrixInv;
}

//motion prior, not used
void PixelWisePyramid::calMotionPrior()
{
    //printf("\nIn motion prior");
    Mat diffPose=Mat::zeros(1, 2, CV_32FC1);
    float* diffpose_ptr=diffPose.ptr<float>(0);
    
    for(int i=0; i<2; i++)
    {
        diffpose_ptr[i]=prevPose[i]-pose[i];
        //printf("\nprev: %f, current: %f, diff: %f", prevPose[i], pose[i], diffpose_ptr[i]);
    }
    motionPrior=Mat::zeros(1, 2, CV_32FC1);
    
    for(int i=0; i<prev_frame->no_nonZeroDepthPts; i++)
        motionPrior+=(covarianceMatrixInv*(diffPose.t())).t(); //1x6
    
    //cout<<"\n\nMotion Prior: \n "<<motionPrior;
}

//this function estimates the steepest descent and hessian matrix which is calculated for each pixel independently
//it takes in a ymin and ymax which correspond to the starting and ending row number in an image
//this allows multiple threads to break the image in sections and use only those sections
//it also takes thread_number as the input in order to use the variables associated with that thread only, thread_number = 0 means a single thread
void PixelWisePyramid::calculatePixelWise(int ymin,int ymax, int thread_num)
{
   // printf("\n\n\nIn calculate pixel-wise with ymin: %d, ymax: %d, thread_num: %d", ymin,ymax,thread_num);
    //cout<<"\npixel wise!! rows: "<<nRows<<" col: "<<nCols;

    int i;
    int x,y;
    int idx;
    
    //*******CALCULATE RESIZED INTRINSIC PARAMETERS*******//
    
    vector<float> resized_intrinsic= GetIntrinsic(prev_frame->pyrLevel);
    float resized_fx=resized_intrinsic[0];
    float resized_fy=resized_intrinsic[1];
    float resized_cx=resized_intrinsic[2];
    float resized_cy=resized_intrinsic[3];
    
    //*******INITIALIZE COUNTERS*******//
    
    int saveimg_row=0; //counter to store in ro0 of saveimg
    
    
    //*******PIXEL VARIABLES*******//
    
    float worldpointX;
    float worldpointY;
    float worldpointZ;
    
    float trfm_worldpointX;
    float trfm_worldpointY;
    float trfm_worldpointZ;
    
    float warpedpointx;
    float warpedpointy;
    
    float warpedintensity;
    
    float gradx;
    float grady;
    
    float jacob_bottom[2];
    float jacob_top[2];
    
    Mat steepestDescentMat(1,2,CV_32FC1);
    Mat weigthed_steepdesc_T;
    
    float residual;
    float res_weight;
    
    //initializations
    switch (thread_num)
    {
        case 0: //single thread, therefore uses the final variable
            sd_param=Mat::zeros(1, 2, CV_32FC1);
            hessian=Mat::zeros(2, 2, CV_32FC1);
            break;
        case 1: //uses variable associated with first thread
            sd_param_thread1 =Mat::zeros(1, 2, CV_32FC1);
            hessian_thread1=Mat::zeros(2, 2, CV_32FC1);
            break;
        case 2:
            sd_param_thread2=Mat::zeros(1, 2, CV_32FC1);
            hessian_thread2=Mat::zeros(2, 2, CV_32FC1);
            break;
        case 3:
            sd_param_thread3=Mat::zeros(1, 2, CV_32FC1);
            hessian_thread3=Mat::zeros(2, 2, CV_32FC1);
            break;
    }
    
    
    //*******INITIALIZE POINTERS*******//
    
    uchar* img_ptr;
    uchar* previmg_ptr;
    //float* saveimg_ptr;
    uchar* mask_ptr;
    float* depth_ptr;
    float* steepdesc_ptr;
    
    if(!util::FLAG_DO_PARALLEL_POSE_ESTIMATION)
    {
     //saveimg_ptr=saveImg.ptr<float>(0); //initialize to row 0 (single row Mat)
    }
    steepdesc_ptr=steepestDescentMat.ptr<float>(0);
    
    //*******DISPLAY POINTERS*******//
    uchar* disp_templateimg_ptr;
    uchar* disp_2bewarpedimg_ptr;
    float* disp_warpedimg_ptr;
    float* disp_iterationres_ptr;
    float* disp_origres_ptr;
    float* disp_weight_ptr;
    
    float* saved_warpedpts_ptrX;
    float* saved_warpedpts_ptrY;
    
    //*******STORE SE3 PARAMETERS*******//


    float c = cos(pose[1]);
    float s = sin(pose[1]);
    float r = pose[0];

    Mat SE3 = (Mat_<float>(4, 4) << c,0,-s,-(r*s),0,1,0,0,s,0,c,((r*c)-r),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    float* SE3_ptr;
    float SE3_vec[12];  //r11, r12, r13, t1, r21 r22, r23, t2, r31, r32, r33, t3
    int vec_counter=0;
    for(i=0; i<3; ++i)
    {
        SE3_ptr=SE3.ptr<float>(i); //get pointer to first row of SE3
        
        SE3_vec[vec_counter++]=SE3_ptr[0];
        SE3_vec[vec_counter++]=SE3_ptr[1];
        SE3_vec[vec_counter++]=SE3_ptr[2];
        SE3_vec[vec_counter++]=SE3_ptr[3];
        
    }
    
    //translation component of SE3 pose
    float tx=SE3_Eigen(0,3);
    float ty=SE3_Eigen(1,3);
    float tz=SE3_Eigen(2,3);
    
    //*******PIXEL WISE LOOP*******//
    
    for(y = ymin; y < ymax; y++) //must run from 0 to nrows when there is only single thread
    {
        //printf("\nHi! thread = %d\n\n", thread_num);
        
        //get pointer for current row i;
        img_ptr = current_frame->image_pyramid[pyrlevel].ptr<uchar>(y); //current image
        previmg_ptr=prev_frame->image_pyramid[pyrlevel].ptr<uchar>(y); //previmg or keyframe
        
        mask_ptr=prev_frame->mask.ptr<uchar>(y);
        depth_ptr = prev_frame->depth_pyramid[pyrlevel].ptr<float>(y);
        
        //taking pointers to the yth row
        disp_templateimg_ptr=display_templateimg.ptr<uchar>(y);
        disp_2bewarpedimg_ptr=display_2bewarpedimg.ptr<uchar>(y);
        disp_warpedimg_ptr=display_warpedimg.ptr<float>(y);
        disp_iterationres_ptr=display_iterationres.ptr<float>(y);
        disp_origres_ptr=display_origres.ptr<float>(y);
        disp_weight_ptr=display_weightimg.ptr<float>(y);
        saved_warpedpts_ptrX=savedWarpedPointsX.ptr<float>(y);
        saved_warpedpts_ptrY=savedWarpedPointsY.ptr<float>(y);
        
        
        for (x = 0; x < nCols; ++x)
            
        {   //printf("\nByee! thread = %d\n\n", thread_num);
            
            //check for zero depth point
            if(mask_ptr[x]==0)
            {
                //here the pixel is masked
                disp_templateimg_ptr[x]=0;
                disp_2bewarpedimg_ptr[x]=0;
                disp_warpedimg_ptr[x]=0;
                disp_iterationres_ptr[x]=0;
                disp_origres_ptr[x]=0;
                disp_weight_ptr[x]=0;
                saved_warpedpts_ptrX[x]=-2.0f; //no depth
                saved_warpedpts_ptrY[x]=-2.0f;
                continue;  //skip loop for zero depth points
            }
            
            disp_templateimg_ptr[x]=img_ptr[x];
            disp_2bewarpedimg_ptr[x]=previmg_ptr[x];
            disp_origres_ptr[x]=img_ptr[x]-previmg_ptr[x];
            //cout<<"\n"<<disp_origres_ptr[x];
            
            idx=x+nCols*y;
            
            //printf("\n\n\n(x, y) : (%d ,%d)", x,y);
            
            if(!util::FLAG_DO_PARALLEL_POSE_ESTIMATION)
            {
                //save image intensity at non zero depth point
                //saveimg_ptr[saveimg_row++]=previmg_ptr[x];
            }
                
            //world point  X ;Y ;depth in coordinate sys of prev img
            worldpointX=(x-resized_cx)*depth_ptr[x]/resized_fx; //X-coordinate
            worldpointY=(y-resized_cy)*depth_ptr[x]/resized_fy; //Y-coordinate
            worldpointZ=depth_ptr[x]; //depth of point at (u,v)
        
            //printf("\n(u, v)=(%d, %d) , world point Z: %f",x, y, worldpointZ);
            
            
            //calculating transformed world point and warped point coordinate(in current img)
            if (SE3_vec[1]==0) //if-else may not be needed here
            {
                trfm_worldpointX=((SE3_vec[0]*worldpointX)+(SE3_vec[1]*worldpointY)+(SE3_vec[2]*worldpointZ)+(SE3_vec[3]));
                trfm_worldpointY=((SE3_vec[4]*worldpointX)+(SE3_vec[5]*worldpointY)+(SE3_vec[6]*worldpointZ)+(SE3_vec[7]));
                trfm_worldpointZ=((SE3_vec[8]*worldpointX)+(SE3_vec[9]*worldpointY)+(SE3_vec[10]*worldpointZ)+(SE3_vec[11]));
                
               trfm_worldpointZ=UNZERO(trfm_worldpointZ); //to avoid zero depth
                
                warpedpointx=((trfm_worldpointX/trfm_worldpointZ)*resized_fx)+resized_cx;
                warpedpointy=((trfm_worldpointY/trfm_worldpointZ)*resized_fy)+resized_cy;
            }
            else
            {
                trfm_worldpointX=(float(SE3_vec[0]*worldpointX)+float(SE3_vec[1]*worldpointY)+float(SE3_vec[2]*worldpointZ)+float(SE3_vec[3]));
                trfm_worldpointY=(float((SE3_vec[4]*worldpointX))+float((SE3_vec[5]*worldpointY))+float((SE3_vec[6]*worldpointZ))+float((SE3_vec[7])));
                trfm_worldpointZ=(float(SE3_vec[8]*worldpointX)+float(SE3_vec[9]*worldpointY)+float(SE3_vec[10]*worldpointZ)+float(SE3_vec[11]));
                
                trfm_worldpointZ=UNZERO(trfm_worldpointZ);
                
                warpedpointx=float(((trfm_worldpointX/trfm_worldpointZ)*resized_fx)+resized_cx);
                warpedpointy=float(((trfm_worldpointY/trfm_worldpointZ)*resized_fy)+resized_cy);
                
            }
            
            //printf("\nthread_num: %d, (x, y) : (%d ,%d) , warped point : (%f, %f), world points: (%f, %f, %f)\n\n", thread_num, x,y, warpedpointx,warpedpointy, worldpointX,worldpointY,worldpointZ);
            
            //printf("\ntransformed world point: (%f,%f,%f)", trfm_worldpointX,trfm_worldpointY,trfm_worldpointZ);
            
            //calculate warped intensity-> this corresponds to the warped version of current image
            warpedintensity= current_frame->getInterpolatedElement(warpedpointx,warpedpointy, 1);
            
            if(warpedintensity==-1) //when the warped point lies outside image plane
            {
                disp_warpedimg_ptr[x]=0;
                saved_warpedpts_ptrX[x]=-1.0f;  //oob
                saved_warpedpts_ptrY[x]=-1.0f;
            }
            else
            {
                disp_warpedimg_ptr[x]=warpedintensity;
                saved_warpedpts_ptrX[x]=warpedpointx;
                saved_warpedpts_ptrY[x]=warpedpointy;
            }
        
            
            //printf("\nwarped point : (%f, %f) and intensity %f", warpedpointx,warpedpointy, warpedintensity);
            
            //calculating gradient of warped image
            //this can be thought of as gradient of current image at warped point
            gradx=current_frame->getInterpolatedElement(warpedpointx, warpedpointy, "gradx");//!!!!
            grady=current_frame->getInterpolatedElement(warpedpointx, warpedpointy, "grady");
            
            //calculating Jacobian
            //precomputed expression of jacobian*gradient
            jacob_bottom[0]  = grady*((resized_cy - y) * pow(depth_ptr[x],-1));
            jacob_top[0] = gradx*((resized_cx - x) * pow(depth_ptr[x],-1));

            jacob_bottom[1]  = grady*(-(resized_cy - y) * pow(depth_ptr[x],-1));
            jacob_top[1] = gradx*(-(resized_cx - x) * pow(depth_ptr[x],-1));
            
            jacob_bottom[2]=grady*(((-resized_cy + y)*(-resized_cx + x)) / resized_fx);
            jacob_top[2]= gradx*(- resized_fx - (pow((-resized_cx + x),2) / resized_fx));
            
            //calculate steepest descent
            steepdesc_ptr[0]=jacob_top[0]+jacob_bottom[0]; //gradient already multiplied
            steepdesc_ptr[1]=jacob_top[1]+jacob_bottom[1];
            
            //cout<<"\nidhar steepest descent: \n "<<steepestDescentMat;
            
            //calculate residual
            if(warpedintensity==-1)
            {
                residual=0.0f; //for oob pixel
            }
            else
                residual=warpedintensity-float(previmg_ptr[x]); //warped img-previmg
            
            disp_iterationres_ptr[x]=residual;
            
            if(warpedintensity==-1) //MAJOR CHANGE HERE!!!!
                res_weight=0;
            else
            {
                //calculate weights
                //these weights change with each iteration
                //are dependent on gradient, residual and depth variance at taht point
                float px = trfm_worldpointX;	// x'
                float py = trfm_worldpointY;	// y'
                float pz = trfm_worldpointZ;	// z'
                float d = 1.0f/depth_ptr[x];	// d
                float rp = residual; // r_p
                float gx = resized_fx * gradx;	// \delta_x I
                float gy = resized_fy * grady;  // \delta_y I
                float s = 1.0f * (*(currentDepthMap->depthvararrptr[prev_frame->pyrLevel]+idx));	// \sigma_d^2
                // calc dw/dd (first 2 components):
                float g0 = (tx * pz - tz * px) / (pz*pz*d);
                float g1 = (ty * pz - tz * py) / (pz*pz*d);
                // calc w_p
                float drpdd = gx * g0 + gy * g1;	// ommitting the minus
                float w_p = 1.0f / (util::CAMERA_PIXEL_NOISE_2 + s * drpdd * drpdd);
                float weighted_rp = fabs(rp*sqrtf(w_p));
                float wh = fabs(weighted_rp < (util::HUBER_D/2) ? 1 : (util::HUBER_D/2) / weighted_rp);
                //sumRes += wh * w_p * rp*rp;
                res_weight= wh * w_p; //final weight
                //printf("\nweight: %f\n",res_weight );
            }
            
            disp_weight_ptr[x]=res_weight;
            
            //printf("\nresidual: %f, weights: %f", residual, res_weight);

            /*printf("\n\nthread no: %d, Steepest descent: %f, %f, %f, %f, %f, %f",  thread_num, steepdesc_ptr[0], steepdesc_ptr[1], steepdesc_ptr[2],steepdesc_ptr[3],steepdesc_ptr[4],steepdesc_ptr[5]);
            
            cout<<"\n\n"<<steepestDescentMat<<"\n";*/
            
            //calculate weighted steepest descent
            weigthed_steepdesc_T= steepestDescentMat.t(); //3X1
            weigthed_steepdesc_T=weigthed_steepdesc_T.mul(res_weight);
            
            //cout<<"\nweighted steepest descent: \n"<<weigthed_steepdesc_T;
            
            //stores the final variables here which are needed after this for estimation of pose change
            //steepest descent and hessian are summed over all the pixel
            ofstream f1,f2,f3;

            switch (thread_num)
            {
                case 0:
                    hessian+=(weigthed_steepdesc_T*steepestDescentMat); //6X6
                    sd_param+=steepestDescentMat.mul(residual*res_weight); //steepest descent parameters, 1x6
                    
                    cout << "sd_param: " << sd_param << endl;
                    cout << "hessian: " << hessian << endl;

                    if(util::FLAG_USE_MOTION_PRIOR) //motion prior, not used
                        hessian+=covarianceMatrixInv;
                    
                    break;
                case 1:
                    hessian_thread1+=(weigthed_steepdesc_T*steepestDescentMat);
                    sd_param_thread1+=steepestDescentMat.mul(residual*res_weight);

                    f1.open("sd_param1");
                    f1 << "sd_param1: " << sd_param_thread1 << endl;
                    f1 << "hessian: " << hessian_thread1 << endl;
                    f1.close();
                    
                    if(util::FLAG_USE_MOTION_PRIOR)
                        hessian_thread1+=covarianceMatrixInv;
                    
                    break;
                case 2:
                    hessian_thread2+=(weigthed_steepdesc_T*steepestDescentMat);
                    sd_param_thread2+=steepestDescentMat.mul(residual*res_weight);

                    f2.open("sd_param2");
                    f2 << "sd_param2: " << sd_param_thread2 << endl;
                    f2 << "hessian: " << hessian_thread2 << endl;
                    f2.close();
                    
                    if(util::FLAG_USE_MOTION_PRIOR)
                        hessian_thread2+=covarianceMatrixInv;
                    
                    break;
                case 3:
                    hessian_thread3+=(weigthed_steepdesc_T*steepestDescentMat);
                    sd_param_thread3+=steepestDescentMat.mul(residual*res_weight);

                    f3.open("sd_param3");
                    f3 << "sd_param3: " << sd_param_thread3 << endl;
                    f3 << "hessian: " << hessian_thread3 << endl;
                    f3.close();
                    
                    if(util::FLAG_USE_MOTION_PRIOR)
                        hessian_thread3+=covarianceMatrixInv;
                    
                    break;
            }
            
            //cout<<"\nHessian: \n"<<hessian;
            //cout<<"\nsd param :\n"<<sd_param;
        }
        
    }
    
    
    /*
    DisplayMatImg(display_templateimg, "template_img");
    DisplayMatImg(display_2bewarpedimg, "2be_warped_img");
    DisplayMatImg(display_origres, "orig_res");
    DisplayMatImg(display_iterationres, "iteration_res");
    DisplayMatImg(display_warpedimg, "warped_img");
    */
    
    return;
    
    
}

//forms threads and calls actual function which will do all the computation
void PixelWisePyramid::calculatePixelWiseParallel()
{
    //struct timeval start, end;
    //gettimeofday(&start, NULL);
    
    //printf("\nIn calculate pixel-wise");
    
    if(util::FLAG_DO_PARALLEL_POSE_ESTIMATION) //if flag on to prallelize pose estimation
    {
    
        boost::thread_group t_group;
    
        int y_increment=nRows/util::NUM_POSE_THREADS; //defines the section size in an image that is given to each thread
        
        //printf("\n\n\nPose: pyrlevel: %d, nRows: %d, nCols: %d, y_increment: %d", pyrlevel, nRows, nCols, y_increment);
    
        //forms only 3 threads (hard-coded)
        //if more threads are to be added, need to be called and thread variables also have to be created in the class
        t_group.create_thread(boost::bind(&PixelWisePyramid::calculatePixelWise,this, 0,y_increment,1));
        t_group.create_thread(boost::bind(&PixelWisePyramid::calculatePixelWise,this, y_increment,2*y_increment,2));
        t_group.create_thread(boost::bind(&PixelWisePyramid::calculatePixelWise,this, 2*y_increment,nRows,3));
            
        t_group.join_all();
           
        //control reaches here when all threads have been completed and joined
        //forms the final hessian and sd_param matrices by summing each thread matrix
        //effectively sums over all the pixels
        cout << "hessian1:\n" ;
        cout << hessian_thread1 << endl;
        cout << "hessian2:\n" ;
        cout << hessian_thread2 << endl;
        cout << "hessian3:\n" ;
        cout << hessian_thread3 << endl;

        cout << "sd1:\n" ;
        cout << sd_param_thread1 << endl;
        cout << "sd2:\n" ;
        cout << sd_param_thread2 << endl;
        cout << "sd3:\n" ;
        cout << sd_param_thread3 << endl;

        hessian = hessian_thread1 + hessian_thread2 + hessian_thread3;
        sd_param = sd_param_thread1 + sd_param_thread2 + sd_param_thread3;

        cout << "hessian:\n";
        cout << hessian << endl;
        
        cout << "sd:\n";
        cout << sd_param << endl;
    }
    else
    {
        calculatePixelWise(0, nRows, 0); //when only single thread, function called over the entire iamge at once
    }

    cout << "det: " << determinant(hessian) << endl;
    //calculate hessian inverse
    hessianInv=hessian.inv();
    
    //cout<<"\nhessian inv: \n"<<hessianInv;
    //cout<<"\nsd param: \n"<<sd_param;
    updatePose(); //only single thread here

}



//calculates delta pose (pose change) and updates current estimate of pose
void PixelWisePyramid::updatePose()
{
    //printf("\nUpdating pose ");
        
    //Mat temp = residual.mul(weights);    // 1 x N

    float *ptr;
    int i,j;

    // cout << "Weighted residuals:\n";
    // ptr = temp.ptr<float>(0);
    // for (j = 0; j < prev_frame->no_nonZeroDepthPts; ++j)
    // {
    //     cout << ptr[j] << " ";
    // }
    // cout << "\n";

    cout << "steepest Descent:\n" << sd_param << endl;
    
    cout << "hessianInv:\n" << hessianInv << endl;

    // for (i = 0; i < 3; ++i)
    // {
    //     ptr = hessianInv.ptr<float>(i);
    //     for (j = 0; j < 3; ++j)
    //     {
    //         cout << ptr[j] << " ";
    //     }
    //     cout << "\n";
    // }


    Mat deltapose1= ((hessianInv*(sd_param.t())).t()); //calculate delta pose (change in pose)
    deltapose1=-deltapose1; //negating here as residual=(warpedImage-previmage), if residual=(previmage-warpedImage) then no minus sign needed

    //not used
    if(util::FLAG_USE_MOTION_PRIOR)
    {
        calMotionPrior();
        Mat deltapose2=(hessianInv*(motionPrior.t())).t();
        deltapose=deltapose1+deltapose2;
    }
    else
    {
        deltapose=deltapose1; //final deltapose
    }
    
    // cout<<"\nhessian Inv: \n"<<hessianInv;
    // cout<<"\nmotionPrior : \n"<<motionPrior;
    

    // float* deltapose1_ptr=deltapose1.ptr<float>(0); //initialize pointer to delta pose
    //float* deltapose2_ptr=deltapose2.ptr<float>(0); //initialize pointer to delta pose
    //printf("\n\nDelta Pose1: %f , %f , %f , %f , %f , %f", deltapose1_ptr[0],deltapose1_ptr[1],deltapose1_ptr[2],deltapose1_ptr[3],deltapose1_ptr[4],deltapose1_ptr[5]);
    
    //printf("\nDelta Pose2: %f , %f , %f , %f , %f , %f", deltapose2_ptr[0],deltapose2_ptr[1],deltapose2_ptr[2],deltapose2_ptr[3],deltapose2_ptr[4],deltapose2_ptr[5]);
    
    
    float* deltapose_ptr=deltapose.ptr<float>(0); //initialize pointer to delta pose
    
    //printf("\n\nFinal Delta Pose: %f , %f , %f , %f , %f , %f", deltapose_ptr[0],deltapose_ptr[1],deltapose_ptr[2],deltapose_ptr[3],deltapose_ptr[4],deltapose_ptr[5]);
    
    /*
     if(isnan(float(deltapose_ptr[0])))
     {
     //cout<<"\n\n\nHessian Inv : "<<hessianInv;
     //cout<<"\n\n\nSteepest descnet: "<<steepestDescent;
     cout<<"\n\n\nworld points: "<<worldPoints;
     cout<<"\n\n\nwarped points: "<<warpedPoints;
     cout<<"\n\n\nTransformed World points: "<<transformedWorldPoints;
     //cout<<"\nWeights: "<<weights;
     
     }
     */
    
    printf("\nDelta Pose: %f , %f ", deltapose_ptr[0],deltapose_ptr[1]);
    
    //cout<<"\nWeighted delta Pose "<<abs(deltapose_ptr[0]*util::weight[0])<<" , "<<abs(deltapose_ptr[1]*util::weight[1])<<" , "<<abs(deltapose_ptr[2]*util::weight[2])<<" , "<<abs(deltapose_ptr[3]*util::weight[3])<<" , "<<abs(deltapose_ptr[4]*util::weight[4])<<" , "<<abs(deltapose_ptr[5]*util::weight[5])<<" , ";
    
    
    //calculate weighted pose value to check termination condition 
    float weighted_pose = abs(deltapose_ptr[0]*util::weight[0])+abs(deltapose_ptr[1]*util::weight[1]);
    
    //printf("\n\nTotal Weighted Pose: %f \n\n ",weighted_pose);
    weightedPose=weighted_pose;
    
    //check condition for termination
    /*if( weighted_pose < util::threshold1)
     {
     PRINTF("\nWeighted pose below threshold! Terminating");
     return -1;  //terminate iteration loop
     }*/
    
    
    //update current estimate of pose
    /*pose[0]+=real(deltapose_ptr[0]);
    pose[1]+=real(deltapose_ptr[1]);
    pose[2]+=real(deltapose_ptr[2]);
    pose[3]+=real(deltapose_ptr[3]);
    pose[4]+=real(deltapose_ptr[4]);
    pose[5]+=real(deltapose_ptr[5]);*/
    
    current_frame->concatenateRelativePose(deltapose_ptr, pose, pose); //current estimate of pose updated
    
    //cout<<"\n\nDelta pose: "<<deltapose;
    
    //printf("\n\nFinal pose in func: %f , %f , %f , %f , %f , %f", pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);
    
    
}

//this function saves the pixel weight
//Case 1:when flag useAverageWeights is True:
//  saves pixel weight of the keyframe using the final weight estimated during normal sequence of pose estimation
//  the weights for the kf are the average of weights estimated at the final iteration for each currentimg that is referenced on that kf
//  these saved weights are then used in the inverse compositional algo during estimation of extra matches for loop closures
//Case 2: when flag useAverageWeights is False: (not used)
//simply saves the weight of the current image
void PixelWisePyramid::saveWeights(bool useAverageWeights)
{
    //for saving weights to current image after warping
    if(!useAverageWeights)
    {
    float* saved_warpedptrX;
    float* saved_warpedptrY;
    
    float* disp_weight;
    float* save_weight;
    
    for(int y=0;y<nRows;y++)
    {
        saved_warpedptrX=savedWarpedPointsX.ptr<float>(y);
        saved_warpedptrY=savedWarpedPointsY.ptr<float>(y);
        
        disp_weight=display_weightimg.ptr<float>(y);
        
        for(int x=0;x<nCols;x++)
        {
            int warpedx=int(floor(saved_warpedptrX[x]));
            int warpedy=int(floor(saved_warpedptrY[x]));
            
            //printf("\nbefore: %f, %f and after: %d, %d",saved_warpedptrX[x], saved_warpedptrY[x], warpedx, warpedy );
            
            
            if( (warpedx==-1 && warpedy==-1) || (warpedx==-2 && warpedy==-2)) //-1 for oob and -2 for zero depth
            {
                continue;
            }

            save_weight=current_frame->weight_pyramid[pyrlevel].ptr<float>(warpedy);
            
            //display_weightimg estimated in pose est. step corresponds to the warped image which is adjusted here
            save_weight[warpedx]=disp_weight[x];
        }
    }
    
    
    //DisplayWeightsPixelWise(current_frame->weight_pyramid[pyrlevel], prev_frame, "saved Weight", 1);
    
     //waitKey(1000);
    
    //printf("\nSaved weights for frame num: %d ", current_frame->frameId);
    return;
    }
    else
    {
        //display_weightimg is aligned with the prev image, therefore can be directly added
        prev_frame->weight_pyramid[pyrlevel]=prev_frame->weight_pyramid[pyrlevel]+display_weightimg;
        prev_frame->numWeightsAdded[pyrlevel]++;
        //printf("\nSaved weights for frame num: %d at level: %d with new weight count: %d", prev_frame->frameId, pyrlevel, prev_frame->numWeightsAdded[pyrlevel]);
    }
    
}



//precomputation step for inverse compositional algo
//precomputes the weighted steepest descent as gradient of template img(prev image) and pixel weights are constant
//it takes in a ymin and ymax which correspond to the starting and ending row number in an image
//this allows multiple threads to break the image in sections and use only those sections
//it also takes thread_number as the input in order to use the variables associated with that thread only, thread_number = 0 means a single thread
void PixelWisePyramid::precomputePixelWiseInvCompositional(int ymin, int ymax, int thread_num)
{
    //*******CALCULATE RESIZED INTRINSIC PARAMETERS*******//
    //printf("\nIn PRECOMPUTE pixel-wise with ymin: %d, ymax: %d, thread_num: %d", ymin,ymax,thread_num);
    //printf("\nIn precompute pixel-wise");
    
    vector<float> resized_intrinsic= GetIntrinsic(prev_frame->pyrLevel);
    float resized_fx=resized_intrinsic[0];
    float resized_fy=resized_intrinsic[1];
    float resized_cx=resized_intrinsic[2];
    float resized_cy=resized_intrinsic[3];
    
    ////////
    
    uchar* mask_ptr;
    
    float* gradx_ptr;
    float* grady_ptr;
    float* depth_ptr;
    float* weight_ptr;
    
    float* steepdesc_ptr0=steepestDescent.ptr<float>(0);
    float* steepdesc_ptr1=steepestDescent.ptr<float>(1);
    float* steepdesc_ptr2=steepestDescent.ptr<float>(2);
    
    float* weighted_steepdesc_ptr0=weightedSteepestDescent.ptr<float>(0);
    float* weighted_steepdesc_ptr1=weightedSteepestDescent.ptr<float>(1);
    float* weighted_steepdesc_ptr2=weightedSteepestDescent.ptr<float>(2);
    
    float jacob_bottom[3];
    float jacob_top[3];
   
    
    for(int y=ymin;y<ymax;y++)
    {
        mask_ptr=prev_frame->mask.ptr<uchar>(y);
        gradx_ptr=prev_frame->gradientx.ptr<float>(y);
        grady_ptr=prev_frame->gradienty.ptr<float>(y);
        depth_ptr=prev_frame->depth_pyramid[pyrlevel].ptr<float>(y);
        weight_ptr=prev_frame->weight_pyramid[pyrlevel].ptr<float>(y);
       // weight_ptr=prev_frame->weights.ptr<float>(y);
        
        for(int x=0; x<nCols;x++)
        {
            int idx=x+nCols*y;
            
            if(mask_ptr[x]==0)
            {
                steepdesc_ptr0[idx]=0;
                steepdesc_ptr1[idx]=0;
                steepdesc_ptr2[idx]=0;
                
                weighted_steepdesc_ptr0[idx]=0;
                weighted_steepdesc_ptr1[idx]=0;
                weighted_steepdesc_ptr2[idx]=0;

            
                continue;
            }
            
            //calculating gradient of prev image
            float gradx=gradx_ptr[x];
            float grady=grady_ptr[x];
            
            //calculating Jacobian
            jacob_bottom[0]  = grady*((resized_cy - y) * pow(depth_ptr[x],-1));
            jacob_top[0] = gradx*((resized_cx - x) * pow(depth_ptr[x],-1));

            jacob_bottom[1]  = grady*(-(resized_cy - y) * pow(depth_ptr[x],-1));
            jacob_top[1] = gradx*(-(resized_cx - x) * pow(depth_ptr[x],-1));
            
            jacob_bottom[2]=grady*(((-resized_cy + y)*(-resized_cx + x)) / resized_fx);
            jacob_top[2]= gradx*(- resized_fx - (pow((-resized_cx + x),2) / resized_fx));
            
            //calculate steepest descent
            //in case of multi-threads, each thread will populate this matrix at a particular pixel, therefore no conflict of memory access
            steepdesc_ptr0[idx]=jacob_top[0]+jacob_bottom[0];
            steepdesc_ptr1[idx]=jacob_top[1]+jacob_bottom[1];
            
            weighted_steepdesc_ptr0[idx]=steepdesc_ptr0[idx]*weight_ptr[x]; ///using the fixed weights in weight pyramid mat here
            weighted_steepdesc_ptr1[idx]=steepdesc_ptr1[idx]*weight_ptr[x];

        }
    }
    
    //printf("\nExiting PRECOMPUTE pixel-wise with ymin: %d, ymax: %d, thread_num: %d", ymin,ymax,thread_num);
    
    
}

//iteration step for inverse compositional algo
//warps image and calculates residual and sd param
//it takes in a ymin and ymax which correspond to the starting and ending row number in an image
//this allows multiple threads to break the image in sections and use only those sections
//it also takes thread_number as the input in order to use the variables associated with that thread only, thread_number = 0 means a single thread
void PixelWisePyramid::iteratePixelWiseInvCompositional(int ymin,int ymax, int thread_num)
{
   // printf("\nIn iterate pixel-wise");
    
    //printf("\nIn ITERATE pixel-wise with ymin: %d, ymax: %d, thread_num: %d", ymin,ymax,thread_num);
    //cout<<"\npixel wise!! rows: "<<nRows<<" col: "<<nCols;
    
    int i;
    int idx;
    
    //*******CALCULATE RESIZED INTRINSIC PARAMETERS*******//
    
    vector<float> resized_intrinsic= GetIntrinsic(prev_frame->pyrLevel);
    float resized_fx=resized_intrinsic[0];
    float resized_fy=resized_intrinsic[1];
    float resized_cx=resized_intrinsic[2];
    float resized_cy=resized_intrinsic[3];
    
    //*******PIXEL VARIABLES*******//
    
    float worldpointX;
    float worldpointY;
    float worldpointZ;
    
    float trfm_worldpointX;
    float trfm_worldpointY;
    float trfm_worldpointZ;
    
    float warpedpointx;
    float warpedpointy;
    
    float warpedintensity;
    
    float residual;
    
    //initializations
    switch (thread_num)
    {
        case 0:
            sd_param=Mat::zeros(1, 2, CV_32FC1);
            break;
        case 1:
            sd_param_thread1 =Mat::zeros(1, 2, CV_32FC1);
            break;
        case 2:
            sd_param_thread2=Mat::zeros(1, 2, CV_32FC1);
            break;
        case 3:
            sd_param_thread3=Mat::zeros(1, 2, CV_32FC1);
            break;
    }
    

    
    //*******INITIALIZE POINTERS*******//
    
    uchar* img_ptr;
    uchar* previmg_ptr;
    uchar* mask_ptr;
    float* depth_ptr;

    float* weight_ptr;
    
    float* steepdesc_ptr0=steepestDescent.ptr<float>(0);
    float* steepdesc_ptr1=steepestDescent.ptr<float>(1);
    float* steepdesc_ptr2=steepestDescent.ptr<float>(2);

    //*******DISPLAY POINTERS*******//
    uchar* disp_templateimg_ptr;
    uchar* disp_2bewarpedimg_ptr;
    float* disp_warpedimg_ptr;
    float* disp_iterationres_ptr;
    float* disp_origres_ptr;
    float* disp_weight_ptr;
    
    //*******STORE SE3 PARAMETERS*******//
    
    float c = cos(pose[1]);
    float s = sin(pose[1]);
    float r = pose[0];

    Mat SE3 = (Mat_<float>(4, 4) << c,0,-s,-(r*s),0,1,0,0,s,0,c,((r*c)-r),0,0,0,1);
    
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> SE3_Eigen(SE3.ptr<float>(), SE3.rows, SE3.cols);
    
    float* SE3_ptr;
    float SE3_vec[12];  //r11, r12, r13, t1, r21 r22, r23, t2, r31, r32, r33, t3
    int vec_counter=0;
    for(i=0; i<3; ++i)
    {
        SE3_ptr=SE3.ptr<float>(i); //get pointer to first row of SE3
        
        SE3_vec[vec_counter++]=SE3_ptr[0];
        SE3_vec[vec_counter++]=SE3_ptr[1];
        SE3_vec[vec_counter++]=SE3_ptr[2];
        SE3_vec[vec_counter++]=SE3_ptr[3];
        
    }
    
    float tx=SE3_Eigen(0,3);
    float ty=SE3_Eigen(1,3);
    float tz=SE3_Eigen(2,3);
    

    for(int y=ymin;y<ymax;y++)
    {
        previmg_ptr=prev_frame->image_pyramid[pyrlevel].ptr<uchar>(y);
        img_ptr=current_frame->image_pyramid[pyrlevel].ptr<uchar>(y);
        mask_ptr=prev_frame->mask.ptr<uchar>(y);
        depth_ptr=prev_frame->depth_pyramid[pyrlevel].ptr<float>(y);
        weight_ptr=prev_frame->weight_pyramid[pyrlevel].ptr<float>(y);
        
        disp_templateimg_ptr=display_templateimg.ptr<uchar>(y);
        disp_2bewarpedimg_ptr=display_2bewarpedimg.ptr<uchar>(y);
        disp_warpedimg_ptr=display_warpedimg.ptr<float>(y);
        disp_iterationres_ptr=display_iterationres.ptr<float>(y);
        disp_origres_ptr=display_origres.ptr<float>(y);
        disp_weight_ptr=display_weightimg.ptr<float>(y);
        
        for(int x=0;x<nCols;x++)
        {
            //check for zero depth point
            if(mask_ptr[x]==0)
            {
                disp_templateimg_ptr[x]=0;
                disp_2bewarpedimg_ptr[x]=0;
                disp_warpedimg_ptr[x]=0;
                disp_iterationres_ptr[x]=0;
                disp_origres_ptr[x]=0;
                disp_weight_ptr[x]=0;
                continue;  //skip loop for zero depth points
            }
            
            disp_templateimg_ptr[x]=img_ptr[x];
            disp_2bewarpedimg_ptr[x]=previmg_ptr[x];
            disp_origres_ptr[x]=img_ptr[x]-previmg_ptr[x];
            //cout<<"\n"<<disp_origres_ptr[x];
            
            idx=x+nCols*y;
            
            
            //world point  ( X ;Y ;depth )
            worldpointX=(x-resized_cx)*depth_ptr[x]/resized_fx; //X-coordinate
            worldpointY=(y-resized_cy)*depth_ptr[x]/resized_fy; //Y-coordinate
            worldpointZ=depth_ptr[x]; //depth of point at (u,v)
            
            //printf("\nworld points: (%f, %f, %f)", worldpointX,worldpointY,worldpointZ);
            
            
            //calculating transformed world point and warped point
            if (SE3_vec[1]==0)
            {
                trfm_worldpointX=((SE3_vec[0]*worldpointX)+(SE3_vec[1]*worldpointY)+(SE3_vec[2]*worldpointZ)+(SE3_vec[3]));
                trfm_worldpointY=((SE3_vec[4]*worldpointX)+(SE3_vec[5]*worldpointY)+(SE3_vec[6]*worldpointZ)+(SE3_vec[7]));
                trfm_worldpointZ=((SE3_vec[8]*worldpointX)+(SE3_vec[9]*worldpointY)+(SE3_vec[10]*worldpointZ)+(SE3_vec[11]));
                
                trfm_worldpointZ=UNZERO(trfm_worldpointZ);
                
                warpedpointx=((trfm_worldpointX/trfm_worldpointZ)*resized_fx)+resized_cx;
                warpedpointy=((trfm_worldpointY/trfm_worldpointZ)*resized_fy)+resized_cy;
            }
            else
            {
                trfm_worldpointX=(float(SE3_vec[0]*worldpointX)+float(SE3_vec[1]*worldpointY)+float(SE3_vec[2]*worldpointZ)+float(SE3_vec[3]));
                trfm_worldpointY=(float((SE3_vec[4]*worldpointX))+float((SE3_vec[5]*worldpointY))+float((SE3_vec[6]*worldpointZ))+float((SE3_vec[7])));
                trfm_worldpointZ=(float(SE3_vec[8]*worldpointX)+float(SE3_vec[9]*worldpointY)+float(SE3_vec[10]*worldpointZ)+float(SE3_vec[11]));
                
                trfm_worldpointZ=UNZERO(trfm_worldpointZ);
                
                warpedpointx=float(((trfm_worldpointX/trfm_worldpointZ)*resized_fx)+resized_cx);
                warpedpointy=float(((trfm_worldpointY/trfm_worldpointZ)*resized_fy)+resized_cy);
                
            }
            
            //printf("\nthread_num: %d, (x, y) : (%d ,%d) , warped point : (%f, %f), world points: (%f, %f, %f)", thread_num, x,y, warpedpointx,warpedpointy, worldpointX,worldpointY,worldpointZ);
            
            //printf("\ntransformed world point: (%f,%f,%f)", trfm_worldpointX,trfm_worldpointY,trfm_worldpointZ);
            
            //calculate warped intensity
            warpedintensity= current_frame->getInterpolatedElement(warpedpointx,warpedpointy, 1); //!!!!!
            
            if(warpedintensity==-1)
            {
                disp_warpedimg_ptr[x]=0;
            }
            else
            {
                disp_warpedimg_ptr[x]=warpedintensity;
            }
            
            if(warpedintensity==-1)
            {
                residual=0.0f;
            }
            else
                residual=warpedintensity-float(previmg_ptr[x]); //current-warped
            
            disp_iterationres_ptr[x]=residual;
            disp_weight_ptr[x]=weight_ptr[x]; //using fixed weight
            //printf("\nwt: %f, warped int: %f, residual: %f",disp_weight_ptr[x], warpedintensity, residual);
            
            Mat steepestdescentMat=(Mat_<float>(1,2)<<steepdesc_ptr0[idx],steepdesc_ptr1[idx]);
            //cout<<"\n"<<steepestdescentMat;
            
            
            switch (thread_num)
            {
                case 0:
                    sd_param+=steepestdescentMat.mul(residual*weight_ptr[x]); //1x2
                    break;
                case 1:
                    sd_param_thread1+=steepestdescentMat.mul(residual*weight_ptr[x]); //1x2
                    break;
                case 2:
                    sd_param_thread2+=steepestdescentMat.mul(residual*weight_ptr[x]); //1x2
                    break;
                case 3:
                    sd_param_thread3+=steepestdescentMat.mul(residual*weight_ptr[x]); //1x2
                    break;
            }
            
            //cout<<sd_param;
        }
    }
    
    //printf("\nExiting ITERATE pixel-wise with ymin: %d, ymax: %d, thread_num: %d", ymin,ymax,thread_num);
}

//inverse compositional gauss newton
//forms threads and calls actual function which will do all the computation
void PixelWisePyramid::calculatePixelWiseParallelInvCompositional(int iter)
{
    //printf("\n\n\nIn inv compositional pixel-wise");

        if(util::FLAG_DO_PARALLEL_CONST_WEIGHT_POSE_EST) //if flag for using inverse compositional with constant weights and multiple thread is on
        {
            boost::thread_group t_group;
            int y_increment=nRows/util::NUM_CONST_WT_POSE_EST_THREADS; //section size of image that is passed on to each thread
            
           // printf("\ny_increment: %d, rows: %d, cols: %d", y_increment, current_frame->image_pyramid[pyrlevel].rows, current_frame->image_pyramid[pyrlevel].cols);
            
            //precomputation, only at the first iteration of a pyramid level
            if(iter==0)
            {
                //hardcoded to form only 3 threads. If need more threads will need to create them
                t_group.create_thread(boost::bind(&PixelWisePyramid::precomputePixelWiseInvCompositional,this, 0,y_increment,1));
                t_group.create_thread(boost::bind(&PixelWisePyramid::precomputePixelWiseInvCompositional,this, y_increment,2*y_increment,2));
                t_group.create_thread(boost::bind(&PixelWisePyramid::precomputePixelWiseInvCompositional,this, 2*y_increment,nRows,3));
                t_group.join_all();
                
                //reaches here when all threads have been joined
                hessian=weightedSteepestDescent*(steepestDescent.t()); //hessian is also precomputed
                hessianInv=hessian.inv();
                
            }

            //iteration steps
            //hardcoded to form only 3 threads. If need more threads will need to create them
            t_group.create_thread(boost::bind(&PixelWisePyramid::iteratePixelWiseInvCompositional,this, 0,y_increment,1));
            t_group.create_thread(boost::bind(&PixelWisePyramid::iteratePixelWiseInvCompositional,this, y_increment,nRows,2));
            t_group.join_all();
            
            //reaches here when all threads have been joined
            //forms the final sd_param by summing over all the pixels
            sd_param=sd_param_thread1+sd_param_thread2;
            
            //update current estimate of pose
            updatePose();
            
        }
        else //when using inv. compositional algo with constant weight but single thread
        {
            
            //precomputation only at the first iteration of pyramid level
            if(iter==0)
            {   precomputePixelWiseInvCompositional(0, nRows, 0); //single thread number 0 created only
                
                hessian=weightedSteepestDescent*(steepestDescent.t());
                hessianInv=hessian.inv();
            }
            
            //iteration
            iteratePixelWiseInvCompositional(0, nRows, 0);
            updatePose();
            
        }
    return;
}



