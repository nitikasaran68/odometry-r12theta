//
//  Homography.cpp
//  odometry_depth_estimation_not_pixelwise
//
//  Created by Himanshu Aggarwal on 10/8/15.
//  Copyright (c) 2015 Himanshu Aggarwal. All rights reserved.
//

#include "Homography.h"
#include <Eigen/SVD>

using namespace cv;
using namespace cv::xfeatures2d;


void performHomography(frame* prev_frame,frame* current_frame)
{
//     int minHessian=400;
    
//    // SurfFeatureDetector detector(minHessian);
  
//     Ptr<SURF> detector = SURF::create(minHessian);

//     std::vector<KeyPoint> keypoints_prev, keypoints_current;
//     detector->detect( prev_frame->image, keypoints_prev );
//     detector->detect( current_frame->image, keypoints_current);
   
//     Ptr<SURF> extractor = SURF::create();
//     Mat descriptors_prev;
//     Mat descriptors_current;
    
//     extractor->compute( prev_frame->image, keypoints_prev, descriptors_prev );
//     extractor->compute( current_frame->image, keypoints_current, descriptors_current);
    
//     FlannBasedMatcher matcher;
//     std::vector< DMatch > matches;
    
//     if ( descriptors_prev.empty() )
//         cvError(0,"MatchFinder","1st descriptor empty",__FILE__,__LINE__);
//     if ( descriptors_current.empty() )
//         cvError(0,"MatchFinder","2nd descriptor empty",__FILE__,__LINE__);
    
//     matcher.match( descriptors_prev, descriptors_current, matches );
//     double max_dist = 0; double min_dist = 100;
    
//     //-- Quick calculation of max and min distances between keypoints
//     for( int i = 0; i < descriptors_prev.rows; i++ )
//     { double dist = matches[i].distance;
//         if( dist < min_dist ) min_dist = dist;
//         if( dist > max_dist ) max_dist = dist;
//     }
    
//     printf("-- Max dist : %f \n", max_dist );
//     printf("-- Min dist : %f \n", min_dist );
    
//     //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//     std::vector< DMatch > good_matches;
    
//     for( int i = 0; i < descriptors_prev.rows; i++ )
//     { if( matches[i].distance < 3*min_dist )
//     { good_matches.push_back( matches[i]); }
//     }
    
//     Mat img_matches;
//     drawMatches( prev_frame->image, keypoints_prev, current_frame->image, keypoints_current,
//                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

//     //-- Localize the object
//     std::vector<Point2f> prev;
//     std::vector<Point2f> current;
    
//     for( int i = 0; i < good_matches.size(); i++ )
//     {
//         //-- Get the keypoints from the good matches
//         prev.push_back( keypoints_prev[ good_matches[i].queryIdx ].pt );
//         current.push_back( keypoints_current[ good_matches[i].trainIdx ].pt );
//     }
    
//     Mat homography = findHomography( prev, current, CV_RANSAC );
//     Mat homo;
//     homography.convertTo(homo, CV_32F);
   
//     vector<Mat> Rot;
//     vector<Mat> Trans;
//     vector<Mat> Norm;
//     decomposeHomographyMat(homography, util::K, Rot, Trans, Norm);
//     cout<<"\n\n\nRotation 0: \n"<<Rot[0];
//     cout<<"\n\nRotation 1: \n"<<Rot[1];
//     cout<<"\n\nRotation 2: \n"<<Rot[2];
//     cout<<"\n\nRotation 3: \n"<<Rot[3];
    
    
    
    
//     //Mat posefrmHomo=util::K.inv()*homo*util::K;
    
    
//     //float factor= pow(determinant(posefrmHomo), 1/3);
//     //posefrmHomo/=factor;
    
//     //cout<<"\nFACTOR= "<<factor;
    
//     //cout<<"\n\n\nPose from home: \n"<<posefrmHomo;
    
//     Eigen::MatrixXf final_rot;
    
//     for(int i=0;i<4;i++)
//     {
//         Mat X;
//         Rot[i].convertTo(X, CV_32FC1);
//         Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>posefrmhomo_Eigen(X.ptr<float>(), X.rows, X.cols);
        
//         cout<<"\n\n\nSo3_rot for i= "<<i<<" \n"<<posefrmhomo_Eigen.log();
//         //cout<<"\n\nSo3 rot from svd \n"<<so3_svd;
//        // cout<<"\n determinant of rot from svd "<<determinant;
//         //cout<<"\nCheck orthogonality? \n"<<Rot[i]*Rot[i].t();
        
//         if(i==0)
//             final_rot=posefrmhomo_Eigen.log();
//     }
//     float se3posefrmHomo[6]={final_rot(2,1),final_rot(0,2),final_rot(1,0),0.0f,0.0f,0.0f};

//     current_frame->calculatePoseWrtOrigin(prev_frame, se3posefrmHomo,true);
//     current_frame->calculatePoseWrtWorld(prev_frame, se3posefrmHomo,true);

    
//     display_warped_image(homo, prev_frame, current_frame);
    
//     return;

}


void display_warped_image(Mat homography,frame* prev_frame,frame* current_frame)
{      
   // int pyr=current_frame->pyrLevel;
    // current_frame->pyrLevel=0;
    
    // current_frame->updationOnPyrChange(0,true);
    
    // Mat warped_points=Mat::zeros(2,util::ORIG_ROWS * util::ORIG_COLS, CV_32FC1);
    // Mat warpedimg=Mat::zeros(util::ORIG_ROWS, util::ORIG_COLS, CV_32FC1);
    
    // float* warpedptrx=warped_points.ptr<float>(0);
    // float* warpedptry=warped_points.ptr<float>(1);
    // float* warpedimg_ptr=warpedimg.ptr<float>(0);
    
    // Mat result(3,1,CV_32FC1);
    // float* u=result.ptr<float>(0);
    // float* v=result.ptr<float>(1);
    // float* z=result.ptr<float>(2);
    
    // int k=0;
    
    // for(int i=0;i<util::ORIG_ROWS;i++)
    // {
    //     warpedimg_ptr=warpedimg.ptr<float>(i);
    //     for (int j=0; j<util::ORIG_COLS; j++) {
           
    //        Mat X=(Mat_<float>(3,1)<<j,i,1);
    //         result=homography*X;
            
    //         warpedptrx[k]=(*u)/(*z);
            
    //         warpedptry[k]=(*v)/(*z);
            
    //         warpedimg_ptr[j]=current_frame->getInterpolatedElement(warpedptrx[k], warpedptry[k]);
            
    //         k++;
    //     }
    // }
    
    // //current_frame->pyrLevel=pyr;
    
    // Mat displaywarpedimg;
    
    // warpedimg.convertTo(displaywarpedimg, CV_8UC1);
    // imshow("Homo Warped",displaywarpedimg);
    // waitKey(30);
    
    // Mat residual=abs(displaywarpedimg-prev_frame->image);
    // residual.convertTo(residual, CV_8UC1);
    // imshow("Residual homo",residual);
    // waitKey(30);

    
    // return;
}





void cameraPoseFromHomography(const Mat& H, Mat& pose)
{
    pose = Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
    float norm1 = (float)norm(H.col(0));
    float norm2 = (float)norm(H.col(1));
    float tnorm = (norm1 + norm2) / 2.0f; // Normalization value
    
    Mat p1 = H.col(0);       // Pointer to first column of H
    Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)
    
    cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose
    
    p1 = H.col(1);           // Pointer to second column of H
    p2 = pose.col(1);        // Pointer to second column of pose (empty)
    
    cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose
    
    p1 = pose.col(0);
    p2 = pose.col(1);
    
    Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
    Mat c2 = pose.col(2);    // Pointer to third column of pose
    p3.copyTo(c2);       // Third column is the crossproduct of columns one and two
    
    pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
}


void calculatePoseEssentialMat(frame* prev_frame,frame* current_frame)
{
    // int minHessian=400;
    
    // // SurfFeatureDetector detector(minHessian);
    
    // Ptr<SURF> detector = SURF::create(minHessian);
    
    // std::vector<KeyPoint> keypoints_prev, keypoints_current;
    // detector->detect( prev_frame->image, keypoints_prev );
    // detector->detect( current_frame->image, keypoints_current);
    
    // Ptr<SURF> extractor = SURF::create();
    // Mat descriptors_prev;
    // Mat descriptors_current;
    
    // extractor->compute( prev_frame->image, keypoints_prev, descriptors_prev );
    // extractor->compute( current_frame->image, keypoints_current, descriptors_current);
    
    // FlannBasedMatcher matcher;
    // std::vector< DMatch > matches;
    
    // if ( descriptors_prev.empty() )
    //     cvError(0,"MatchFinder","1st descriptor empty",__FILE__,__LINE__);
    // if ( descriptors_current.empty() )
    //     cvError(0,"MatchFinder","2nd descriptor empty",__FILE__,__LINE__);
    
    // matcher.match( descriptors_prev, descriptors_current, matches );
    // double max_dist = 0; double min_dist = 100;
    
    // //-- Quick calculation of max and min distances between keypoints
    // for( int i = 0; i < descriptors_prev.rows; i++ )
    // { double dist = matches[i].distance;
    //     if( dist < min_dist ) min_dist = dist;
    //     if( dist > max_dist ) max_dist = dist;
    // }
    
    // printf("-- Max dist : %f \n", max_dist );
    // printf("-- Min dist : %f \n", min_dist );
    
    // //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    // std::vector< DMatch > good_matches;
    
    // for( int i = 0; i < descriptors_prev.rows; i++ )
    // { if( matches[i].distance < 3*min_dist )
    // { good_matches.push_back( matches[i]); }
    // }
    
    // Mat img_matches;
    // drawMatches( prev_frame->image, keypoints_prev, current_frame->image, keypoints_current,
    //             good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
    //             vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    
    // //-- Localize the object
    // std::vector<Point2f> prev;
    // std::vector<Point2f> current;
    
    // for( int i = 0; i < good_matches.size(); i++ )
    // {
    //     //-- Get the keypoints from the good matches
    //     prev.push_back( keypoints_prev[ good_matches[i].queryIdx ].pt );
    //     current.push_back( keypoints_current[ good_matches[i].trainIdx ].pt );
    // }
    // double focal=double(util::ORIG_FX+util::ORIG_FY/2.0f);
    // cv::Point2d pp(util::ORIG_CX, util::ORIG_CY);
    
    // Mat E, R, t;
    
    // E = findEssentialMat(prev, current, focal, pp, RANSAC, 0.999, 1.0);
    // recoverPose(E, prev, current, R, t, focal, pp);
    
    
    
    // cout<<"\n\nRotation for ess: \n"<<R;
    // cout<<"\n\ntranslation from ess:\n"<<t;
    // R.convertTo(R, CV_32F);
    
    
    // double y=t.at<double>(0,0)*t.at<double>(0,0)+t.at<double>(1,0)*t.at<double>(1,0)+t.at<double>(2,0)*t.at<double>(2,0);
    
    // cout<<"\n\nTRanslation mag: "<<y<<endl;
    
    // Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>rotfrmess_Eigen(R.ptr<float>(), R.rows, R.cols);
    // cout<<"\n\n\nRot foress = "<<" \n"<<rotfrmess_Eigen;
    // Eigen::MatrixXf rotfrmess_Eigen_log=rotfrmess_Eigen.log();
    // cout<<"\n\n\nSo3_rotess = "<<" \n"<<rotfrmess_Eigen_log;

    // float se3posefrmEss[6]={rotfrmess_Eigen_log(2,1),rotfrmess_Eigen_log(0,2),rotfrmess_Eigen_log(1,0),0.0f,0.0f,0.0f};
    
    // // printf("\nPose after homography!!");
    // current_frame->calculatePoseWrtOrigin(prev_frame, se3posefrmEss,true);
    // current_frame->calculatePoseWrtWorld(prev_frame, se3posefrmEss,true);
    
    // return;
}

