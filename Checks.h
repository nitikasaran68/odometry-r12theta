//
//  Checks.h
//  odometry_depth_estimation_not_pixelwise
//
//  Created by Himanshu Aggarwal on 10/9/15.
//  Copyright (c) 2015 Himanshu Aggarwal. All rights reserved.
//

#ifndef odometry_depth_estimation_not_pixelwise_Checks_h
#define odometry_depth_estimation_not_pixelwise_Checks_h

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>


#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "Histogram.h"
#include "HistogramData.h"
#include "Frame.h"

//*******CONSTANTS*******//

using namespace std;
using namespace cv;
using namespace Eigen;

namespace check {
    
    static const int max_exceeded_rotations=0;
    static const int max_tracked_on_homography=50;
    static const int min_track_on_keyframe=5;
    
    static const int HOMO_WINDOW=4;
    static const int HOMO_MIN_TRIGGERS=3;
    static const int LIE_MAX_TRIGGER=2;
    static bool homography_ON=false;
    static bool homography_ON_prev=false;
    static bool switch_Keyframe=false;
    static int count_exceeded_rotations;
    static int count_tracked_on_homography;
    static int count_for_keyframe_prop=0;
    
    static bool triggerWorkingWindow[HOMO_WINDOW]={0,0,0,0};
    
    inline void pushToTriggerWindow(bool switch_std_dev)
    {
        static int num_calls=0;
        triggerWorkingWindow[num_calls%HOMO_WINDOW]=switch_std_dev;
        num_calls++;
    }   
    
    inline void switch_to_homography()
    {
        int i,count=0;
        for(i=0;i<HOMO_WINDOW;i++)
        {
            count+=int(triggerWorkingWindow[i]);
        }
        
        if(count>=HOMO_MIN_TRIGGERS)
        {   homography_ON_prev=homography_ON;
            homography_ON=true;
        }
        
    }
    
    inline void switch_back_to_lie()
    {
        int i,count=0;
        for(i=0;i<HOMO_WINDOW;i++)
        {
            count+=int(triggerWorkingWindow[i]);
        }
        
        if(count<=LIE_MAX_TRIGGER)
        {   homography_ON_prev=homography_ON;
            homography_ON=false;
        }
    }
    
    inline char homography_trigger()
    {
        if(homography_ON_prev==false && homography_ON==true)
            return 1;
        else if(homography_ON_prev==true && homography_ON==false)
            return 2;
        else
            return 0;
    }

    inline bool check_with_std_deviation(histogram* histogram,float w1,float w2,float w3)
    {
    
        if (abs(w1-histogram->mean[0])>2*histogram->stdDeviation[0]||abs(w2-histogram->mean[1])>2*histogram->stdDeviation[1]||abs(w3-histogram->mean[2])>2*histogram->stdDeviation[2]) {
            return true;
        }
        
        return false;
    }
    
    inline void  check_for_keyframe_propagation(frame* current_frame)
    {
        count_for_keyframe_prop++;
        
        float weighted_pose_for_keyframe_switching = abs(current_frame->poseWrtOrigin[0]*util::weight[0])+abs(current_frame->poseWrtOrigin[1]*util::weight[1])+abs(current_frame->poseWrtOrigin[2]*util::weight[2]);

        cout<<"\n weighted pose for keyframe prop :  "<<weighted_pose_for_keyframe_switching/1000.0f<<endl;
        
        if((homography_trigger()==1||weighted_pose_for_keyframe_switching/1000.0f>util::switchKeyframeThreshold) && count_for_keyframe_prop>=min_track_on_keyframe)
        {
            count_for_keyframe_prop=0;
            switch_Keyframe = true;
        }
        else
        switch_Keyframe=false;
    }
    
    inline void updateCheckFlags(bool switch_std_dev,frame* current_frame)
    {
        cout<<"\nhomo flag"<<homography_ON;
        pushToTriggerWindow(switch_std_dev);
        switch_to_homography();
        cout<<"\nswitch to homo flag"<<homography_ON;
        switch_back_to_lie();
        cout<<"\nswitch to lie homo flag"<<homography_ON;
        check_for_keyframe_propagation(current_frame);
        
        cout<<"\n\n\nFINAL HOMOGRAPHY ON FLAG: ";
        if(homography_ON==true)
            cout<<"ON!!";
        else
            cout<<"OFF!!";
        
        cout<<"\nSWITCH TO NEW KEYFRAME: ";
        if(switch_Keyframe==true)
            cout<<"ON!!"<<"\n\n";
        else
            cout<<"OFF"<<"\n\n";
    
    }
}
#endif
