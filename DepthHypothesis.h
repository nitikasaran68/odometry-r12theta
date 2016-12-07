//
//  DepthHypothesis.h
//  odometry
//
//  Created by Himanshu Aggarwal on 8/29/15.
//  Copyright (c) 2015 Himanshu Aggarwal. All rights reserved.
//
#pragma once

#ifndef odometry_DepthHypothesis_h
#define odometry_DepthHypothesis_h

//structure that contains the variables associated with the depth of a pixel in the current depth map
struct depthhypothesis
{        
    int pixId;
    
    float invDepth=0.0f;//stores inv depth
    
    float invDepthSmoothed=0.0f; //after regularization

    float variance=0.0f;
    
    float varianceSmoothed=0.0f; //after regularization
    
    bool isValid=false;
    
    bool isPerfect=false; //not used
    
    int nUsed=0;
    
    int oldestFrameId=1;
    
    int successful=0;
    
    int validity_counter=0;
    
    int blacklisted=0;
    
   // float idepth_smoothed=-1;
    
    //float idepth_var_smoothed=-1;
};

#endif


