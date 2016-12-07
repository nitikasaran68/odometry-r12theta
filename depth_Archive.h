//
//  depth_Archive.h
//  odometry_depth_estimation_not_pixelwise
//
//  Created by Himanshu Aggarwal on 10/9/15.
//  Copyright (c) 2015 Himanshu Aggarwal. All rights reserved.
//

#ifndef __odometry_depth_estimation_not_pixelwise__depth_Archive__
#define __odometry_depth_estimation_not_pixelwise__depth_Archive__

#include <stdio.h>
#include "Frame.h"

#include <cstdio>
#include <ctime>
#include <iostream>

#include "ExternVariable.h"
#include <cmath>
#include "DepthPropagation.h"
#include "DepthHypothesis.h"

using namespace std;
using namespace cv;

class depth_Archive
{
public:
    int start_idx;
    char no_of_files;  //CONVERT TO INT!!!!!!!!!!!!!!
    int hist_w;
    int hist_h;
    int bin_w;
    int pop_idx;
    
    //constants
    int hsize;
    float hrange[2];
    
    Mat image_archive[5];
    MatND image_histogram[5];
    depthhypothesis depth_archive_arr[5][5000000];
    
    depth_Archive();

    void depth_Pushback(depthhypothesis* currentDepthMap,Mat image);
    void showHistogram(MatND histogram_normalized, string name="histogram1");
    double compareHistograms(MatND first_histogram_norm,MatND second_histogram_norm);
    bool depth_Pop(MatND current_histigram_norm);
};

#endif /* defined(__odometry_depth_estimation_not_pixelwise__depth_Archive__) */
