//
//  Histogram.h
//  odometry code 4
//
//  Created by Himani Arora on 07/10/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//
#pragma once

#ifndef __odometry_code_4__Histogram__
#define __odometry_code_4__Histogram__

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "ExternVariable.h"
#include <math.h>
#include "HistogramData.h"

class histogram
{
public:
    histogramData histogramDataArray[3][util::HISTOGRAM_SIZE];
    float stdDeviation[3];
    float mean[3];
    bool isComplete;
    
    void initializeHistogramVal(void); //initializes the value parameter
    void addToHistogram(float w1,float w2,float w3);
    void saveHistogramToFile(ofstream* gnufile);
    void printHistogram(void);
    void calStdDeviation(void);
   
    
};


#endif /* defined(__odometry_code_4__Histogram__) */
