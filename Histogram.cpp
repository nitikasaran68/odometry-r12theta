//
//  Histogram.cpp
//  odometry code 4
//
//  Created by Himani Arora on 07/10/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//


#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <cstdio>
#include<ctime>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include<complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>


#include "Histogram.h"

using namespace cv;

void histogram::initializeHistogramVal()
{
    //int count=0;
    for (int i=0; i<3; i++) {
    
    histogramData* histogramarr_ptr=histogramDataArray[i];
    
    isComplete=false;
    for(float i=-util::HISTOGRAM_EXTREME; i<=util::HISTOGRAM_EXTREME;i=i+util::HISTOGRAM_STEP)
     {
        histogramarr_ptr->val=i;
       
        
        //printf("\ni: %f ",i);
        //i=histogramarr_ptr->val;
        
        histogramarr_ptr++;
        
        //printf(",  i: %f ",-0.462+0.001);
        //i=i+0.001f;
        //count++;
    
      }
    }
    return;
}

void histogram::addToHistogram(float w1,float w2,float w3)
{
    static int count=0;
    count++;
    
    float values[]={w1,w2,w3};
    for(int i=0;i<3;i++)
    {
    int beg, end, mid;
    int addedValueFlag=0;
    if(values[i]>=0)
    {
        beg=util::HISTOGRAM_SIZE/2;
        end=util::HISTOGRAM_SIZE;
    }
    else
    {
        beg=0;
        end=util::HISTOGRAM_SIZE/2;
    }
    
    while(!addedValueFlag)
    {
        mid=(beg+end)/2;
        //printf("\nbeg: %d, end: %d, mid: %d:",beg,end,mid );
        
        if(mid>=util::HISTOGRAM_SIZE-1)
        {
            histogramDataArray[i][mid].freq++;
            addedValueFlag=1;
        }
        
        else if(mid==0)
        {
            histogramDataArray[i][mid].freq++;
            printf("\nbeg: %d, end: %d, mid: %d:",beg,end,mid );
            addedValueFlag=1;
        }
    
        else if(values[i]>=histogramDataArray[i][mid].val && values[i]<histogramDataArray[i][mid+1].val)
        {
            if(values[i]>=histogramDataArray[i][mid].val+util::HISTOGRAM_STEP/2)
                histogramDataArray[i][mid+1].freq++;
            else
                histogramDataArray[i][mid].freq++;
            addedValueFlag=1;
        }
        
        else if(values[i]>histogramDataArray[i][mid].val)
        {
            beg=mid;
        }
        else
        {
            end=mid;
        }
       // if(addedValueFlag)
       //     printf("\nAdded! mid: %d", mid);
    }
 }
    if(count==util::HISTOGRAM_INIT_FRAMES)
    {
        isComplete=true;
        calStdDeviation();
    }
}

void histogram::printHistogram()
{
    for (int j=0; j<3; j++) {
     
    printf("\nSIZE: %d",util::HISTOGRAM_SIZE);
    for(int i=0;i<util::HISTOGRAM_SIZE;i++)
    {
        printf("\nchannel=%c i= %d, val= %f, freq= %d",j,i,histogramDataArray[j][i].val,histogramDataArray[j][i].freq);
        
    }
  }
    return;
    
}

void histogram::calStdDeviation()
{
    /*for (int j; j<3; j++) {
           mean[j]=0.0f;
    for(int i=0;i<util::HISTOGRAM_SIZE;i++)
    {
        mean[j]+=(histogramDataArray[j][i].val*histogramDataArray[j][i].freq);
    }
    mean[j]/=util::HISTOGRAM_INIT_FRAMES;
    
    stdDeviation[j]=0.0f;
    for(int i=0;i<util::HISTOGRAM_SIZE;i++)
    {
        stdDeviation[j]+=((histogramDataArray[j][i].val-mean[j])*(histogramDataArray[j][i].val-mean[j])*histogramDataArray[j][i].freq);
    }
    stdDeviation[j]/=util::HISTOGRAM_INIT_FRAMES;
    stdDeviation[j]=float(sqrt(stdDeviation[j]));

    printf("\nChannel:%c Mean: %f, Std Deviation: %f", j, mean[j], stdDeviation[j]);
    
    //meanStdDev(histogramDataArray->val, mean, stdDeviation);
  }*/
    //printf("\n in calstd deviation");
    stdDeviation[0]=0.008105f;
    stdDeviation[1]= 0.005767f;
    stdDeviation[2]=0.009531f;
    
    mean[0]=-0.007423f;
    mean[1]=-0.005758f;
    mean[2]=-0.001728f;

}

void histogram::saveHistogramToFile(ofstream *gnufile)
{
    if(gnufile->is_open())
    {
        for(int i=0;i<util::HISTOGRAM_SIZE;i++)
        {

        for (int j=0; j<3; j++) {
           
                    *gnufile<<histogramDataArray[j][i].val<<" "<<histogramDataArray[j][i].freq<<" ";
        }
            *gnufile<<"\n";
      }
    }
    
}




