// NOT USED (NITIKA)
//
//  depth_Archive.cpp
//  odometry_depth_estimation_not_pixelwise
//
//  Created by Himanshu Aggarwal on 10/9/15.
//  Copyright (c) 2015 Himanshu Aggarwal. All rights reserved.
//

#include "depth_Archive.h"

depth_Archive::depth_Archive(){
    no_of_files=5;
    hsize=256;
    hrange[0]=0;
    hrange[1]=255;
    hist_h=400;
    hist_w=512;
    bin_w=cvRound( (double) hist_w/hsize );
    pop_idx=0;
}

void depth_Archive::depth_Pushback(depthhypothesis *currentDepthMap,Mat image)
{
    static int no_of_pushbacks=0;
    
    start_idx=no_of_pushbacks%5;
    
    const float *hranges[]={hrange};
    
    memcpy(depth_archive_arr[start_idx], currentDepthMap, util::ORIG_COLS*util::ORIG_ROWS*sizeof(depthhypothesis));
    
    image_archive[start_idx]=image.clone();
    
   // imshow("Depth archive image", image_archive[start_idx]);
    
    calcHist(&image_archive[start_idx], 1, 0, Mat(), image_histogram[start_idx], 1, &hsize, hranges, true, false);
    
    //normalize(image_histogram[start_idx], image_histogram[start_idx], 0, hist_h, NORM_MINMAX, -1, Mat());
    float sum=0;
    for(int i=0;i<hsize;i++)
    {
        sum+=image_histogram[start_idx].at<float>(i);
    }
    
    for(int i=0;i<hsize;i++)
    {
        image_histogram[start_idx].at<float>(i)/=sum;
    }
    
    
    //MatND normalizeHist;
    //normalize(image_histogram[start_idx], normalizeHist, 0, hist_h, NORM_MINMAX, -1, Mat());
    
    
   // showHistogram(normalizeHist);
    
    no_of_pushbacks++;
}

void depth_Archive::showHistogram(MatND histogram_normalized, string name)
{
   
    Mat histImage( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );
        
    for( int i = 1; i < hsize; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(histogram_normalized.at<float>(i-1)) ) ,
                Point( bin_w*(i), hist_h - cvRound(histogram_normalized.at<float>(i)) ),
                 Scalar( 255, 0, 0), 2, 8, 0  );
    }
        
    imshow( name, histImage );

}

double depth_Archive::compareHistograms(MatND first_histogram_norm, MatND second_histogram_norm)
{
return compareHist(first_histogram_norm, second_histogram_norm, CV_COMP_KL_DIV);
  /*  double KLdiv=0.0f;
    for(int i=0;i<hsize;i++)
    {
        KLdiv+=(first_histogram_norm.at<float>(i)*(log2(double(first_histogram_norm.at<float>(i)))-log2(double(second_histogram_norm.at<float>(i)))));
        printf("\n %f", KLdiv);
    }
    return KLdiv;
   */
    
}

bool depth_Archive::depth_Pop(MatND current_histogram_norm)
{
    char current;
    for (int i=0; i<5; i++)
    {
        current=start_idx-i>=0?start_idx-i:start_idx-i+5;
        //if(compareHistograms(image_histogram[current], current_histogram_norm)<3000)
        //{
            //printf("\nMatch value: %f, start id: %d", compareHistograms(current_histogram_norm, image_histogram[current]), int(current));
            //pop_idx=current;
            //printf("MATCH FOUND AT %d", int(pop_idx));
            //showHistogram(image_histogram[current]);
           // return true;
        
       // }
    }
    
    
    showHistogram(image_histogram[0], "hist 0");
    showHistogram(image_histogram[1], "hist 1");
    showHistogram(image_histogram[2], "hist 2");
    showHistogram(image_histogram[3], "hist 3");
    showHistogram(image_histogram[4], "hist 4");
    
    
    
    return false;
}

