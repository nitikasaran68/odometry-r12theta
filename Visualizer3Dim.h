//
//  Visualizer3Dim.h
//  odometry code 6b
//
//  Created by Himani Arora on 08/02/16.
//  Copyright (c) 2016 Himani Arora. All rights reserved.
//

#ifndef __odometry_code_6b__Visualizer3Dim__
#define __odometry_code_6b__Visualizer3Dim__

#include"opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "ExternVariable.h"


using namespace std;
using namespace cv;

namespace Visualizer3Dim
{

//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
inline boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
    viewer->addCoordinateSystem ( 1.0 );
    viewer->initCameraParameters ();
    return (viewer);
}

inline void reconstructPointCloud(Mat img_rgb, Mat img_disparity)
{

    float depth_multipler=1.0f;
    cv::imshow("rbg-image", img_rgb);
    std::cout << "\n\nPress a key to continue..." << std::endl;
    cv::waitKey(0);
    //cv::destroyWindow("rgb-image");
    
    //Create point cloud and fill it
    std::cout << "Creating Point Cloud..." <<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    double px, py, pz;
    uchar pr, pg, pb;
    
    for (int i = 0; i < img_rgb.rows; i++) //i-> y
    {
        uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
        float* disp_ptr = img_disparity.ptr<float>(i);
        
        for (int j = 0; j < img_rgb.cols; j++) //j-> x
        {
            //Get 3D coordinates
            float d = (depth_multipler*disp_ptr[j]); //  /util::GLOABL_DEPTH_SCALE; //depth or disparity
            if ( d == 0 )
                continue; //Discard bad pixels
            
            px=((float(j)-util::ORIG_CX)/util::ORIG_FX)*float(d);
            py=((float(i)-util::ORIG_CY)/util::ORIG_FY)*float(d);
            pz=float(d);
            
            //printf("\npx: %f, py: %f, pz: %f", px,py,pz);
            
            //Get RGB info
            pb = rgb_ptr[3*j];
            pg = rgb_ptr[3*j+1];
            pr = rgb_ptr[3*j+2];
            
            //Insert info into point cloud structure
            pcl::PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                            static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);
        }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    
    //Create visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = createVisualizer( point_cloud_ptr );
    
    int viewer_hold=0;
    //Main loop
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        viewer_hold++;
        //cout<<"\nWAS STOPPED:  "<<viewer->wasStopped();
        //cout<<"\n"<<viewer_hold;
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    viewer->close();
    //cout<<"\nWAS STOPPED:  "<<viewer->wasStopped();
    cout<<"\nReturning!";
    return;
    
}
    

}


#endif /* defined(__odometry_code_6b__Visualizer3Dim__) */
