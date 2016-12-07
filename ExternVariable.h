
#pragma once

#ifndef odometry_code_3_ExternVariable_h
#define odometry_code_3_ExternVariable_h

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <time.h>
#include <sys/time.h>
#include "opencv2/opencv.hpp"
#include <complex>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <sstream>


//*******CONSTANTS*******//

using namespace std;
using namespace cv;
using namespace Eigen;


namespace util
{
    
void  initializeConstants();

    // canon eos 7d

    static const int KEYFRAME_PROPAGATE_INTERVAL=4;
    static const int MAX_PYRAMID_LEVEL=4;
    static const int INTRINSIC_FACTOR=2;
    static const float DIM_FACTOR=2.0f; //dimension factor for resizing images
    static const float RESIZE_FACTOR=1/float(DIM_FACTOR);
    
    static const int ORIG_COLS= 1920/DIM_FACTOR;  //1920/DIM_FACTOR;//640/1.0f; //1920/2;//1080;//540;//
    static const int ORIG_ROWS =1080/DIM_FACTOR; //1080/DIM_FACTOR; //480/1.0f; //1080/2;//720;//360;//
    
    static const float ORIG_FX = 2200.8356f/INTRINSIC_FACTOR; //1236/INTRINSIC_FACTOR;//535.4f/1.0f; // 481.20f;//1642.405612f/4.0f; //1680.85023f/4.0f;//1642.405612f/4.0f; //1524.7f/2;//3785.54549f/2.0f;  //1003.4f/1.0f;//
    static const float ORIG_FY = 2153.899f/INTRINSIC_FACTOR;//1250/INTRINSIC_FACTOR;539.2f/1.0f; //  -480.00f;//1636.148027f/4.0f;//1678.14263f/4.0f;//1636.148027f/4.0f;// 1524.7f/2;// 3790.35277f/2.0f;//1002.4f/1.0f ;//
    static const float ORIG_CX = ORIG_COLS/2.0f; ////320.1f/1.0f; // 319.50f;// 892.153028f/4.0f;//970.51439f/4.0f;//892.153028f/4.0f;//320.1f; //(1920.0f/2.0f)/2.0f;//(1280.0f/2.0f)/2.0f; //540.0f/1.0f;//526.4/2.0f;//
    static const float ORIG_CY = ORIG_ROWS/2.0f;// // 247.6f/1.0f; // 239.50f;// 483.240652f/4.0f;//563.73722f/4.0f;//483.240652f/4.0f;//247.6f; //(1080.0f/2.0f)/2.0f;//(720.0f/2.0f)/2.0f;//360.0f/1.0f;//355.3/2.0f;//
    

    
    //hujichetan orig (1280x720)


    /*
    static const int KEYFRAME_PROPAGATE_INTERVAL=8;
    static const int MAX_PYRAMID_LEVEL=4;
    static const int INTRINSIC_FACTOR=2;
    static const float DIM_FACTOR=2.0f; //dimension factor for resizing images
    static const float RESIZE_FACTOR=1/float(DIM_FACTOR);
    
    static const int ORIG_COLS= 1280/DIM_FACTOR;  //1920/DIM_FACTOR;//640/1.0f; //1920/2;//1080;//540;//
    static const int ORIG_ROWS =720/DIM_FACTOR; //1080/DIM_FACTOR; //480/1.0f; //1080/2;//720;//360;//
    
    static const float ORIG_FX = 1642.405612f/INTRINSIC_FACTOR; //1236/INTRINSIC_FACTOR;//535.4f/1.0f; // 481.20f;//1642.405612f/4.0f; //1680.85023f/4.0f;//1642.405612f/4.0f; //1524.7f/2;//3785.54549f/2.0f;  //1003.4f/1.0f;//
    static const float ORIG_FY = 1636.148027f/INTRINSIC_FACTOR;//1250/INTRINSIC_FACTOR;539.2f/1.0f; //  -480.00f;//1636.148027f/4.0f;//1678.14263f/4.0f;//1636.148027f/4.0f;// 1524.7f/2;// 3790.35277f/2.0f;//1002.4f/1.0f ;//
    static const float ORIG_CX =ORIG_COLS/2.0f; ////320.1f/1.0f; // 319.50f;// 892.153028f/4.0f;//970.51439f/4.0f;//892.153028f/4.0f;//320.1f; //(1920.0f/2.0f)/2.0f;//(1280.0f/2.0f)/2.0f; //540.0f/1.0f;//526.4/2.0f;//
    static const float ORIG_CY =ORIG_ROWS/2.0f;// // 247.6f/1.0f; // 239.50f;// 483.240652f/4.0f;//563.73722f/4.0f;//483.240652f/4.0f;//247.6f; //(1080.0f/2.0f)/2.0f;//(720.0f/2.0f)/2.0f;//360.0f/1.0f;//355.3/2.0f;//
    */
    
    //hujichetan orig (1920x1080)
    
    /*
    static const int KEYFRAME_PROPAGATE_INTERVAL=8;
    static const int MAX_PYRAMID_LEVEL =4;
    static const int INTRINSIC_FACTOR=4;
    static const float DIM_FACTOR=4.0f; //dimension factor for resizing images
    static const float RESIZE_FACTOR=1/float(DIM_FACTOR);

    static const int ORIG_COLS= 1920/DIM_FACTOR;  //1920/DIM_FACTOR;//640/1.0f; //1920/2;//1080;//540;//
    static const int ORIG_ROWS =1080/DIM_FACTOR; //1080/DIM_FACTOR; //480/1.0f; //1080/2;//720;//360;//

    static const float ORIG_FX = 1642.405612f/INTRINSIC_FACTOR; //535.4f/1.0f; // 481.20f;//1642.405612f/4.0f; //1680.85023f/4.0f;//1642.405612f/4.0f; //1524.7f/2;//3785.54549f/2.0f;  //1003.4f/1.0f;//
    static const float ORIG_FY = 1636.148027f/INTRINSIC_FACTOR;//539.2f/1.0f; //  -480.00f;//1636.148027f/4.0f;//1678.14263f/4.0f;//1636.148027f/4.0f;// 1524.7f/2;// 3790.35277f/2.0f;//1002.4f/1.0f ;//
    static const float ORIG_CX =892.153028f/INTRINSIC_FACTOR; ////320.1f/1.0f; // 319.50f;// 892.153028f/4.0f;//970.51439f/4.0f;//892.153028f/4.0f;//320.1f; //(1920.0f/2.0f)/2.0f;//(1280.0f/2.0f)/2.0f; //540.0f/1.0f;//526.4/2.0f;//
    static const float ORIG_CY =483.240652f/INTRINSIC_FACTOR;// // 247.6f/1.0f; // 239.50f;// 483.240652f/4.0f;//563.73722f/4.0f;//483.240652f/4.0f;//247.6f; //(1080.0f/2.0f)/2.0f;//(720.0f/2.0f)/2.0f;//360.0f/1.0f;//355.3/2.0f;//
    */
    
    //walkabout
    /*
    static const int MAX_ITER[4]={ 4, 7, 9, 12} ; //0->highest image size, 3->smallest image size
    static const int KEYFRAME_PROPAGATE_INTERVAL=8;
    static const int MAX_PYRAMID_LEVEL =4;
    static const int INTRINSIC_FACTOR=2;
    static const float DIM_FACTOR=2.0f; //dimension factor for resizing images
    static const float RESIZE_FACTOR=1/float(DIM_FACTOR);
    
    static const int ORIG_COLS= 854/DIM_FACTOR;  //1920/DIM_FACTOR;//640/1.0f; //1920/2;//1080;//540;//
    static const int ORIG_ROWS =480/DIM_FACTOR; //1080/DIM_FACTOR; //480/1.0f; //1080/2;//720;//360;//
    
    static const float ORIG_FX = 1642.405612f/INTRINSIC_FACTOR; //535.4f/1.0f; // 481.20f;//1642.405612f/4.0f; //1680.85023f/4.0f;//1642.405612f/4.0f; //1524.7f/2;//3785.54549f/2.0f;  //1003.4f/1.0f;//
    static const float ORIG_FY = 1636.148027f/INTRINSIC_FACTOR;//539.2f/1.0f; //  -480.00f;//1636.148027f/4.0f;//1678.14263f/4.0f;//1636.148027f/4.0f;// 1524.7f/2;// 3790.35277f/2.0f;//1002.4f/1.0f ;//
    static const float ORIG_CX =ORIG_COLS/2.0f;//892.153028f/INTRINSIC_FACTOR; ////320.1f/1.0f; // 319.50f;// 892.153028f/4.0f;//970.51439f/4.0f;//892.153028f/4.0f;//320.1f; //(1920.0f/2.0f)/2.0f;//(1280.0f/2.0f)/2.0f; //540.0f/1.0f;//526.4/2.0f;//
    static const float ORIG_CY =ORIG_ROWS/2.0f;//483.240652f/INTRINSIC_FACTOR;// // 247.6f/1.0f; // 239.50f;// 483.240652f/4.0f;//563.73722f/4.0f;//483.240652f/4.0f;//247.6f; //(1080.0f/2.0f)/2.0f;//(720.0f/2.0f)/2.0f;//360.0f/1.0f;//355.3/2.0f;//
    */
    
    //american lung egomotion
    /*
    static const int MAX_ITER[4]={ 4, 7, 9, 12} ; //0->highest image size, 3->smallest image size
    static const int KEYFRAME_PROPAGATE_INTERVAL=8;
    static const int MAX_PYRAMID_LEVEL =4;
    static const int INTRINSIC_FACTOR=4;
    static const float DIM_FACTOR=2.0f; //dimension factor for resizing images
    static const float RESIZE_FACTOR=1/float(DIM_FACTOR);
    
    static const int ORIG_COLS= 960/DIM_FACTOR;  //1920/DIM_FACTOR;//640/1.0f; //1920/2;//1080;//540;//
    static const int ORIG_ROWS =720/DIM_FACTOR; //1080/DIM_FACTOR; //480/1.0f; //1080/2;//720;//360;//
    
    static const float ORIG_FX = 1642.405612f/INTRINSIC_FACTOR; //535.4f/1.0f; // 481.20f;//1642.405612f/4.0f; //1680.85023f/4.0f;//1642.405612f/4.0f; //1524.7f/2;//3785.54549f/2.0f;  //1003.4f/1.0f;//
    static const float ORIG_FY = 1636.148027f/INTRINSIC_FACTOR;//539.2f/1.0f; //  -480.00f;//1636.148027f/4.0f;//1678.14263f/4.0f;//1636.148027f/4.0f;// 1524.7f/2;// 3790.35277f/2.0f;//1002.4f/1.0f ;//
    static const float ORIG_CX =ORIG_COLS/2.0f;//892.153028f/INTRINSIC_FACTOR; ////320.1f/1.0f; // 319.50f;// 892.153028f/4.0f;//970.51439f/4.0f;//892.153028f/4.0f;//320.1f; //(1920.0f/2.0f)/2.0f;//(1280.0f/2.0f)/2.0f; //540.0f/1.0f;//526.4/2.0f;//
    static const float ORIG_CY =ORIG_ROWS/2.0f;//483.240652f/INTRINSIC_FACTOR;// // 247.6f/1.0f; // 239.50f;// 483.240652f/4.0f;//563.73722f/4.0f;//483.240652f/4.0f;//247.6f; //(1080.0f/2.0f)/2.0f;//(720.0f/2.0f)/2.0f;//360.0f/1.0f;//355.3/2.0f;//
    */
    
    
    //hyperlapse
    
    //static const int MAX_ITER[4]={ 4, 7, 9, 12} ; //0->highest image size, 3->smallest image size
    // static const int KEYFRAME_PROPAGATE_INTERVAL=8;
    // static const int MAX_PYRAMID_LEVEL =4;
    // static const int INTRINSIC_FACTOR=4;
    // static const float DIM_FACTOR=4.0f; //dimension factor for resizing images
    // static const float RESIZE_FACTOR=1/float(DIM_FACTOR);
    
    // static const int ORIG_COLS= 1280/DIM_FACTOR;  //1920/DIM_FACTOR;//640/1.0f; //1920/2;//1080;//540;//
    // static const int ORIG_ROWS =960/DIM_FACTOR; //1080/DIM_FACTOR; //480/1.0f; //1080/2;//720;//360;//
    
    // static const float ORIG_FX = 1642.405612f/INTRINSIC_FACTOR; //535.4f/1.0f; // 481.20f;//1642.405612f/4.0f; //1680.85023f/4.0f;//1642.405612f/4.0f; //1524.7f/2;//3785.54549f/2.0f;  //1003.4f/1.0f;//
    // static const float ORIG_FY = 1636.148027f/INTRINSIC_FACTOR;//539.2f/1.0f; //  -480.00f;//1636.148027f/4.0f;//1678.14263f/4.0f;//1636.148027f/4.0f;// 1524.7f/2;// 3790.35277f/2.0f;//1002.4f/1.0f ;//
    // static const float ORIG_CX =ORIG_COLS/2.0f;//892.153028f/INTRINSIC_FACTOR; ////320.1f/1.0f; // 319.50f;// 892.153028f/4.0f;//970.51439f/4.0f;//892.153028f/4.0f;//320.1f; //(1920.0f/2.0f)/2.0f;//(1280.0f/2.0f)/2.0f; //540.0f/1.0f;//526.4/2.0f;//
    // static const float ORIG_CY =ORIG_ROWS/2.0f;//483.240652f/INTRINSIC_FACTOR;// // 247.6f/1.0f; // 239.50f;// 483.240652f/4.0f;//563.73722f/4.0f;//483.240652f/4.0f;//247.6f; //(1080.0f/2.0f)/2.0f;//(720.0f/2.0f)/2.0f;//360.0f/1.0f;//355.3/2.0f;//
    
    
    
    static const float weight[]={100000.0f,100000.0f,100000.0f,10000.0f,10000.0f,10000.0f};
    static const float level_term[]={0.999f,1.0f,1.05f,1.1f,1.1f};//{1.1f,1.07f,1.05f,1.0f,0.999f};
    static const float threshold1= 10.0f;
    static const float switchKeyframeThreshold=5.5f;

    static const float MIN_ABS_GRAD_CREATE=1.0f;
    static const float MIN_ABS_GRAD_DECREASE=5.0f;
    static const int MIN_BLACKLIST=-1;
    
    static const float  MAX_DIFF_CONSTANT =40.0f*40.0f;
    static const float MAX_DIFF_GRAD_MULT = 0.5f*0.5f;
    
    static const float VAR_RANDOM_INIT_INITIAL=0.125f;
    
    
// ============== initial stereo pixel selection ======================
    static const float MIN_EPL_GRAD_SQUARED=(2.0f*2.0f);
    static const float MIN_EPL_LENGTH_SQUARED= (1.0f*1.0f);
    static const float MIN_EPL_ANGLE_SQUARED= (0.3f*0.3f);
   
    
// ============== stereo & gradient calculation ======================
    static const float MIN_DEPTH =0.05f ;// this is the minimal depth tested for stereo., using now! 
    
    // particularely important for initial pixel.
    static const float MAX_EPL_LENGTH_CROP =30.0f; // maximum length of epl to search.
    static const float MIN_EPL_LENGTH_CROP =3.0f; // minimum length of epl to search.
    
    // this is the distance of the sample points used for the stereo descriptor.
    static const float GRADIENT_SAMPLE_DIST =1.0f;  //using now!
    
    // pixel a point needs to be away from border... if too small: segfaults!
    static const float SAMPLE_POINT_TO_BORDER= 7.0f;  //using now ; MADE FLOAT
    
    // pixels with too big an error are definitely thrown out.
    static const float MAX_ERROR_STEREO =1300.0f; // maximal photometric error for stereo to be successful (sum over 5 squared intensity differences)
    static const float MIN_DISTANCE_ERROR_STEREO =1.5f; // minimal multiplicative difference to second-best match to not be considered ambiguous.
    
    // defines how large the stereo-search region is. it is [mean] +/- [std.dev]*STEREO_EPL_VAR_FAC
    static const float STEREO_EPL_VAR_FAC= 2.0f;
    
    static const float DIVISION_EPS =1e-10f;
    
    /// to be understood///////////////////////
    static const int CAMERA_PIXEL_NOISE=4*4;
    static const float VAR=0.5f*0.5f;
    static const int VALIDITY_COUNTER_INITIAL_OBSERVE=5;	// initial validity for first observations
    
    static const float SUCC_VAR_INC_FAC=(1.01f); // before an ekf-update, the variance is increased by this factor.
    static const float FAIL_VAR_INC_FAC=1.1f; // after a failed stereo observation, the variance is increased by this factor.
    static const float MAX_VAR=(0.5f*0.5f); // initial variance on creation - if variance becomes larter than this, hypothesis is removed.

    
    static const float VAR_GT_INIT_INITIAL=0.01f*0.01f;	// initial variance vor Ground Truth Initialization
    
    static const float DIFF_FAC_OBSERVE=(1.0f*1.0f);
    static const float DIFF_FAC_PROP_MERGE= (1.0f*1.0f);

    static const float VALIDITY_COUNTER_MAX=(5.0f);
    static const float VALIDITY_COUNTER_MAX_VARIABLE=(250.0f);
    static const float VALIDITY_COUNTER_DEC=5.0f;
    static const float VALIDITY_COUNTER_INC=5.0f;
    static const float REFERANCE_FRAME_MAX_AGE= 10.0f;

    static const float IDEPTH_MULIPLIER=100;
    //static const float IDEPTH_MULIPLIER=70;
    
    static const float VAL_SUM_MIN_FOR_CREATE= 30.0f;
    static const float VAL_SUM_MIN_FOR_UNBLACKLIST=100.0f;
    static const float VAL_SUM_MIN_FOR_KEEP=24.0f;
    
    static const float REG_DIST_VAR= 0.075f*0.075f*1.0f*1.0f;
    static const float DIFF_FAC_SMOOTHING=1.0f*1.0f;
    
    static const float CAMERA_PIXEL_NOISE_2=4.0f*4.0f;
    static const float HUBER_D=3.0f;
    
    static const float HISTOGRAM_STEP=0.001f;
    static const float HISTOGRAM_EXTREME=0.500f;
    static const int HISTOGRAM_SIZE=1001;//(((HISTOGRAM_EXTREME/HISTOGRAM_STEP))*2)+1;//
    static const int HISTOGRAM_INIT_FRAMES=1;//200;
    
    static const int YMIN=3;
    static const int YMAX=ORIG_ROWS-3; //<change>
    static const int XMIN=3;
    static const int XMAX=ORIG_COLS-3;
    
    static const int MAX_LOOP_ARRAY_LENGTH=20;
    static const int MAX_LOOP_ARRAY_LENGTH_SCALE_AVG=(MAX_LOOP_ARRAY_LENGTH*2)+3;
    static const float MATCH_THRESHOLD=0.1f;
    static const int MIN_MATCH_DIFFERENCE=KEYFRAME_PROPAGATE_INTERVAL;
    static const float MAX_REL_VIEW_ANGLE=10.0f;
    static const int MIN_WAIT_COUNT=0;
    static const int MIN_SEEDS_FOR_DJK_CALCULATION=5;
    static const float TRIGGER_LOOP_CLOSURE_ON=20.0f;
    static const float TRIGGER_LOOP_CLOSURE_OFF=1.0f;
    
    static const float MIN_SEEDS_FOR_CONNECTION_LOST=0.0f;
    
    
    
    /////////utility flags ///////////
    
    //flags for results: display, save, write
    static const bool FLAG_DISPLAY_IMAGES=false;
    static const bool FLAG_DISPLAY_DEPTH_MAP=false;
    static const bool FLAG_SAVE_DEPTH_MAP=true;
    static const bool FLAG_SAVE_MATCH_IMAGES=false;
    static const bool FLAG_DO_3D_VISUALIZATION=false;
    
    static const bool FLAG_WRITE_ORIG_POSES=true;
    static const bool FLAG_WRITE_MATCH_POSES=true;
    static const bool FLAG_WRITE_SIM_AVG_VARS=false;
    
    //flags for improved accuracy
    
    static  bool FLAG_DO_SIMILARITY_AVERAGE=false;
    static  bool FLAG_DO_UNDISTORTION=false;
    static  bool FLAG_RESTORE_CONNECTION=false;
    static  bool FLAG_USE_MOTION_PRIOR=false; //uses prev frame pose!
    static  bool FLAG_REPLICATE_POSE_ESTIMATION=false; //not used now , change iterations! (to replicate depth and pose estimation conditions)
    
    static  bool FLAG_ALTERNATE_GN_RA=false; //gauss newton, rotation averaging

    static  bool FLAG_CONCATENATE_INITIAL_POSE=false;
    static  int  CONCATENATE_STEP=160;
    
    //flags for parallel multi-thread operations
    static  bool FLAG_DO_PARALLEL_DEPTH_ESTIMATION=true;
    static  bool FLAG_DO_PARALLEL_POSE_ESTIMATION=true;

    
    
    
    //extern vars
   
    extern int MAX_ITER[4];  //0->highest image size, 3->smallest image size

    //aleternate GN, RA
    extern  int  FLAG_IS_BOOTSTRAP;
    extern  int  BATCH_START_ID;
    extern  int  BATCH_SIZE; //in terms of number of ketframe propagations needed
    
    extern  int  NUM_GN_PROPAGATION;
    extern  int  NUM_RA_PROPAGATION;
    extern bool  GAUSS_NEWTON_FLAG_ON;
    extern bool  ROTATION_AVERAGING_FLAG_ON;
    
    extern  bool FLAG_DO_LOOP_CLOSURE; //LC
    extern  bool FLAG_REPLICATE_NEW_DEPTH;
    extern  bool FLAG_INITIALIZE_NONZERO_POSE;
    extern  bool FLAG_SAVE_MATS;
    
    extern  bool FLAG_DO_PARALLEL_SHORT_LOOP_CLOSURE; //LC
    
    //within loop closure
    extern  bool FLAG_DO_CONST_WEIGHT_POSE_ESTIMATION; //LC
    extern  bool FLAG_DO_PARALLEL_CONST_WEIGHT_POSE_EST; //LC
    extern  bool FLAG_USE_LOOP_CLOSURE_TRIGGER;
    
    extern bool EXIT_CONDITION;

    
    
    
    
    //strings
    static const string SAVED_MATS_PATH="/Users/himaniarora/Desktop/Saved_mats";
    
    
// ================== MULTITHREAD VARIABLES ============================================
    
    //need to be made directly configureable!!
    static const int NUM_POSE_THREADS=3;
    static const int NUM_DEPTH_THREADS=3;
    static const int NUM_LOOP_CLOSURE_THREADS=1;
    static const int NUM_CONST_WT_POSE_EST_THREADS=3;
    
    static float GLOABL_DEPTH_SCALE=1.0f;
    
// ================== MACROS ============================================
    #define UNZERO(val) (val < 0 ? (val > -1e-10 ? -1e-10 : val) : (val < 1e-10 ? 1e-10 : val))
    

// ================== UTILITY CLASS ============================================

    //time measure
    class measureTime
    {
    
        struct timeval start;
        struct timeval end;
        double elapsed;
        
    public:
        inline void startTimeMeasure()
        {
         gettimeofday(&start, NULL);
        }
    
        inline float stopTimeMeasure(bool millisec=true)
        {
            if(millisec)
            {
            gettimeofday(&end, NULL);
            elapsed = ((end.tv_sec - start.tv_sec) * 1000)
            + (end.tv_usec / 1000 - start.tv_usec / 1000);
            return elapsed;
            }
            gettimeofday(&end, NULL);
            elapsed = ((end.tv_sec - start.tv_sec) * 1000000)
            + (end.tv_usec - start.tv_usec);
            return elapsed;
 
        }
        
    };
    

}
    
    
#endif
