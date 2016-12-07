//
//  SimilarityAverage.hpp
//  Loop_closure_sim_avg
//
//  Created by Himanshu Aggarwal on 1/16/16.
//  Copyright © 2016 Himanshu Aggarwal. All rights reserved.
//

#ifndef SimilarityAverage_h
#define SimilarityAverage_h

#include <cstdio>
#include<ctime>
#include <cstdlib>
#include"opencv2/opencv.hpp"
#include<complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "Frame.h"
#include "DepthPropagation.h"
#include "GlobalOptimize.h"
#include "EigenInitialization.h"
//#include "Visualizer3Dim.h"
#include "Pyramid.h"


using namespace cv;
using namespace std;

struct graph_center{
    
    int frameId=-1; //frame id of center
    bool isValid=false; //becomes true when Djk has been calculated
    vector<int> matchedArrayId; //vector of matched frame array ids
    vector<tuple<float,float,float,float,float,float>> matchedRelativePoses; //vector of poses w.r.t matched frames
    vector<tuple<int,int,float>> Djk;// (graph node array id 1, graph node array id 2, Djk value b/w 1 and 2)
   // Mat warpedDepthMap=Mat::zeros(util::)
    int warpingCenter; //center node array id
    
    
};

class SimilarityAverage
{
    
public:
    
    globalOptimize* aliasGlobalOptimize;
    
    graph_center graphCenterArchive[util::MAX_LOOP_ARRAY_LENGTH_SCALE_AVG];
    
    ofstream write_sim_params_file;
    
    //constructor
    SimilarityAverage(globalOptimize* dummyGlobalOptimize, string write_sim_params_path);
    
    //methods
    void determineCombinations(int centerKfArrayID);
    
    void fillUpDjks(int centerKfArrayID);
    
    void fillPoseAndMatches(int loopMatchedID, int currentArrayId,float* relPose);
    
    void resetArrayElement(int arrayID);
    
    void calculateWarpedDepthMap( int matchID,int centerID,tuple<float,float,float,float,float,float> relPose);
    
    void findDepthByTriangulation(int centerID, int matchID);
    
    tuple<float,float,float,float,float,float> inverseTuple(tuple<float,float,float,float,float,float> inputTuple);
    
    float calculateDjks(int node_J_arrayid, int node_K_arrayid);
    
    void finaliseSimilarityAverageParams(int centerKfArrayID);
    
    //checking funcs
    void printVars(int centerKfArrayID );
    void compareLoopFrameId();
    void clearGraphArray();
    void displayWarpedDepth(int matchID,int centerID);
    
    
    
    
};

#endif /* SimilarityAverage_h */
