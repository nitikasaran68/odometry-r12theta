

#include <cstdio>
#include <ctime>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include <complex>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

#include "DepthPropagation.h"
#include "depth_Archive.h"
#include "Checks.h"
#include "GnuPLots.h"
#include "DisplayFunc.h"
#include "ImageFunc.h"
#include "ExternVariable.h"
#include "Frame.h"
#include "EigenInitialization.h"
#include "Histogram.h"
#include "Trajectory.h"
#include "GlobalOptimize.h"
#include "SimilarityAverage.h"

#include "ToggleFlags.h"


using namespace std;
using namespace cv;

//---------------------------------------------


//--------- EXTERN VARS INIIALIZATION ------------

// Non-constant flags/variables that can be changed by program during alternate RA-GN mode

int util::MAX_ITER[]={4,7,9,12} ; //maximum GN iteration at different levels, 0->highest image size, 3->smallest image size

// alternate GN, RA
// these are used in alternate GN-RA mode
int  util::FLAG_IS_BOOTSTRAP=false;                         //read from samplefile
int  util::BATCH_START_ID=0;                                //read from samplefile
int  util::BATCH_SIZE=0;                                    //read from samplefile, make default iniialization as 1!!! (in terms of number of keyframe propagations needed)

int  util::NUM_GN_PROPAGATION=0;
int  util::NUM_RA_PROPAGATION=0;
bool util::GAUSS_NEWTON_FLAG_ON=false;
bool util::ROTATION_AVERAGING_FLAG_ON=false;

//toggle flags depending on requirement
bool util::FLAG_DO_LOOP_CLOSURE=false;                      //(LC) to find extra poses through short loop closure detection
bool util::FLAG_REPLICATE_NEW_DEPTH=false;                  //use a saved depth matrix saved as a text file during pose est.
bool util::FLAG_INITIALIZE_NONZERO_POSE=false;              //use non-zero pose as initialization
bool util::FLAG_SAVE_MATS=false;                            //save depth matrices as text files

bool util::FLAG_DO_PARALLEL_SHORT_LOOP_CLOSURE=false;       //(LC) use multi-threads for loop closure

//flags within loop closure
bool util::FLAG_DO_CONST_WEIGHT_POSE_ESTIMATION=false;      //(LC) use constant weights during LC for extra matches
bool util::FLAG_DO_PARALLEL_CONST_WEIGHT_POSE_EST=false;    //(LC) use multi-threaded implementation of const. weight
bool util::FLAG_USE_LOOP_CLOSURE_TRIGGER=false;             //default value, dont change

bool util::EXIT_CONDITION=false;                            //default value, don't change

//---------------------------------------------


// fstream objects to read & write
ifstream my_file;               //for samplefile.txt
ofstream pose_file_orig;        //to write orig poses
ofstream match_file;            //to write extra matches
ofstream similarityavg_file;    //no longer used
ifstream initialize_pose_file;  //read non-zero pose initializations


int main(int argc, char *argv[])
{
    my_file.open(argv[1]);

    // check if sample file opened
    if (!my_file.is_open()) 
    { 
        printf("\nUnable to open Sample Text File!");
        return -1;
    }


    //--------- READING INPUT FILE ------------
    
    string vid_path;                                    // first line: path of video/images
    getline(my_file,  vid_path);
    
    string match_path;                                  // second line: path to write relative poses between original rame and its keyframe
    getline(my_file,  match_path);
    match_file.open(match_path);
    
    string match_path_globalopt;                        // third line: path to write relative poses of extra matches calculated through global optimization
    getline(my_file,  match_path_globalopt);
    
    string depthmap_path;                               // fourth line: path of folder to store depth map associated with kf
    getline(my_file,  depthmap_path);
    
    string pose_path_orig;                              // fifth line: path to write absolute (w.r.t world) poses of each frame computed on its original kf
    getline(my_file,  pose_path_orig);
    pose_file_orig.open(pose_path_orig);
    
    string matchsave_path;                              // sixth line: path to folder to save matched images on which extra matches are computed
    getline(my_file,  matchsave_path);
    
    string similarityaverage_path;                      // seventh line: similarity average is no longer used
    getline(my_file,  similarityaverage_path);
    similarityavg_file.open(similarityaverage_path);
    
    string initialize_pose_path;                        // eight line: path to text file containing absolute(w.r.t world) poses to be used as initializations
    getline(my_file,  initialize_pose_path);
    initialize_pose_file.open(initialize_pose_path);
    
    
    //--------- BATCH PARAMETER INITIALIZATION ------------

    if(util::FLAG_ALTERNATE_GN_RA)
    {
        my_file>>util::BATCH_START_ID; //set start frame id of batch
        my_file>>util::BATCH_SIZE; //set batch size in terms of kf propagations
        my_file>>util::FLAG_IS_BOOTSTRAP;
    }

    bool toExit=false;
    if(util::FLAG_ALTERNATE_GN_RA )
    {
        toExit=checkExitCondition(); //updates GN and RA flags
    }    
    
    //--------- VIDEO INITIALIZATION ------------

    VideoCapture bgr_capture(vid_path);
    
    // check if video file opened
    if (!bgr_capture.isOpened())
    { 
        printf("\nUnable to open Frame File!");
        return -1;
    }
    
    // skips all images in folder till first frame of batch is reached
    if(util::FLAG_REPLICATE_NEW_DEPTH || (util::FLAG_IS_BOOTSTRAP && (util::BATCH_START_ID>0)))
    {
        Mat temp_im;
        int m;
        for(m=1;m<util::BATCH_START_ID;m++)
        {
            bgr_capture>>temp_im;
            cout<<"\nSkipping frame: "<<m;
            waitKey(5);
        }
    }

    // get the number of frames in video
    float no_of_frames = bgr_capture.get(CV_CAP_PROP_FRAME_COUNT);
    
    // max number of frames
    if(no_of_frames>32500)
        no_of_frames=32500;

    printf("\nNo of frames:%f",no_of_frames);
    
    
    //--------- MAIN PARAMETER INITIALIZATION ------------
    
    // vector of pointers to each frame currently in memory
    vector<frame*> frameptr_vector;
    
    vector<float>pose_vec;
        
    frame* activeKeyFrame=NULL;
    depthMap* currentDepthMap=new depthMap();
    
    currentDepthMap->save_depthmap_path=depthmap_path;
    
    check::homography_ON=false;                             //homography is no longer used
    depth_Archive* present_depth_archive=new depth_Archive; //depth archive is no longer used
    histogram* initialRotHistogram=new histogram;           //rotation histogram is no longer used
    initialRotHistogram->initializeHistogramVal();          //not used
    bool switch_std_dev=false;                              //not used
    
    globalOptimize globalOptimizeLoop(match_path_globalopt); //does global optimizations (i.e over short loop closures)
    SimilarityAverage similarityAverageLoop(&globalOptimizeLoop, similarityaverage_path); //not used
    
    int waitFrameCount=0;
    int num_keyframe=0;
    float seeds_num;
    int max_frame_counter=no_of_frames; //maximum frames to run pose est.
    
    
    //------------ ENTER FRAME LOOP ------------

    float new_world[3]={0,0,0};
    int frame_counter;

    for (frame_counter = 1; frame_counter <= max_frame_counter; frame_counter++)
    {

        // if(util::FLAG_ALTERNATE_GN_RA)
        //     printFlags();
        
        // start = clock()
        
        // to measure execution time
        util::measureTime time_main; 
        time_main.startTimeMeasure();
        
        // push input frame in vector
        frameptr_vector.push_back(new frame(bgr_capture)); 
        
        float initial_pose[3];
        
        // if flag set, initializes pose to be used in GN from text file
        if(util::FLAG_INITIALIZE_NONZERO_POSE) 
        {
            int temp_frame_no;
            int temp_frame_no2;
            float temp_seeds;
            float temp_scale;
            //float temp_pose;
            initialize_pose_file>>temp_frame_no >> initial_pose[0] >> initial_pose[1] >> initial_pose[2];
            
            if(util::FLAG_CONCATENATE_INITIAL_POSE)
            {
                    float concatenated_initial_pose[3]={0,0,0};
                    frameptr_vector.back()->concatenateRelativePose(initial_pose, new_world, concatenated_initial_pose);
                    
                    //copy only concatenated rotations to initial pose
                    initial_pose[2]=concatenated_initial_pose[2];
            }
            
            //   printf("\n\nINITIAL ABS pose: %f, %f, %f", initial_pose[0], initial_pose[1], initial_pose[2]);
        }
    
        // to bootstrap the first image in the sequence
        if(frame_counter==1)
        {
            activeKeyFrame=frameptr_vector.back(); //first active keyframe is first frame
            num_keyframe++;
            currentDepthMap->formDepthMap(frameptr_vector.back()); //does random initialization
            currentDepthMap->updateDepthImage();
            
            // // display kf
            // imshow("First Keyframe", activeKeyFrame->image);
            // waitKey(2000);
            // globalOptimizeLoop.pushToArray(activeKeyFrame, currentDepthMap);

            continue;
        }
        
        // not used
        // to re-estimate only at keyframes, since relative poses between frame and kf will remain unchanged
        if(util::FLAG_REPLICATE_POSE_ESTIMATION)
        {
            if(frame_counter%util::KEYFRAME_PROPAGATE_INTERVAL!=0)
            {   
                continue;
            }
        }
        
        // connection recovery mechanism which finds new depth map from history 
        // in the event that current depth has few/no seeds left
        // checks each incoming frame with history of frames
        // if match found then tries to estimate pose using saved depth map and 
        // then propagates depth map to check if seeds are above threshold
        // if no match found, then drops this frame
        
        // check if connection present between keyframe and current frame
        // set flag to use this
        if(util::FLAG_RESTORE_CONNECTION)
        {   
            globalOptimizeLoop.checkConnection(currentDepthMap);
        
            // find new connection if connection is lost
            if(globalOptimizeLoop.connectionLost==true)
            {
                
                // searches loop array for new connection
                globalOptimizeLoop.findConnection(frameptr_vector.back());
                
                // new connection found!
                if(globalOptimizeLoop.connectionLost==false)
                {
                    printf("\nConnection has been FOUND! Updating variables!");
                    delete currentDepthMap; //delete existing current depth map
                
                    currentDepthMap=new depthMap(*globalOptimizeLoop.temp_depthMap); //re-initialize to new depth map
                    
                    //update pointers, not sure if this is needed
                    currentDepthMap->depthvararrptr[0]=currentDepthMap->depthvararrpyr0;
                    currentDepthMap->depthvararrptr[1]=currentDepthMap->depthvararrpyr1;
                    currentDepthMap->depthvararrptr[2]=currentDepthMap->depthvararrpyr2;
                    currentDepthMap->depthvararrptr[3]=currentDepthMap->depthvararrpyr3;
                    
                    currentDepthMap->deptharrptr[0]=currentDepthMap->deptharrpyr0;
                    currentDepthMap->deptharrptr[1]=currentDepthMap->deptharrpyr1;
                    currentDepthMap->deptharrptr[2]=currentDepthMap->deptharrpyr2;
                    currentDepthMap->deptharrptr[3]=currentDepthMap->deptharrpyr3;
                    
                    
                    seeds_num= currentDepthMap->calculate_no_of_Seeds();
                    
                    //writing new pose
                    if(util::FLAG_WRITE_ORIG_POSES)
                    {
                        //writing orig poses for normal keyframe sequence
                        if (pose_file_orig.is_open())
                        {
                            pose_file_orig<<(frameptr_vector.back()->frameId+util::BATCH_START_ID-1)<<" "<<(activeKeyFrame->frameId+util::BATCH_START_ID-1)<<" "<<frameptr_vector.back()->poseWrtWorld[0]<<" "<<frameptr_vector.back()->poseWrtWorld[1]<<" "<<frameptr_vector.back()->poseWrtWorld[2]<<" "<<activeKeyFrame->rescaleFactor<<" "<<seeds_num<<"\n";  
                        }
                    }
                    
                    if(util::FLAG_WRITE_MATCH_POSES)
                    {
                        //only keyframes
                        if(frameptr_vector.back()->frameId%util::KEYFRAME_PROPAGATE_INTERVAL==0)
                        {
                            //pose wrt origin ->frame id, kf id
                            if(match_file.is_open())
                            {
                                match_file<<(frameptr_vector.back()->frameId+util::BATCH_START_ID-1)<<" "<<(activeKeyFrame->frameId+util::BATCH_START_ID-1)<<" "<<frameptr_vector.back()->poseWrtOrigin[0]<<" "<<frameptr_vector.back()->poseWrtOrigin[1]<<" "<<frameptr_vector.back()->poseWrtOrigin[2]<<" "<<frameptr_vector.back()->poseWrtOrigin[3]<<" "<<frameptr_vector.back()->poseWrtOrigin[4]<<" "<<frameptr_vector.back()->poseWrtOrigin[5]<<" "<<activeKeyFrame->rescaleFactor<<" "<<seeds_num<<" "<<"0"<<" "<<"0"<<" "<<"0"<<"\n";
                            }
                        }
                    }
                    
                    // updating keyframe pointers
                    currentDepthMap->keyFrame = frameptr_vector.back(); //change keyframe
                    activeKeyFrame=frameptr_vector.back();
                    
                    delete globalOptimizeLoop.temp_depthMap;
                    globalOptimizeLoop.temp_depthMap=NULL;
                    
                    printf("\nAsking for new frame");
                    continue; //ask for new frame to be mapped on newly created keyframe
                }
                
                //if no new connection has been found, go to next frame
                printf("\nConection still Lost");
                delete frameptr_vector.back(); //delete this frame, no use
                frameptr_vector.erase(frameptr_vector.begin()+frameptr_vector.size()-1);
                printf("\nAsking for new frame");
                continue;
            }
        }
        

       //control reaches here only when flag_restore_connection is off OR new connection has been found, current depth map propagated to non-zero seeds and active keyframe updated
       
       //function that estimates pose of current frame w.r.t active keyframe
       pose_vec = GetImagePoseEstimate(activeKeyFrame, frameptr_vector.back(), frame_counter, currentDepthMap, frameptr_vector[frameptr_vector.size()-2], initial_pose);
        
        //not useful
        if(util::FLAG_CONCATENATE_INITIAL_POSE)
        {
            if(frame_counter%util::CONCATENATE_STEP==0)
            {
                printf("\nUpdating New World!");
                //re-initialize with new world pose
                new_world[0]=frameptr_vector.back()->poseWrtWorld[0];
                new_world[1]=frameptr_vector.back()->poseWrtWorld[1];
                new_world[2]=frameptr_vector.back()->poseWrtWorld[2];
            }
        }
        
        
        //printf("\nFINAL ABS pose: %f, %f, %f, %f, %f, %f", frameptr_vector.back()->poseWrtWorld[0],frameptr_vector.back()->poseWrtWorld[1], frameptr_vector.back()->poseWrtWorld[2], frameptr_vector.back()->poseWrtWorld[3], frameptr_vector.back()->poseWrtWorld[4],frameptr_vector.back()->poseWrtWorld[5]);
        
        //rough calculation of newly estimated pose from initial non-zero pose
        //not sure if this can be done in lie algebra domain
        if(util::FLAG_INITIALIZE_NONZERO_POSE)
        {
            printf("\n\nFINAL Perc Change: %f, %f, %f", (abs(initial_pose[0]-frameptr_vector.back()->poseWrtWorld[0])/abs(initial_pose[0]))*100, (abs(initial_pose[1]-frameptr_vector.back()->poseWrtWorld[1])/abs(initial_pose[1]))*100, (abs(initial_pose[2]-frameptr_vector.back()->poseWrtWorld[2])/abs(initial_pose[2]))*100);
        }
               
        
        //plotViewVector(frameptr_vector.back()->poseWrtWorld);
        
        //calculating norm of translation
        //frameptr_vector.back()->calculateRelativeRandT(frameptr_vector.back()->poseWrtOrigin);
        //printf("\n\n>>translation norm: %f", frameptr_vector.back()->SE3_T.norm());
        
        //globalOptimizeLoop.triggerRotation(frameptr_vector.back());


        //print poses
        seeds_num= currentDepthMap->calculate_no_of_Seeds();
        cout << "\n" <<(frameptr_vector.back()->frameId+util::BATCH_START_ID-1)<<" "<<(activeKeyFrame->frameId+util::BATCH_START_ID-1)<<" "<<frameptr_vector.back()->poseWrtWorld<<" "<<activeKeyFrame->rescaleFactor<<" "<<seeds_num;
        
        //no longer used
        // PUSH VALUES IN HISTOGRAM TILL ITS COMPLETE
        if(!initialRotHistogram->isComplete)
            initialRotHistogram->addToHistogram(frameptr_vector.back()->poseWrtWorld[0]-frameptr_vector[frameptr_vector.size()-2]->poseWrtWorld[0],frameptr_vector.back()->poseWrtWorld[1]-frameptr_vector[frameptr_vector.size()-2]->poseWrtWorld[1],frameptr_vector.back()->poseWrtWorld[2]-frameptr_vector[frameptr_vector.size()-2]->poseWrtWorld[2]);

        
        //writing newly calculated poses
        if(util::FLAG_WRITE_ORIG_POSES)
        {
            //writing orig poses (w.r.t world) for normal keyframe sequence
            if (pose_file_orig.is_open())
            {
                pose_file_orig<<(frameptr_vector.back()->frameId+util::BATCH_START_ID-1)<<" "<<(activeKeyFrame->frameId+util::BATCH_START_ID-1)<<" "<<frameptr_vector.back()->poseWrtWorld[0]<<" "<<frameptr_vector.back()->poseWrtWorld[1]<<" "<<frameptr_vector.back()->poseWrtWorld[2]<<" "<<activeKeyFrame->rescaleFactor<<" "<<seeds_num<<"\n";
                
            }
        }
    
        if(util::FLAG_WRITE_MATCH_POSES)
        {
            //pose wrt origin (i.e. kf) ->frame id, kf id
            if(match_file.is_open())
            {
                match_file<<(frameptr_vector.back()->frameId+util::BATCH_START_ID-1)<<" "<<(activeKeyFrame->frameId+util::BATCH_START_ID-1)<<" "<<frameptr_vector.back()->poseWrtOrigin[0]<<" "<<frameptr_vector.back()->poseWrtOrigin[1]<<" "<<frameptr_vector.back()->poseWrtOrigin[2]<<" "<<frameptr_vector.back()->poseWrtOrigin[3]<<" "<<frameptr_vector.back()->poseWrtOrigin[4]<<" "<<frameptr_vector.back()->poseWrtOrigin[5]<<" "<<activeKeyFrame->rescaleFactor<<" "<<seeds_num<<" "<<"0"<<" "<<"0"<<" "<<"0"<<"\n";
            }
        }
        
       
        //here pose estimation complete
        //now moving to depth propagation/refinement
        
       PRINTF("\n\n\nDEPTH MAP:");
       currentDepthMap->formDepthMap(frameptr_vector.back()); //updates depth map variables
        
        //not used
        if((frame_counter%util::KEYFRAME_PROPAGATE_INTERVAL!=0))
        {   
            if( util::FLAG_REPLICATE_POSE_ESTIMATION)
            {
                continue;
            }
        }
        

        // Checking for keyframe propagation
        if ( (frame_counter%util::KEYFRAME_PROPAGATE_INTERVAL==0) || (frame_counter==max_frame_counter))
        {
            if(util::FLAG_ALTERNATE_GN_RA)
            {
               updatePropagationCount();
            }
            
            //not used
            if(util::FLAG_REPLICATE_POSE_ESTIMATION)
            {
                //update active keyframes
                activeKeyFrame=frameptr_vector.back();
                currentDepthMap->keyFrame=activeKeyFrame;
                
                //delete all frames except most recent
                unsigned long size=frameptr_vector.size();
                for (unsigned long i = 0; i < size-1; ++i)
                {
                    delete frameptr_vector[i]; // Calls ~object and deallocates *tmp[i]
                }
                frameptr_vector.erase(frameptr_vector.begin(),frameptr_vector.begin()+size-1);
                
                continue;
            }
            
            
            if(util::FLAG_DO_CONST_WEIGHT_POSE_ESTIMATION) //save weights of keyframe to be used in extra match pose est.
            {
                activeKeyFrame->finaliseWeights();
            }
            
            currentDepthMap->finaliseKeyframe(); //finalises current depth map associated with active keyframe
            
            //do 3Dimensional Visualization
            if(util::FLAG_DO_3D_VISUALIZATION )
            {
                printf("\nVisualzing frame id: %d", activeKeyFrame->frameId );
                //Visualizer3Dim::reconstructPointCloud(activeKeyFrame->rgb_image, activeKeyFrame->depth);
            }
            
            //checks exit condition (called from main)
            if(util::FLAG_ALTERNATE_GN_RA)
            {
                printf("\nChecking exit condition from Main...");
                toExit=checkExitCondition();
                if(toExit)
                {   printf("\nDetected Exit condition in Main");
                    util::EXIT_CONDITION=true;
                   // break;
                }
            }
            
            if(util::FLAG_DO_LOOP_CLOSURE)
            {
                if(globalOptimizeLoop.connectionLost==true)
                {
                    globalOptimizeLoop.findConnection(frameptr_vector.back());
                }
                else
                {
                    //pushing active keyframe
                    //within this, it detects loop closure and find redundant poses
                    globalOptimizeLoop.pushToArray(activeKeyFrame, currentDepthMap); //pushes active keyframe
        
                    //no longer used
                    if(util::FLAG_DO_SIMILARITY_AVERAGE)
                    {
                    //checking if any center to finalise sim avg parameters
                        
                        if(globalOptimizeLoop.simavg_center!= -1)
                        {
                            similarityAverageLoop.finaliseSimilarityAverageParams(globalOptimizeLoop.simavg_center);
                            
                            //writing to text file
                            if(util::FLAG_WRITE_SIM_AVG_VARS)
                            {
                            
                            /*
                            if (similarityavg_file.is_open())
                            {
                               
                                //format of text file
                                //sim_avg_center_id no_of_combinations
                                //node_frame_id_J node_frame_id_K Djk_value
                                //......no_of_combination lines for each sim_avg_center
                                
                               // cout<<"\n Printing similarity avg Djks";
                                similarityavg_file <<globalOptimizeLoop.loopFrameArray[globalOptimizeLoop.simavg_center].this_frame->frameId<<" "<<similarityAverageLoop.graphCenterArchive[globalOptimizeLoop.simavg_center].Djk.size()<<"\n";
                                
                                for(int i=0;i<similarityAverageLoop.graphCenterArchive[globalOptimizeLoop.simavg_center].Djk.size();i++)
                                {
                                    similarityavg_file<<globalOptimizeLoop.loopFrameArray[get<0>(similarityAverageLoop.graphCenterArchive[globalOptimizeLoop.simavg_center].Djk[i])].this_frame->frameId
                                    <<" "<<globalOptimizeLoop.loopFrameArray[get<1>(similarityAverageLoop.graphCenterArchive[globalOptimizeLoop.simavg_center].Djk[i])].this_frame->frameId<<" "<<get<2>(similarityAverageLoop.graphCenterArchive[globalOptimizeLoop.simavg_center].Djk[i])<<"\n" ;
                                }
                            }
                            */
                                
                            }
                            
                            //resetting center id to clear free vector memory
                            similarityAverageLoop.resetArrayElement(globalOptimizeLoop.simavg_center);
                            
                        }
                    }
                }
            }
            
            if(util::FLAG_ALTERNATE_GN_RA && util::EXIT_CONDITION)
            {
                if(globalOptimizeLoop.loopClosureTerminated)
                    printf("\nLoop Closure Terminated. Returned in main.");
                else
                    printf("\nExit detected Again in Main");
                break;
            }
            
            //all computations on active keyframe has been completed, so now make new active keyframe and propagate depth map
            currentDepthMap->createKeyFrame(frameptr_vector.back());
            activeKeyFrame=frameptr_vector.back();
            
            
            //delete frames, not keyframe
            /*
            num_keyframe++;
            unsigned long size=frameptr_vector.size();
            for (unsigned long i = num_keyframe-1; i < size-1; ++i)
            {
                delete frameptr_vector  [i]; // Calls ~object and deallocates *tmp[i]
            }
            frameptr_vector.erase(frameptr_vector.begin()+num_keyframe-1,frameptr_vector.begin()+size-1);
            */
            
            //delete all frames except most recent
            unsigned long size=frameptr_vector.size();
            for (unsigned long i = 0; i < size-1; ++i)
            {
                delete frameptr_vector[i]; // Calls ~object and deallocates *tmp[i]
            }
            frameptr_vector.erase(frameptr_vector.begin(),frameptr_vector.begin()+size-1);
            
            //time measure
            float elapsed=time_main.stopTimeMeasure();
            printf("\nKEYFRAME TIME! Current Frame no: %d, do parallel depth: %d , do parallel pose: %d, Time: %f \n", frame_counter, util::FLAG_DO_PARALLEL_DEPTH_ESTIMATION, util::FLAG_DO_PARALLEL_POSE_ESTIMATION, elapsed);
            
            continue;
            //waitKey(0);
        }
        
        //control reaches here when there is no keyframe propagation
        //does depth refinement and then updates associated depth matrix of active keyframe with newly refined depth
        currentDepthMap->updateKeyFrame();
        currentDepthMap->observeDepthRowParallel();
        currentDepthMap->doRegularization();
        currentDepthMap->updateDepthImage();
        
        //time measure
        float elapsed=time_main.stopTimeMeasure();
        //printf("\nCurrent Frame no: %d, do parallel depth: %d , do parallel pose: %d, Time: %f \n", frame_counter, util::FLAG_DO_PARALLEL_DEPTH_ESTIMATION, util::FLAG_DO_PARALLEL_POSE_ESTIMATION, elapsed);
        
        //end = clock();
        //cout <<"\nTime: "<<float( end - start) / CLOCKS_PER_SEC<<endl;
        
    } //exit frame loop
    
    
    //no longer used
    if(util::FLAG_DO_SIMILARITY_AVERAGE && util::FLAG_DO_LOOP_CLOSURE)
    {
        //calculating all remaining matches, Djks and clearing array memories
        printf("\nClearing array");
        similarityAverageLoop.clearGraphArray();
        
    }

    return 0;
} //exit main








