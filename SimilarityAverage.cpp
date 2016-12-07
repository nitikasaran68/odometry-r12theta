//
//  SimilarityAverage.cpp
//  Loop_closure_sim_avg
//
//  Created by Himanshu Aggarwal on 1/16/16.
//  Copyright © 2016 Himanshu Aggarwal. All rights reserved.
//

#include "SimilarityAverage.h"
#include <tuple>

using namespace std;

//constructor
SimilarityAverage::SimilarityAverage(globalOptimize* dummyGlobalOptimize,  string write_sim_params_path)
{
    aliasGlobalOptimize=dummyGlobalOptimize;
    write_sim_params_file.open(write_sim_params_path);

}

//finalise all parameters associated with a center node
void SimilarityAverage::finaliseSimilarityAverageParams(int centerKfArrayID)
{
    printf("\n\nFinalising similarity average parameter for center id: %d with frame id: %d", centerKfArrayID, aliasGlobalOptimize->loopFrameArray[centerKfArrayID].frameId);
    
    //printVars(centerKfArrayID);
    
    graphCenterArchive[centerKfArrayID].warpingCenter=centerKfArrayID; //stores center array id
    graphCenterArchive[centerKfArrayID].frameId=aliasGlobalOptimize->loopFrameArray[centerKfArrayID].frameId; //stores frame id of center
    
    determineCombinations(centerKfArrayID);
    fillUpDjks(centerKfArrayID);
    
    graphCenterArchive[centerKfArrayID].isValid=true;
    
    printVars(centerKfArrayID);
    
}

//determine all possible (non-redunant) combination of nodes from matched frame ids
void SimilarityAverage::determineCombinations(int centerKfArrayID)
{

    //int index=centerArrayKfID/util::KEYFRAME_PROPAGATE_INTERVAL;

    for (int i=0; i<graphCenterArchive[centerKfArrayID].matchedArrayId.size(); i++)
    {
        for (int j=i+1; j<graphCenterArchive[centerKfArrayID].matchedArrayId.size(); j++)
        {
            tuple<int,int,float> pair(graphCenterArchive[centerKfArrayID].matchedArrayId[i],graphCenterArchive[centerKfArrayID].matchedArrayId[j],0.0f);
            
            graphCenterArchive[centerKfArrayID].Djk.push_back(pair);
            
        }
    }
}

//fill up median Djks of all node pairs
void SimilarityAverage::fillUpDjks(int centerKfArrayID){
    
    //int index=expiringKfID/util::KEYFRAME_PROPAGATE_INTERVAL;
    //printf(" \n Filling up Djks for center array id %d ",centerKfArrayID);
    //calculate warped depth of all matched nodes for a center
    for(int i=0; i<graphCenterArchive[centerKfArrayID].matchedArrayId.size(); i++)
    {
        //store loop frame array id pairs
        
        //calculateWarpedDepthMap(graphCenterArchive[centerKfArrayID].matchedArrayId[i], centerKfArrayID, graphCenterArchive[centerKfArrayID].matchedRelativePoses[i]);
        
        findDepthByTriangulation(centerKfArrayID, graphCenterArchive[centerKfArrayID].matchedArrayId[i]);
        
    }
    
    //fill up Djks for each combination of nodes (j,k)
    for (int i=0; i<graphCenterArchive[centerKfArrayID].Djk.size(); i++)
    {
        
        get<2>(graphCenterArchive[centerKfArrayID].Djk[i])=calculateDjks( get<0>(graphCenterArchive[centerKfArrayID].Djk[i]),get<1>(graphCenterArchive[centerKfArrayID].Djk[i]));
        
        //Djk invalid=-1.0f
        //Dkl left uncalculated=0.0f
        //Djk valid>0.0f
        
    }

}


void SimilarityAverage::fillPoseAndMatches(int loopMatchedID, int currentArrayId,float* relPose)
{
    //printf("\nFilling current-match pair with frame id: (%d, %d) and array id: (%d, %d)", aliasGlobalOptimize->loopFrameArray[currentArrayId].frameId, aliasGlobalOptimize->loopFrameArray[loopMatchedID].frameId, currentArrayId, loopMatchedID);
    //Taking center=currentArrayId ; matchedId=loopMatchedId
    graphCenterArchive[currentArrayId].matchedArrayId.push_back(loopMatchedID);
    tuple<float,float,float,float,float,float> relPoseTuple(relPose[0],relPose[1],relPose[2],relPose[3],relPose[4],relPose[5]);
    graphCenterArchive[currentArrayId].matchedRelativePoses.push_back(relPoseTuple);
    
    if(currentArrayId>loopMatchedID)
    {
    //Taking center=loopMatchedId ; matchedId=currentArrayId
    graphCenterArchive[loopMatchedID].matchedArrayId.push_back(currentArrayId);
    tuple<float,float,float,float,float,float> invRelPoseTuple(-relPose[0],-relPose[1],-relPose[2],-relPose[3],-relPose[4],-relPose[5]);
    graphCenterArchive[loopMatchedID].matchedRelativePoses.push_back(invRelPoseTuple);
    }
    
}

void SimilarityAverage::resetArrayElement(int arrayID)
{
 
    if(util::FLAG_WRITE_SIM_AVG_VARS)
    {
        
        if (write_sim_params_file.is_open())
        {
            
            //format of text file
            //sim_avg_center_id
            //no_of_matches
            //match_frame_id1 match_frame_id2.....
            //no_of_combinations
            //node_frame_id_J node_frame_id_K Djk_value
            //......no_of_combination lines for each sim_avg_center
            
            // cout<<"\n Printing similarity avg Djks";
            
            write_sim_params_file<<aliasGlobalOptimize->loopFrameArray[arrayID].this_frame->frameId<<"\n";
            write_sim_params_file<<graphCenterArchive[arrayID].matchedArrayId.size()<<"\n";
            
            for(int i=0; i<graphCenterArchive[arrayID].matchedArrayId.size(); i++)
            {
                write_sim_params_file<<aliasGlobalOptimize->loopFrameArray[graphCenterArchive[arrayID].matchedArrayId[i]].this_frame->frameId<<" ";
            }
        
            write_sim_params_file<<"\n"<<graphCenterArchive[arrayID].Djk.size()<<"\n";
            
            for(int i=0;i<graphCenterArchive[arrayID].Djk.size();i++)
            {
                write_sim_params_file<<aliasGlobalOptimize->loopFrameArray[get<0>(graphCenterArchive[arrayID].Djk[i])].this_frame->frameId
                <<" "<<aliasGlobalOptimize->loopFrameArray[get<1>(graphCenterArchive[arrayID].Djk[i])].this_frame->frameId
                <<" "<<get<2>(graphCenterArchive[arrayID].Djk[i])<<"\n";
            }
        }
        
    }
    
    
    
    
    graphCenterArchive[arrayID].matchedArrayId.clear();
    //printf("\n!!!Size of matched array of array Id %d is = %lu", arrayID, graphCenterArchive[arrayID].matchedArrayId.size());
     graphCenterArchive[arrayID].matchedRelativePoses.clear();
     graphCenterArchive[arrayID].Djk.clear();
     graphCenterArchive[arrayID].frameId=-1;
     graphCenterArchive[arrayID].isValid=false;
     graphCenterArchive[arrayID].warpingCenter=-1;
}

//calculate warped depth map of matched frame w.r.t center node
void SimilarityAverage::calculateWarpedDepthMap( int matchID,int centerID,tuple<float,float,float,float,float,float> relPose)
{
    //printf("\nWarping depthmap from prev frame Id: %d to current frame Id: %d", aliasGlobalOptimize->loopFrameArray[matchID].frameId,aliasGlobalOptimize->loopFrameArray[centerID].frameId);
    
    frame* new_keyframe=aliasGlobalOptimize->loopFrameArray[centerID].this_frame;
    frame* keyFrame=aliasGlobalOptimize->loopFrameArray[matchID].this_frame;
    
    // wipe depthmap
   for(depthhypothesis* pt = aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap-> otherDepthHypothesis+util::ORIG_COLS*util::ORIG_ROWS-1; pt >= aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap-> otherDepthHypothesis; pt--)
    {
        //PRINTF("%f",pt->invDepth);
        pt->isValid = false;
        pt->blacklisted = 0;
    }
    
    
    // re-usable values.
    // SE3 oldToNew_SE3 = se3FromSim3(new_keyframe->pose->thisToParent_raw).inverse();
    
    float relPoseArray[]={get<0>(relPose),get<1>(relPose),get<2>(relPose),get<3>(relPose),get<4>(relPose),get<5>(relPose)};
    
    keyFrame->calculateRelativeRandT(relPoseArray);// parent id to be checked
    Eigen::MatrixXf oldToNew_SE3=keyFrame->SE3_Pose; //New Wrt Old
    Eigen::VectorXf trafoInv_t=keyFrame->SE3_T;
    Eigen::MatrixXf trafoInv_R=keyFrame->SE3_R;
    
    
    float* newKFMaxGrad_ptr = new_keyframe->maxAbsGradient.ptr<float>(0);
    uchar* sourceImg_ptr=keyFrame->image.ptr<uchar>(0);
    
    // go through all pixels of OLD image, propagating forwards.
    for(int y=0;y<util::ORIG_ROWS;y++)
    {
        newKFMaxGrad_ptr=new_keyframe->maxAbsGradient.ptr<float>(y);
        sourceImg_ptr=keyFrame->image.ptr<uchar>(y);
        
        for(int x=0;x<util::ORIG_COLS;x++)
        {
            depthhypothesis* source = aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap->currentDepthHypothesis + x + y*util::ORIG_COLS;
            
            if(!source->isValid)
            {
                // PRINTF("\nNot Valid!!!");
                continue;
            }
            
           // if(source->invDepth<0 || source->invDepthSmoothed<0)
            //{
              //  printf("\nx: %d, y: %d, valid source_idepth: %f, valid source_idepth_smooth: %f", x, y, source->invDepth, source->invDepthSmoothed);
                //waitKey(0);
            //}
            
            //if(enablePrintDebugInfo) runningStats.num_prop_attempts++;
            
            Eigen::Vector3f pn = (trafoInv_R * Eigen::Vector3f(x*util::ORIG_FX_INV + util::ORIG_CX_INV,y*util::ORIG_FY_INV + util::ORIG_CY_INV,1.0f)) / source->invDepthSmoothed + trafoInv_t;
            
            
            float new_idepth = 1.0f / pn[2];
        
            
            //if(new_idepth<0)
            //{  /*  printf("\nx: %d, y: %d, new_idepth: %f, source_invdepth: %f, source_invdepth_smooth: %f", x, y, new_idepth, source->invDepth, source->invDepthSmoothed);
                //cout<<"\ntrafoInv_R: \n"<<trafoInv_R;
                //cout<<"\nDeterminant of trafoInv_R: "<<trafoInv_R.determinant();
                //cout<<"\ntrafoInv_t: \n"<<trafoInv_t;
                //cout<<"\n pn: \n"<<pn;
                //cout<<"\n (trafoInv_R * Eigen::Vector3f(x*util::ORIG_FX_INV + util::ORIG_CX_INV,y*util::ORIG_FY_INV + util::ORIG_CY_INV,1.0f)): "<<(trafoInv_R * Eigen::Vector3f(x*util::ORIG_FX_INV + util::ORIG_CX_INV,y*util::ORIG_FY_INV + util::ORIG_CY_INV,1.0f));
               // */
                
               // printf("\nnew idepth: %f ", new_idepth);
                
                //waitKey(0);
           // }
            
            //PRINTF("\nSource inverse depth : %f",source->invDepth);
            float u_new = pn[0]*new_idepth*util::ORIG_FX + util::ORIG_CX;
            float v_new = pn[1]*new_idepth*util::ORIG_FY + util::ORIG_CY;
            
            
            // check if still within image, if not: DROP.
            if(!(u_new > 2.1f && v_new > 2.1f && u_new < util::ORIG_COLS-3.1f && v_new < util::ORIG_ROWS-3.1f))
            {
                
                continue;
            }
            
            int newIDX = (int)(u_new+0.5f) + ((int)(v_new+0.5f))*util::ORIG_COLS; //sub pixel ?
            float destAbsGrad = newKFMaxGrad_ptr[x];
            
            
            float sourceColor =sourceImg_ptr[x];
            float destColor = new_keyframe->getInterpolatedElement(u_new, v_new);
            float residual = destColor - sourceColor;
            
            if(residual*residual / (util::MAX_DIFF_CONSTANT + util::MAX_DIFF_GRAD_MULT*destAbsGrad*destAbsGrad) > 1.0f || destAbsGrad < util::MIN_ABS_GRAD_DECREASE)
            {
                //if(enablePrintDebugInfo) runningStats.num_prop_removed_colorDiff++;
                //PRINTF("\nRemoved- Color Difference");
                continue;
            }
            
            depthhypothesis* targetBest = aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap->otherDepthHypothesis +  newIDX;
            
            // large idepth = point is near = large increase in variance.
            // small idepth = point is far = small increase in variance.
            float idepth_ratio_4 = new_idepth / source->invDepthSmoothed;
            idepth_ratio_4 *= idepth_ratio_4;
            idepth_ratio_4 *= idepth_ratio_4;
            
            float new_var =idepth_ratio_4*source->invDepth;
            
            /*
             if((source->invDepth<0 ) ||(source->invDepthSmoothed<0 ))
             {    printf("\nx: %d, y: %d, source_idepth: %f, source_idepth_smooth: %f", x, y, source->invDepth,source->invDepthSmoothed);
             waitKey(0);
             }
             */
        
            // check for occlusion
            if(targetBest->isValid)
            {
                
                
                // if they occlude one another, one gets removed.
                float diff = targetBest->invDepth - new_idepth;
                if(util::DIFF_FAC_PROP_MERGE*diff*diff > new_var + targetBest->variance)
                {
                    if(new_idepth < targetBest->invDepth)
                    {
                        //if(enablePrintDebugInfo) runningStats.num_prop_occluded++;
                        //PRINTF("\nOccluded-1");
                        continue;
                    }
                    else
                    {
                        // if(enablePrintDebugInfo) runningStats.num_prop_occluded++;
                        // PRINTF("\nOccluded-2");
                        targetBest->isValid = false;
                    }
                }
            }
            
            if(!targetBest->isValid)
            {
                //if(enablePrintDebugInfo) runningStats.num_prop_created++;
                
                  targetBest->invDepth=new_idepth;
                  targetBest->isValid=true;
                
              //  targetBest->variance=new_var;
              //  targetBest->varianceSmoothed=-1;
              //  targetBest->invDepthSmoothed=-1;
              //  targetBest->validity_counter=source->validity_counter;
              
              //  targetBest->blacklisted=0;
                // PRINTF("\nFAIL");
                
            }
            
            
            else
            {
                //if(enablePrintDebugInfo) runningStats.num_prop_merged++;
                
                // merge idepth ekf-style
                float w = new_var / (targetBest->variance + new_var);
                float merged_new_idepth = w*targetBest->invDepth + (1.0f-w)*new_idepth;
                
                // merge validity
                int merged_validity = source->validity_counter + targetBest->validity_counter;
                if(merged_validity > util::VALIDITY_COUNTER_MAX+(util::VALIDITY_COUNTER_MAX_VARIABLE))
                    merged_validity = util::VALIDITY_COUNTER_MAX+(util::VALIDITY_COUNTER_MAX_VARIABLE);
                
                float temp_var=targetBest->variance; ////!!!!
                
                targetBest->invDepth=merged_new_idepth;
                targetBest->variance= 1.0f/(1.0f/temp_var + 1.0f/new_var);  //!!!!!
                targetBest->validity_counter=merged_validity;
                // targetBest->validity_counter=source->validity_counter;
                targetBest->isValid=true;
                targetBest->blacklisted=0;
                targetBest->invDepthSmoothed=-1.0f;
                targetBest->varianceSmoothed=-1.0f;
                //PRINTF("\nVALID");
                // *targetBest = DepthMapPixelHypothesis( merged_new_idepth,1.0f/(1.0f/targetBest->idepth_var + 1.0f/new_var),merged_validity);
                //if(new_var<0 || temp_var<0)
                //    printf("\nnew var: %f, temp_var: %f", new_var, temp_var);
                
                //if(new_idepth<0)
                //    printf("\nmerged idepth: %f", merged_new_idepth);
                
                
                /*if(targetBest->invDepth<0 || targetBest->variance<0)
                 {    printf("\nx: %d, y: %d, already valid target_inv_depth: %f, var: %f", x, y, targetBest->invDepth, targetBest->variance);
                 waitKey(0);
                 }
                 */
                
            }
            
            
          //  if(targetBest->invDepth<0 && targetBest->invDepth!=-1.0f)
          //      printf("\nx: %d, y: %d, inv_depth: %f", x, y, targetBest->invDepth);
            
            
            
        }
    }
    
    // swap!
   // std::swap(currentDepthHypothesis, otherDepthHypothesis);
    
    printf("\n\n\nSIM AVG: frame center id: %d, frame match id: %d", aliasGlobalOptimize->loopFrameArray[centerID].frameId, aliasGlobalOptimize->loopFrameArray[matchID].frameId);
    printf("\nRelative Pose: %f, %f, %f, %f, %f, %f", relPoseArray[0], relPoseArray[1], relPoseArray[2],  relPoseArray[3], relPoseArray[4], relPoseArray[5]);
    
    //make inv depth as 1
    aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap->makeInvDepthOne(false);
    
  
      //display warped depth
    if(util::FLAG_DO_3D_VISUALIZATION )
    {
        displayWarpedDepth(matchID, centerID);
    }
    
    
}

//inverse input pose tuple
tuple<float,float,float,float,float,float> SimilarityAverage::inverseTuple(tuple<float, float, float, float, float, float> inputTuple){

return tuple<float,float,float,float,float,float>(-get<0>(inputTuple),-get<1>(inputTuple),-get<2>(inputTuple),-get<3>(inputTuple),-get<4>(inputTuple),-get<5>(inputTuple));

    
}

//returns median Djk for a pair of input node J and K
float SimilarityAverage::calculateDjks(int node_J_arrayid, int node_K_arrayid)
{

   // printf("\nCalculating DJks node j %d and node k %d",node_J_arrayid,node_K_arrayid);
    //condition on minimum seeds
    if((aliasGlobalOptimize->loopFrameArray[node_J_arrayid].this_currentDepthMap->calculate_no_of_Seeds(false)<util::MIN_SEEDS_FOR_DJK_CALCULATION) || (aliasGlobalOptimize->loopFrameArray[node_K_arrayid].this_currentDepthMap->calculate_no_of_Seeds(false)<util::MIN_SEEDS_FOR_DJK_CALCULATION))
    return -1.0f;
    
    vector<float> calculatedDjks;
    calculatedDjks.reserve(util::MIN_SEEDS_FOR_DJK_CALCULATION*util::ORIG_ROWS*util::ORIG_COLS/25);
    
    for(int idx=0;idx<util::ORIG_ROWS*util::ORIG_COLS;idx++)
    {

        //condition on valid points
        if((!aliasGlobalOptimize->loopFrameArray[node_J_arrayid].this_currentDepthMap-> otherDepthHypothesis[idx].isValid)||(!aliasGlobalOptimize->loopFrameArray[node_K_arrayid].this_currentDepthMap-> otherDepthHypothesis[idx].isValid))
            continue;
        
        calculatedDjks.push_back(aliasGlobalOptimize->loopFrameArray[node_K_arrayid].this_currentDepthMap-> otherDepthHypothesis[idx].invDepth / aliasGlobalOptimize->loopFrameArray[node_J_arrayid].this_currentDepthMap-> otherDepthHypothesis[idx].invDepth); //Djk=depth_j / depth_k
        
   }
    
    //sorting
    sort(calculatedDjks.begin(), calculatedDjks.end());
    
    //calculating median
    if (calculatedDjks.size()%2==0)
    {
        //for even size
        return (calculatedDjks[calculatedDjks.size()/2-1]+calculatedDjks[calculatedDjks.size()/2])/2.0f;
    }
    
    //for odd size
    return calculatedDjks[floor(calculatedDjks.size()/2.0f)];
    

}

void SimilarityAverage::printVars(int centerKfArrayID)
{
    //center node
    printf("\nPrinting results: ");
    printf("\nCentr Kf array Id: %d", centerKfArrayID);
    
    //matched frames
    printf("\nMatched frame array Id: ");
    for (int i=0; i<graphCenterArchive[centerKfArrayID].matchedArrayId.size(); i++)
    {
        printf("%d , ",graphCenterArchive[centerKfArrayID].matchedArrayId[i]);
    }
    
    printf("\nMatched corresponding frame Id: ");
    for (int i=0; i<graphCenterArchive[centerKfArrayID].matchedArrayId.size(); i++)
    {
        printf("%d , ",aliasGlobalOptimize->loopFrameArray[graphCenterArchive[centerKfArrayID].matchedArrayId[i]].frameId);
    }
    
    //node pairs and Djk values
    printf("\nDjk tuples (node J, node K, Djk):  ");
    for (int i=0; i<graphCenterArchive[centerKfArrayID].Djk.size(); i++)
    {
        
        printf("\n (%d, %d) = %f ",get<0>(graphCenterArchive[centerKfArrayID].Djk[i]),get<1>(graphCenterArchive[centerKfArrayID].Djk[i]), get<2>(graphCenterArchive[centerKfArrayID].Djk[i]));

        
    }
   

}

void SimilarityAverage::compareLoopFrameId()
{
    printf("\n\nComparing array id and frame id: arrayId, global_optimize_frameId, similarity_avgId");
    
    /*for(int i=0; i<util::MAX_LOOP_ARRAY_LENGTH_SCALE_AVG; i++)
    {
        if(aliasGlobalOptimize->loopFrameArray[i].isValid && graphCenterArchive[i].isValid)
            printf("\n %d, %d, %d ", i, aliasGlobalOptimize->loopFrameArray[i].frameId, graphCenterArchive[i].frameId);
    }*/
    
    printf("\nLoop Frame Array: \n");
    for(int i=0; i<util::MAX_LOOP_ARRAY_LENGTH_SCALE_AVG; i++)
    {
        if(aliasGlobalOptimize->loopFrameArray[i].isValid==true)
            printf("\narray id: %d, frame id: %d ,", i, aliasGlobalOptimize->loopFrameArray[i].this_frame->frameId);
        
    }
    
    printf("\nGraph Archive Array: \n");
    for(int i=0; i<util::MAX_LOOP_ARRAY_LENGTH_SCALE_AVG; i++)
    {
        if(graphCenterArchive[i].matchedArrayId.size()>0)
        {
            printf("\n(array id: %d, no_match_frames: %ld):  ,", i, graphCenterArchive[i].matchedArrayId.size());
            for(int j=0; j<graphCenterArchive[i].matchedArrayId.size();j++)
                printf(" %d, ", graphCenterArchive[i].matchedArrayId[j]);
        }
        
        
    }
    
}

void SimilarityAverage::clearGraphArray()
{
    compareLoopFrameId();
    
    //clearing graph archive array
    for(int i=0;i<util::MAX_LOOP_ARRAY_LENGTH_SCALE_AVG; i++)
    {
        if(graphCenterArchive[i].matchedArrayId.size()>0)
        {
            printf("\n\n\n!!!Clearing array id: %d, with frame id: %d!!!", i, aliasGlobalOptimize->loopFrameArray[i].this_frame->frameId);
            finaliseSimilarityAverageParams(i);
            resetArrayElement(i);
        }
    }
    
    //clearing loop frame array
    for(int i=0;i<util::MAX_LOOP_ARRAY_LENGTH_SCALE_AVG; i++)
    {
        if(aliasGlobalOptimize->loopFrameArray[i].isValid==true)
            aliasGlobalOptimize->resetArrayElement(i);
    }
}

void SimilarityAverage::displayWarpedDepth(int matchID,int centerID)
{
    //write to warped depth matrix from otherDepthHypothesis
    Mat warped_depth_mat=Mat::zeros(util::ORIG_ROWS, util::ORIG_COLS, CV_32FC1);
    
    depthhypothesis* pt_other = aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap->otherDepthHypothesis;
    float* depth_ptr= warped_depth_mat.ptr<float>(0);
    
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        depth_ptr= warped_depth_mat.ptr<float>(y);
        
        for(int x=0; x<util::ORIG_COLS; x++)
        {
            pt_other = aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap->otherDepthHypothesis + x + y*util::ORIG_COLS;
            
            if(y<3||y>=util::ORIG_ROWS-3||x<3||x>=util::ORIG_COLS-3)
            {
                continue;
            }
            
            if(pt_other->isValid==true && (pt_other->invDepthSmoothed>=-0.05f) )
            {
                depth_ptr[x]=(1/(pt_other->invDepth));
            }
            
        }
        
    }
    
    //create colored warped depth map
    
    Mat colourMap(util::ORIG_ROWS,util::ORIG_COLS,CV_8UC1);
    uchar* color_ptr=colourMap.ptr<uchar>(0);
    
    for(int y=0; y<util::ORIG_ROWS; y++ )
    {
        depth_ptr=  warped_depth_mat.ptr<float>(y);
        color_ptr=colourMap.ptr<uchar>(y);
        int j=0;
        for(int x=0; x<util::ORIG_COLS;x++)
        {
            
            color_ptr[j++]=uchar(depth_ptr[x]*100.f);//depth_ptr[x]
            
            if(depth_ptr[x]*100.f > 255)
                color_ptr[j-1]=255;
            
        }
    }
    
    Mat colourMap_rgb;
    applyColorMap(colourMap, colourMap_rgb, COLORMAP_JET);
    
    uchar* colour_rgb=colourMap_rgb.ptr<uchar>(0);
    uchar* image_ptr=aliasGlobalOptimize->loopFrameArray[centerID].this_frame->image.ptr<uchar>(0);
    uchar r,g,b;
    int j;
    
    for(int y=0;y<util::ORIG_ROWS;y++)
    {
        colour_rgb=colourMap_rgb.ptr<uchar>(y);
        image_ptr=aliasGlobalOptimize->loopFrameArray[centerID].this_frame->image.ptr<uchar>(y);
        j=0;
        
        for(int x=0;x<3*util::ORIG_COLS;x=x+3)
        {
            b=colour_rgb[x];
            g=colour_rgb[x+1];
            r=colour_rgb[x+2];
            
            if(b==128 & g==0 & r==0)
            {
                // colour_rgb[x]=0;
                colour_rgb[x]=image_ptr[j];
                colour_rgb[x+1]=image_ptr[j];
                colour_rgb[x+2]=image_ptr[j];
                
            }
            j++;
        }
    }
    
    printf("\nVisualzing depth for center id: %d by warping match id: %d", centerID, matchID );
    
    //display warped depth map
    if(util::FLAG_DISPLAY_DEPTH_MAP)
    {
        
        imshow("Warped Colour_depth",colourMap_rgb);
        //waitKey(1000);
    }
    
    //do 3Dimensional Visualization
  //  Visualizer3Dim::reconstructPointCloud(aliasGlobalOptimize->loopFrameArray[centerID].this_frame->rgb_image, warped_depth_mat);
    
    

}


void SimilarityAverage::findDepthByTriangulation(int centerID, int matchID)
{
    printf("\nFinding depth by triangulation for center id: %d using match id %d", centerID, matchID);
    
    //calculate relative poses using world pose
    float pose_match_wrt_center[6];
    
    frame* prev_frame=aliasGlobalOptimize->loopFrameArray[centerID].this_frame;
    frame* current_frame=aliasGlobalOptimize->loopFrameArray[matchID].this_frame;
    
  
    prev_frame->concatenateOriginPose(current_frame->poseWrtWorld, prev_frame->poseWrtWorld, pose_match_wrt_center);
    
    //pyramid initializations
    Pyramid workingPyramid(prev_frame, current_frame, pose_match_wrt_center, aliasGlobalOptimize->loopFrameArray[centerID].this_currentDepthMap);
    workingPyramid.pose=pose_match_wrt_center;
    
    prev_frame->updationOnPyrChange(0);
    current_frame->updationOnPyrChange(0,false);
    
    //calculating correspondences
    workingPyramid.calculateWorldPoints();
    workingPyramid.calculateWarpedPoints();
    
    //workingPyramid.warpedPoints.convertTo(workingPyramid.warpedPoints, CV_16UC1);
    
    Mat mask_filtered=prev_frame->mask.clone();
    
    uchar* mask_filtered_ptr=mask_filtered.ptr<uchar>(0);
    uchar* mask_ptr=prev_frame->mask.ptr<uchar>(0);
    float* warped_ptr_row0=workingPyramid.warpedPoints.ptr<float>(0);
    float* warped_ptr_row1=workingPyramid.warpedPoints.ptr<float>(1);
    
    int k=0;
    int filtered_non_zero=0;
    
    //creating new mask for handling oob warped points
    for(int y=0;y<util::ORIG_ROWS; y++)
    {
        mask_filtered_ptr=mask_filtered.ptr(y);
        mask_ptr=prev_frame->mask.ptr(y);
        
        for(int x=0;x<util::ORIG_COLS;x++)
        {
            if(mask_ptr[x]==0)
                continue;
            
            //cout<<"\nwarped x,y : "<<warped_ptr_row0[k]<<" , "<<warped_ptr_row1[k];
            if(ceil(warped_ptr_row0[k])<util::XMIN || ceil(warped_ptr_row0[k])>util::XMAX || ceil(warped_ptr_row1[k])<util::YMIN || ceil(warped_ptr_row1[k])>util::YMAX)
            {
                mask_filtered_ptr[x]=2;
                k=k+1;
                //cout<<"  Skipped!!!";
                continue;
            }
            
            
            filtered_non_zero++;
            k=k+1;
            
        }
    }
    
    
    int N=filtered_non_zero;
    
    Mat cam_center_pnts(1,N,CV_64FC2);
    Mat cam_match_pnts(1,N,CV_64FC2);
    
    //initializing pointers
    double* cam_center_ptr=cam_center_pnts.ptr<double>(0);
    double* cam_match_ptr=cam_match_pnts.ptr<double>(0);
    
    k=0;
    int k_cam=0;
    
    //filling mats
    for(int y=0;y<util::ORIG_ROWS;y++)
    {
        mask_filtered_ptr=mask_filtered.ptr<uchar>(y);
        
        for(int x=0;x<util::ORIG_COLS;x++)
        {
            if(mask_filtered_ptr[x]==0)
                continue;
            
            if(mask_filtered_ptr[x]==2)
            {
                k=k+1;
                continue;
            }
            
            cam_center_ptr[2*k_cam]=x;
            cam_center_ptr[2*k_cam+1]=y;
            
            cam_match_ptr[2*k_cam]=double(ceil(warped_ptr_row0[k]));
            cam_match_ptr[2*k_cam+1]=double(ceil(warped_ptr_row1[k]));
            
            
            //printf("\nCenter (%f, %f) ; match (%f. %f)", cam_center_ptr[2*k_cam], cam_center_ptr[2*k_cam+1], cam_match_ptr[2*k_cam], cam_match_ptr[2*k_cam+1]);
            
            if(cam_match_ptr[2*k_cam]<util::XMIN)
            {
                cout<<"\nwarped x: "<<warped_ptr_row0[k];
    
            }
            
            
            k=k+1;
            k_cam=k_cam+1;
            
        }
    }
    
    Mat pnts3D; //(4,N,CV_64FC1);
    
    //initializing projection matrices
    float center_pose[6]={0,0,0,0,0,0};
    prev_frame->calculateRelativeRandT(center_pose);
    current_frame->calculateRelativeRandT(pose_match_wrt_center);
    current_frame->SE3_Pose.block(0, 0, 1, 3)=current_frame->SE3_Pose.block(0, 0, 1, 3)/current_frame->SE3_Pose.block(0, 0, 1, 3).norm();
    
    //printf("\n\nNorm of translation: %f", current_frame->SE3_Pose.block(0, 0, 1, 3).norm());
    //cout<<"\nK mat: \n"<<util::K_Eigen;
    //cout<<"\nCenter pose: \n"<<prev_frame->SE3_Pose.block(0, 0, 3, 4) ;
    //cout<<"\nMatch pose: \n"<<current_frame->SE3_Pose.block(0, 0, 3, 4);
    

    Eigen::MatrixXf temp_proj_mat=(util::K_Eigen*prev_frame->SE3_Pose.block(0, 0, 3, 4));
    //Eigen::MatrixXd temp_proj_mat =temp_proj_mat_1.cast<double>();
   // cout<<"\nTemp Center pose: \n"<<temp_proj_mat;
    Mat cam_center_proj_matrix=(Mat_<float>(3,4)<<temp_proj_mat(0,0),temp_proj_mat(0,1),temp_proj_mat(0,2),temp_proj_mat(0,3),temp_proj_mat(1,0),temp_proj_mat(1,1),temp_proj_mat(1,2),temp_proj_mat(1,3),temp_proj_mat(2,0),temp_proj_mat(2,1),temp_proj_mat(2,2),temp_proj_mat(2,3));
    
    
    Eigen::MatrixXf temp_proj_mat_new=(util::K_Eigen*current_frame->SE3_Pose.block(0, 0, 3, 4));
   // cout<<"\nTemp Center pose: NEW\n"<<temp_proj_mat_new;
    Mat cam_match_proj_matrix=(Mat_<float>(3,4)<<temp_proj_mat_new(0,0),temp_proj_mat_new(0,1),temp_proj_mat_new(0,2),temp_proj_mat_new(0,3),temp_proj_mat_new(1,0),temp_proj_mat_new(1,1),temp_proj_mat_new(1,2),temp_proj_mat_new(1,3),temp_proj_mat_new(2,0),temp_proj_mat_new(2,1),temp_proj_mat_new(2,2),temp_proj_mat_new(2,3));
    
    
   // cout<<"\n\nCenter proj mat: \n"<<cam_center_proj_matrix;
   // cout<<"\n\nMatch proj mat: \n"<<cam_match_proj_matrix;
    
    
    //triangulation to get depth
    triangulatePoints(cam_center_proj_matrix, cam_match_proj_matrix, cam_center_pnts, cam_match_pnts, pnts3D);
    
   // cout<<"\nSize of pnts3D: "<<pnts3D.rows<<" "<<pnts3D.cols<<" type: "<<(pnts3D.type()==CV_64FC1);
    
    //storing calculated depth in otherdepth
    depthhypothesis* depth_dest=aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap->otherDepthHypothesis;
    double* depth_source=pnts3D.ptr<double>(2);
    double* depth_source0=pnts3D.ptr<double>(0);
    double* depth_source1=pnts3D.ptr<double>(1);
    double* depth_source3=pnts3D.ptr<double>(3);

    
    
    k=0;
    
    for(int y=0;y<util::ORIG_ROWS;y++)
    {
        mask_filtered_ptr=mask_filtered.ptr<uchar>(y);
        
        for(int x=0;x<util::ORIG_COLS;x++)
        {
            depth_dest=aliasGlobalOptimize->loopFrameArray[matchID].this_currentDepthMap->otherDepthHypothesis+x+y*util::ORIG_COLS;
            
            if(mask_filtered_ptr[x]==0 || mask_filtered_ptr[x]==2)
            {
                depth_dest->isValid=false;
                depth_dest->invDepth=-1;
                depth_dest->invDepthSmoothed=-1;
                continue;
            }
            
            depth_dest->isValid=true;
            depth_dest->invDepth=1/(depth_source[k]/depth_source3[k]);
            depth_dest->invDepthSmoothed=1/(depth_source[k]/depth_source3[k]);
            //printf("\nx: %d, y: %d, [Z]: %f", x, y, depth_source[k]/depth_source3[k]);
            
            if(depth_dest->invDepth <=0)
                printf("\n !!!!!! negative depth");
            k=k+1;
        }
        
    }
    
    cam_center_ptr=cam_center_pnts.ptr<double>(0);
    cam_match_ptr=cam_match_pnts.ptr<double>(0);
    
    /*
    for(int k=0;k<N;k++)
    {
        printf("\nCenter (%f, %f) ; match (%f. %f)", cam_center_ptr[2*k], cam_center_ptr[2*k+1], cam_match_ptr[2*k], cam_match_ptr[2*k+1]);
    }
     */
    
    
    //cout<<"\n\nCenter points: \n"<<cam_center_pnts;
    //cout<<"\n\n\nMatch points: \n"<<cam_match_pnts;
   
    
    

    
    displayWarpedDepth(matchID, centerID);
    //  imshow("MASK", mask_filtered);
    //waitKey(0);

    
}





