#ifndef DISCRETE_BAYES_CPP
#define DISCRETE_BAYES_CPP

#include "discreteBayes.h"
void discreteBayes::update_measurement(int index, int measurement)
{
    // check if index is a new cone)
    int length = state.size();


    if (index >= length){
        // new landmark do conservative resizing
        state.conservativeResizeLike(Eigen::MatrixXf::Zero(index,1));
        estimates.conservativeResizeLike(Eigen::MatrixXf::Zero(index,3));
        state(index,0) = UNKNOWN;
        estimates(index,BLUE) =  BLUE_PROB;
        estimates(index,YELLOW) =  YELLOW_PROB;
        estimates(index,ORANGE) =  ORANGE_PROB;
    }
    if (measurement == UNKNOWN) return;
    for (int j = 0; j<CONE_VARIETIES; j++){
        if (measurement==j){
            estimates(index,j) +=  L_PROB_READING_T;
        }
        else{
            estimates(index,j) +=  L_PROB_READING_F; 
        }
    }
    update_state();
}
void discreteBayes::update_state(){
    int length = state.size();

    for (int i = 0; i < length; i++){
        int min_indx = 3; 
        double min_val = LARGE_NEGATIVE_NUMBER; 
        for(int j = 0; j<CONE_VARIETIES; j++){
            if (estimates(i,j) > min_val){
                min_val = estimates(i,j); 
                min_indx = j;
            }
        }
        state(i,0) = min_indx;
    }
    return;
}
#endif