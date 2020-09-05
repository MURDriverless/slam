#ifndef DISCRETE_BAYES_CPP
#define DISCRETE_BAYES_CPP

#include "discreteBayes.h"

void discreteBayes::update_measurement(int index, int measurement)
{
    // check if index is a new cone)
    int length = state.size();
    // printf("Index: %d\n", index);
    
    if ((index+1) > length){
        // new landmark do conservative resizing
        state.conservativeResizeLike(Eigen::MatrixXf::Zero(index+1,1));
        estimates.conservativeResizeLike(Eigen::MatrixXf::Zero(index+1,CONE_VARIETIES));
        state(index,0) = UNKNOWN;
        estimates(index,BLUE) =  BLUE_PROB;
        estimates(index,YELLOW) =  YELLOW_PROB;
        estimates(index,ORANGE) =  ORANGE_PROB;
    }
    if (measurement == UNKNOWN) return;
    for (int j = 0; j<CONE_VARIETIES; j++){
        if (measurement==j){
            // printf("Updating estimates at index (%d, %d)\n", index, j);
            estimates(index,j) +=  L_PROB_READING_T ;
        }
        else{
            estimates(index,j) +=  L_PROB_READING_F; 
        }
    }
    update_state();
}
void discreteBayes::update_state()
{
    int length = state.size();
    double val; 
    for (int i = 0; i < length; i++){
        int min_indx = CONE_VARIETIES; 
        double min_val = LARGE_NEGATIVE_NUMBER; 
        for(int j = 0; j<CONE_VARIETIES; j++){
            val = estimates(i,j)+ colour_prob[j];
            if (val > min_val){
                min_val = estimates(i,j); 
                min_indx = j;
            }
        }
        state(i,0) = min_indx;
    }
    // ROS_INFO_STREAM("ESTIMATES");
    // printEigenMatrix(estimates);
    // renormalize_est();
    return;
}
void discreteBayes::renormalize_est(){
    double sum; 
    for (int i = 0; i<state.size();i++){
        sum = 0.0;
        for (int j = 0; j<CONE_VARIETIES; j++){
            sum += estimates(i,j);
        }
        for (int j = 0; j<CONE_VARIETIES; j++){
            estimates(i,j) /= sum;
        }
    }
}
void printEigenMatrix(Eigen::MatrixXf mat){
	printf("\n\n");
	if (mat.rows() == 0){
		ROS_WARN("ARRAY is empty");
	}
	int rows = mat.rows(); 
	int cols = mat.cols(); 
	printf("Rows: %d || Cols: %d \n\n", rows, cols);
	printf("[");
	for (int i = 0 ; i<mat.rows() ;i++){
			printf("[");
		for (int j = 0; j< mat.cols(); j++){
			printf("%f ,", mat(i,j));
		}
		printf("]\n");
	}
	printf("]\n\n");
	return;
}
#endif