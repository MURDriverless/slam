#ifndef DATA_ASSOCIATION_H
#define DATA_ASSOCIATION_H

#include <Eigen/Dense>
#include <math.h>
#include "../point/point.h"

class dataAssociation(){
    public: 
        dataAssociation(point z_t, Eigen::MatrixXf x_t, int state_size, int lm_size)
        {
            N_t = x_t.size();
            STATE_SIZE = state_size; 
            LM_SIZE = lm_size;

            return;
        }
    private:
        int N_t;
        int STATE_SIZE; 
        int LM_SIZE; 

        Eigen::MatrixXf genF(int idx);

};
#endif