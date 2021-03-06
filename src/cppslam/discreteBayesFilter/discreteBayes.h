#ifndef DISCRETE_BAYES_H
#define DISCRETE_BAYES_H
#include <math.h>
#include <Eigen/Dense>
#include "ros/ros.h"
/*
    This class implements a discrete bayesian filter that estimates
    the colour of a cone given a series of readings. Main intended use case is 
    filtering out type 1 and type 2 errors.  

    Ref: Probabilistic Robotics Pg 86/94: Thrun Et al 
*/

#define CONE_VARIETIES 4
#define LARGE_NEGATIVE_NUMBER -100000000000000
#define BLUE_PROB log(0.5)
#define YELLOW_PROB log(0.45)
#define ORANGE_PROB log(0.025)

#define PROB_READING_T 0.99
#define PROB_READING_F 1 - PROB_READING_T

// log definitions

#define L_PROB_READING_T log(PROB_READING_T)
#define L_PROB_READING_F log(PROB_READING_F)

#define BLUE 0
#define YELLOW 1
#define ORANGE 2
#define BIG 3 
#define UNKNOWN 4

void printEigenMatrix(Eigen::MatrixXf mat);

class discreteBayes{
    public:
        Eigen::VectorXf state; 
        discreteBayes()
        {
           estimates = Eigen::MatrixXf::Zero(1,CONE_VARIETIES);
           estimates(0,BLUE) =  BLUE_PROB;
           estimates(0,YELLOW) =  YELLOW_PROB;
           estimates(0,ORANGE) =  ORANGE_PROB;
           estimates(0,BIG) =  ORANGE_PROB;
           

           state = Eigen::MatrixXf::Zero(1,1);
           state(0,0) = UNKNOWN; // initial state is unknown
        }

        void update_measurement(int index, int measurement);
    private:
        Eigen::MatrixXf estimates; 
        void renormalize_est();
        void update_state();

        double colour_prob[4] = {BLUE_PROB,YELLOW_PROB, ORANGE_PROB,ORANGE_PROB };
};
#endif