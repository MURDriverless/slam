#ifndef DISCRETE_BAYES_CPP
#define DISCRETE_BAYES_CPP

#include "discreteBayes.h"

void discreteBayes::update(bool measured, double p_measured){
    /*
    Inputs:
        measured: whether the 'thing' was measured. 
        p_measured: (Probability that it exists given the measurement)
     */
    double x_z = measured ? p_measured:1-p_measured;

    l_t = logOdds(x_z) - l_0 + l_t; 
    
    if (l_t>1.0){
        state = true;
    }
    else {
        state = false;
    }
    return; 
}

void discreteBayes::initialize(double p_measured){
    return; 
}
double discreteBayes::logOdds(double x){
    return log(x)/(1.0-log(x)); 
}
double discreteBayes::probFromLogOdds(double lt){
    return 1.0/(1.0 + exp(lt));
}
bool discreteBayes::getState()
{
    return state; 
}
#endif