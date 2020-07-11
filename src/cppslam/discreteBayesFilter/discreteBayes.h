#ifndef DISCRETE_BAYES_H
#define DISCRETE_BAYES_H
#include <math.h>
/*
    This class implements a discrete bayesian filter that estimates
    the boolean state of some variable. Main intended use case is 
    filtering out type 1 and type 2 errors.  

    Ref: Probabilistic Robotics Pg 86/94: Thrun Et al 
*/
class discreteBayes{
    public:

        discreteBayes(double p_x)
        {
            /* 
            Inputs:
                p_x :Probability of the measurement given state. 
            */
           l_0 = logOdds(p_x);

           state = true;
        }
        
        /* Default Constructor */
        discreteBayes() : l_0(1.0) {}
        
        void update(bool measured, double p_measured);
        void initialize(double p_measured); 
        bool getState();
    
    private: 
        bool state; // state of the measured variable
        double l_0; // initial log odds
        double l_t;  // updated log odds
        double logOdds(double x);
        double probFromLogOdds(double x);
};
#endif