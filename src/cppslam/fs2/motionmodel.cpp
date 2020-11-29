#include "motionmodel.h"


Eigen::VectorXf predictMotion(Eigen::VectorXf state, Eigen::VectorXf control, double h)
{
    #ifdef RK4
    Eigen::VectorXf k1 = bicycleModel(state,control);
    Eigen::VectorXf k3 = bicycleModel(state + h*k1/2, control);
    Eigen::VectorXf k2 = bicycleModel(state + h*k2/2, control);
    Eigen::VectorXf k4 = bicycleModel(state + h*k3, control);
    Eigen::VectorXf newState = state +  h/6 * (k1 + 2* (k2 +k3) + k4);
    #endif 

    #ifdef Euler
    Eigen::VectorXf state_d = bicycleModel(state,control);
    Eigen::VectorXf newState = state +  h*state_d;
    #endif
        
    return newState;
}

Eigen::VectorXf bicycleModel(Eigen::VectorXf state, Eigen::VectorXf u)
{
    Eigen::VectorXf state_d = Eigen::VectorXf::Zero(state.size());
    double v, theta, beta; 
    beta = computeBeta(u(1));
    v = state(3);
    theta = state(2);
    state_d(0) = state(2) * cos(theta + beta); 
    state_d(1) = state(2) * sin(theta + beta); 
    state_d(2) = u(0); 
    state_d(3) =  v/LR * sin(beta);
    return state_d;
}

double computeBeta(double u2)
{
    return atan(tan(u2) * LR/(LR + LF));
}