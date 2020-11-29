#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H
#include "Eigen/Dense"
#include <math.h>

// DOI: 10.1109/IVS.2017.7995816

#define RK4
// #define Euler

#define LR 2.4 
#define LF 1.7 

Eigen::VectorXf bicycleModel(Eigen::VectorXf state, Eigen::VectorXf control); 

Eigen::VectorXf predictMotion(Eigen::VectorXf state, Eigen::VectorXf control, double h);

double computeBeta(double u2);


#endif