#ifndef OBSERVATION_H
#define OBSERVATION_H
#include <math.h> 
#include "Eigen/Dense"

class Observation{
    public:
        Observation(double x, double y, int colour); 
        Eigen::Vector2d getCart(); 
        Eigen::Vector2d getPolar(); 
        double getR(); 
        double getT(); 
        
    private: 
        double x_pos; 
        double y_pos; 
        double r; 
        double theta; 
        int colour; 
    };
#endif
