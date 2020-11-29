#ifndef OBSERVATION_CPP
#define OBSERVATION_CPP
#include "Observation.h"
Observation::Observation(double x, double y, int colour_detected)
{   
    x_pos = x; 
    y_pos = y; 
    colour = colour_detected;
    r = sqrt(x_pos*x_pos + y_pos*y_pos);
    theta = atan2(y_pos, x_pos); 
} 
Eigen::Vector2d Observation::getCart()
{
    Eigen::Vector2d ret; 
    ret << x_pos, y_pos; 
    return ret;
}
Eigen::Vector2d Observation::getPolar()
{
    Eigen::Vector2d ret;
    r = sqrt(x_pos*x_pos + y_pos*y_pos);
    theta = atan2(y_pos, x_pos); 
    ret << r, theta; 
    return ret;
}
double Observation::getR(){
    return r;
}
double Observation::getT(){
    return theta;
}

#endif
