#include "cone.h"

cone::cone(double x_new, double y_new, int colour_new)
{
    x = x_new; 
    y = y_new; 
    colour = colour_new; 
}
void cone::set_cov(Eigen::Matrix2d new_cov)
{
    cov = new_cov;
}

Eigen::Matrix2d cone::get_cov()
{
    return cov;
}
Eigen::Vector2d cone::getCart()
{
    Eigen::Vector2d ret; 
    ret << x, y; 
}
Eigen::Vector2d  cone::get_polar()
{
    Eigen::Vector2d ret;
    double r = sqrt(x*x + y*y);
    double theta = atan2(y, x); 
    ret << r, theta; 
    return ret;
}