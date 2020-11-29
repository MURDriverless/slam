#ifndef CONE_H
#define CONE_H

#include "Eigen/Dense"


class cone{
    public:
        cone(double x, double y, int colour);
        
        Eigen::Vector2d getCart();
        Eigen::Vector2d get_polar();

        int getColour();

        Eigen::Matrix2d get_cov(); 

        void set_cov(Eigen::Matrix2d new_cov);

    private: 
        double x; 
        double y; 
        int colour;
        Eigen::Matrix2d cov;
};
#endif 