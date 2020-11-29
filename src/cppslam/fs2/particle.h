#ifndef PARTICLE_H
#define PARTICLE_H

#include "Eigen/Dense"
#include "cone.h"

class particle
{
    public: 
        particle(double weight_0, Eigen::MatrixXf cov); 
        cone get_landmark(int idx);
        
        Eigen::VectorXf get_p_pose();
        Eigen::Matrix2d get_cov();

        void set_pose(Eigen::VectorXf new_pose); 
        void set_cov(Eigen::Matrix2d new_cov);
                
        int lm_size(); 

    private:
        
        Eigen::VectorXf pose; 
        Eigen::Matrix2d cov;

        std::vector<cone> landmarks; 

};

#endif