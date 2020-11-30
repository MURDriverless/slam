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
        Eigen::MatrixXf get_cov();

        void set_pose(Eigen::Vector4f new_pose); 
        void set_cov(Eigen::Matrix4f new_cov);
                
        int lm_size(); 
        std::vector<cone>  get_landmarks();
        std::vector<Eigen::Matrix2f> landmark_covariance; 
    
    private:
        
        Eigen::VectorXf pose; 
        Eigen::Matrix2d cov;

        std::vector<cone> landmarks; 

};

#endif