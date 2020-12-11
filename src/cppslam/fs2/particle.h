#ifndef PARTICLE_H
#define PARTICLE_H

#include "Eigen/Dense"
#include "cone.h"

class particle
{
    public: 
        particle(double weight_0, Eigen::MatrixXf cov); 
        cone get_landmark(int idx);
        
        Eigen::Vector4f get_p_pose();
        Eigen::Matrix4f get_cov();

        void set_pose(Eigen::Vector4f new_pose); 
        void set_cov(Eigen::Matrix4f new_cov);

        void add_new_lm(cone new_lm, Matrix2f cov);
                
        int lm_size();
        void set_weight(double w);

        std::vector<cone>  get_landmarks();
        std::vector<Eigen::Matrix2f> landmark_covariance; 
    
    private:
        double weight;
        Eigen::VectorXf pose; 
        Eigen::Matrix2d cov;
        std::vector<cone> landmarks; 

};

#endif