#ifndef PARTICLE_CPP
#define PARTICLE_CPP

#include "particle.h"

particle::particle(double weight_0,Eigen::Matrix4f icov)
{
    cov = icov;
    return;
}

void particle::set_weight(double w)
{
    weight = w;
}

Eigen::Vector4f particle::get_p_pose()
{
    return pose;
}
void particle::set_pose(Eigen::Vector4f new_pose)
{
    pose = new_pose;
}

void particle::set_cov(Eigen::Matrix4f new_cov)
{
    cov = new_cov;
}
Eigen::Matrix4f particle::get_cov()
{
    return cov;
}

int particle::lm_size()
{
    return landmarks.size();
}

cone particle::get_landmark(int idx)
{
    return landmarks[idx];
}
Eigen::Matrix2f particle::get_landmark_covariance(int i )
{
    return landmark_covariance[i];
}
std::vector<cone>  particle::get_landmarks()
{
    return landmarks;
}

void particle::add_new_lm(cone new_lm, Eigen::Matrix2f cov)
{
    landmarks.push_back(new_lm); 
    landmark_covariance.push_back(cov);
    return ;
}

void particle::update_landmark(int idx)
{

}
#endif 