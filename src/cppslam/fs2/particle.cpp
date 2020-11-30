#ifndef PARTICLE_CPP
#define PARTICLE_CPP

#include "particle.h"

particle::particle(double weight_0,Eigen::MatrixXf cov)
{
    return;
}

Eigen::VectorXf particle::get_p_pose()
{
    return pose;
}
void particle::set_pose(Eigen::VectorXf new_pose)
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

std::vector<cone>  particle::get_landmarks()
{
    return landmarks;
}

#endif 