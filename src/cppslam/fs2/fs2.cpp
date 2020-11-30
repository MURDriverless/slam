#ifndef FS2_CPP
#define FS2_CPP

#include "fs2.h"

fastslamtwo::fastslamtwo(ros::NodeHandle n, int state_size, int hz)
{
    this->m_R_T <<  1e-3,   0,
                0,      1e-3;
    this->m_P_T <<  1e-3,   0,      0,  0,
                    0,      1e-3,   0,  0,
                    0,      0,   1e-3,   0,
                    0,      0,      0,      1e-3;

    nh = n; 
    Eigen::MatrixXf cov_init = Eigen::MatrixXf::Identity(STATE_SIZE,STATE_SIZE);
    for (int i = 0; i<M; i++){
        particle tmp = particle(1.0/M, cov_init );
        particles.push_back(tmp);
    }
    return; 
} 
fastslamtwo::fastslamtwo(ros::NodeHandle n, int state_size)
{
    this->m_R_T <<  1e-3,   0,
                0,      1e-3;
    this->m_P_T <<  1e-3,   0,      0,  0,
                    0,      1e-3,   0,  0,
                    0,      0,   1e-3,   0,
                    0,      0,      0,      1e-3;
    return; 
}
void fastslamtwo::initSubscribers()
{
    launchSubscribers();
    launchPublishers();
    return;
}
void fastslamtwo::run(std::vector<Observation> Observations, Eigen::Vector2d u, double dt)
{
    for (particle &p : particles)
    {
        Eigen::Matrix4f cov_n_t_1 = p.get_cov();
        Eigen::Vector4d s_predicted = predictMotion(p.get_p_pose(), u, DT); 

        for(Observation z : Observations)
        {
            std::vector<float> pn;
            for (cone mu : p.get_landmarks())
            {
                Observation z_pred = predict_observation(mu, s_predicted);

                std::pair<Eigen::Matrix2d, Eigen::Matrix<double, 2, STATE_SIZE>> out_jacob = calculate_jacobians(mu,  s_predicted);

                Eigen::Matrix2d G_theta_n = out_jacob.first;

                Eigen::Matrix<double, 2, STATE_SIZE> G_s_n = out_jacob.second;      

                Eigen::Matrix2d Z_t_n = m_R_T + (G_theta_n * cov_n_t_1) * G_theta_n.transpose();

                Eigen::Matrix2d Z_t_n_inv = Z_t_n.inverse();

                Eigen::Matrix4f cov_st_n_inv = G_s_n.transpose() * Z_t_n_inv * G_s_n + cov_st_n_1.inverse();

                Eigen::Matrix4f cov_st_n = cov_st_n_inv.inverse();

            }
        }
    }

    for (particle &p : particles) {
        auto likelihood = calc_samp_dist(p, z, u, dt);
        associate_data(p, likelihood, u, z, dt);
    }

    particles = resample(particles, M);

    Eigen::VectorXf mean_pose = Eigen::VectorXf::Zero(STATE_SIZE);

    for (particle p : particles) {
        mean_pose = mean_pose + p.get_p_pose();
    }
    mean_pose = mean_pose / particles.size();
    return;
}
Observation fastslamtwo::predict_observation(cone lm, Eigen::VectorXf pose)
{
    Observation tmp(1.0,1.0,1); 
    
    return tmp;
}

std::vector<std::vector<double>> fastslamtwo::calc_samp_dist(particle &p, std::vector<Observation> zs, Eigen::Vector2d u, double dt)
{   
    std::vector<std::vector<double>> likelihood; 

    Eigen::VectorXf pose = p.get_p_pose();
    
    Eigen::VectorXf s_t_hat = predictMotion(pose, u, dt);

    //update the pose of the particle.
    p.set_pose(s_t_hat);
    p.set_cov(m_P_T);

    if (p.lm_size() == 0 || zs.size() == 0) {
        return likelihood;
    }

    std::vector<double> max_ps(zs.size(), 0.0);

    for (uint i = 0; i < p.lm_size(); i++){
        cone lm = p.get_landmark(i);
        std::vector<double> likelihood_lm;
        Eigen::Matrix2d cov_n_t_1 = lm.get_cov();

        Eigen::Vector4f mu_st_n_1 = p.get_p_pose();
        
        Eigen::Matrix4f cov_st_n_1 = p.get_cov();

        for (uint z_i = 0; z_i < zs.size(); ++z_i) {
            Observation z = zs[z_i];

            Observation z_t_n_hat = predict_observation(lm, mu_st_n_1);

            std::pair<Eigen::Matrix2d, Eigen::Matrix<double, 2, STATE_SIZE>> out_jacob = calculate_jacobians(lm,  p.get_p_pose());

            Eigen::Matrix2d G_theta_n = out_jacob.first;

            Eigen::Matrix<double, 2, STATE_SIZE> G_s_n = out_jacob.second;

            Eigen::Matrix2d Z_t_n = m_R_T + (G_theta_n * cov_n_t_1) * G_theta_n.transpose();

            Eigen::Matrix2d Z_t_n_inv = Z_t_n.inverse();

            Eigen::Matrix4f cov_st_n_inv = G_s_n.transpose() * Z_t_n_inv * G_s_n + cov_st_n_1.inverse();
            Eigen::Matrix4f cov_st_n = cov_st_n_inv.inverse();
            
            p.set_cov(cov_st_n);

            Eigen::Vector2d z_t_n_hat_polar = z_t_n_hat.getPolar();

            Eigen::Matrix2d obs_diff1;

            obs_diff1 << z.getR() - z_t_n_hat_polar(0), z.getT() - z_t_n_hat_polar(1);

            Eigen::VectorXf mu_st_n;

            mu_st_n = mu_st_n + (cov_st_n * G_s_n.transpose()* Z_t_n_inv) * obs_diff1;
        }

        //////////////////////////////
            //sample from this distribution to update pose
            Eigen::VectorXf sm_t_n;
            sm_t_n = mutlivariate_gaussian(mu_st_n, cov_st_n); 

            //calculate the data association likelihood, p
            Coordinate zm_t_n = predict_observation(lm, sm_t_n);

            std::pair<double, double> zm_t_n_polar = zm_t_n.get_polar();
            Observation obs_diff{z.range - zm_t_n_polar.first, z.bearing - zm_t_n_polar.second};

            //this is some sort of gaussian (3.40 in the springer book)
            double exponent = ((obs_diff.toVec().transpose()*Z_t_n_inv)*obs_diff.toVec())(0, 0);
            //std::cout << "exp: " << exponent << "\n";
            double num = std::exp(-0.5 * exponent);
            double den = std::sqrt( std::abs(2*M_PI* Z_t_n.determinant()));
            double p_n = num / den;

            //appending result
            WeightedObservation result{p_n, {z_t_n_hat_polar.first, z_t_n_hat_polar.second}};
            likelihood_lm.push_back(result);

            if (p_n >= max_ps[z_i]) {
                max_ps[z_i] = p_n;
                p.set_sampled_pose(sm_t_n, i, z_i);
                if (p_n > P_0) {
                    p.set_pose(mu_st_n);
                }
            }
        // } ///////////////////////////////// 


        //append likelihood_lm to likelihood.
        likelihood.push_back(likelihood_lm);
    }

    return likelihood;
}
std::pair<Eigen::Matrix2d, Eigen::Matrix<double, 2, 5>> fastslamtwo::calculate_jacobians(cone lm, Eigen::VectorXf particle_pose) 
{
    Eigen::Matrix2d G_theta;
    Eigen::Matrix<double, 2, STATE_SIZE> G_s;
    Eigen::Vector2d lm_pose = lm.getCart();

    double dx = particle_pose(0) - lm_pose(0);
    double dy = particle_pose(1) - lm_pose(1);

    double q = (dx * dx) + (dy * dy);
    double sqrt_q = sqrt(q);

    // review this!

    G_theta <<  (dx / sqrt_q),  (dy / sqrt_q),
                (-dy / q),      (dx / q);

    // s is a function of x, y and theta
    G_s <<  (-dx / sqrt_q), (-dy / sqrt_q), 0.0, 0.0,
            (dy / q),       (-dx / q),      -1.0, 0.0;

    return std::make_pair(G_theta, G_s);
}
int fastslamtwo::launchSubscribers(){
	try {
	odomSub = nh.subscribe(ODOM_TOPIC, 3, &fastslamtwo::odomcallback, this);
	camCld = nh.subscribe(CAM_TOPIC, QUE_SIZE, &fastslamtwo::cameracallback, this);
	lidarCld = nh.subscribe(LIDAR_TOPIC, QUE_SIZE, &fastslamtwo::lidarcallback, this);
	steeringSub = nh.subscribe(STEERING_TOPIC, QUE_SIZE, &fastslamtwo::steeringcallback, this); 
	accelSub = nh.subscribe(ACCEL_TOPIC, QUE_SIZE, &fastslamtwo::accelerationcallback, this);
    }
	catch (const char *msg){
		ROS_ERROR_STREAM(msg);
		return 0; // failure
	}
	
	return 1;
}

int fastslamtwo::launchPublishers(){
	try {
	track = nh.advertise<mur_common::cone_msg>(FILTERED_TOPIC, QUE_SIZE); 
	pose = nh.advertise<nav_msgs::Odometry>(SLAM_POSE_TOPIC, QUE_SIZE);
	track_markers = nh.advertise<visualization_msgs::MarkerArray>(MARKER_ARRAY_TOPIC, QUE_SIZE);
	}
	catch(const char *msg){
		ROS_ERROR_STREAM(msg);
		return 0; //failure
	}
	return 1; 
}

void fastslamtwo::steeringcallback(const geometry_msgs::Twist &data)
{

}
void fastslamtwo::accelerationcallback(const geometry_msgs::Accel &data)
{

}

void fastslamtwo::cameracallback(const mur_common::cone_msg &data)
{

}
void fastslamtwo::lidarcallback(const mur_common::cone_msg &data)
{

}
void fastslamtwo::odomcallback(const nav_msgs::Odometry &data)
{

}
#endif 