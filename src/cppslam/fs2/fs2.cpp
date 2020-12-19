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
        Eigen::Matrix4d cov_n_t_1 = p.get_cov();
        Eigen::Vector4d s_predicted = predictMotion(p.get_p_pose(), u, DT); 

        std::vector<int> association; 

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

                Eigen::Matrix4d cov_st_n_inv = G_s_n.transpose() * Z_t_n_inv * G_s_n + cov_n_t_1.inverse();

                Eigen::Matrix4d cov_n_t_1 = cov_st_n_inv.inverse();

                Eigen::Vector4f mu_st_n = s_predicted + cov_n_t_1 * G_s_n.transpose() * Z_t_n * (z.getPolar()-z_pred.getPolar());

                Eigen::MatrixXf y = mu.get_polar()- predict_observation(mu, mu_st_n);

                double p = 1.0/sqrt(2* PI * Z_t_n.determinant()) * exp( (y.transpose()) * Z_t_n_inv * (y));
                
                pn.push_back(p);
            }

            pn.push_back(P_0)

            int max_idx = std::max_element(pn.begin(),pn.end()) - pn.begin();

            association.push_back(max_idx);
        }

        for (int i = 0; i<association.size(); i++)
        {
            if (association[i] > p.get_landmarks().size())
            {
                Observation new_observation= Observations[i];

                cone new_cone = predict_observation_inverse(new_observation, p.get_p_pose())

                std::pair<Eigen::Matrix2d, Eigen::Matrix<double, 2, STATE_SIZE>> out_jacob = calculate_jacobians(new_cone,p.get_p_pose());

                Eigen::Matrix2d G_theta_n = out_jacob.first;

                Matrix2d cov = G_theta_n.transpose() * m_R_T.inverse() * G_theta_n;

                p.add_new_lm(new_cone,cov.inverse());

                p.set_weight(P_0);                
            }
            else
            {
                std::pair<Eigen::Matrix2d, Eigen::Matrix<double, 2, STATE_SIZE>> out_jacob = calculate_jacobians(new_cone,p.get_p_pose());

                Eigen::Matrix2d G_theta_n = out_jacob.first;

                Eigen::Matrix<double, 2, STATE_SIZE> G_s_n = out_jacob.second;      

                Eigen::Matrix2d Z_t_n = m_R_T + (G_theta_n * cov_n_t_1) * G_theta_n.transpose();

		        Eigen::Matrix2d K = p.get_landmark_covariance(max_idx) * G_theta_n * Z_t_n.inverse();
                
                Eigen::Vector2d y = mu.get_polar()- predict_observation(mu, mu_st_n);

                Eigen::Vector2d mu_n_hat = p.get_landmark(max_idx).get_polar() + K * (y);

                Eigen::Matrix2d cov_n_t = (Eigen::Matrix2d::Identity() - K * G_theta_n) * p.get_landmark_covariance(max_idx);

                p.update_landmark(mu_n_hat, cov_n_t);
                
                Eigen::Matrix2d L = G_s_n * p.get_cov() * G_s_n.transpose()  + G_theta_n * cov_n_t * G_theta_n + m_R_T;
            }
            
        }
    }

    return;
}

Observation fastslamtwo::predict_observation(cone lm, Eigen::VectorXf pose)
{
    Observation tmp(1.0,1.0,1); 
    assert(0);    
    return tmp;
}

cone fastslamtwo::predict_observation_inverse(Observation z, Eigen::VectorXf pose)
{
    cone cn; 
    assert(0);
    return cn;
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

	track = nh.advertise<mur_common::cone_msg>(FILTERED_TOPIC, QUE_SIZE); 
	pose = nh.advertise<nav_msgs::Odometry>(SLAM_POSE_TOPIC, QUE_SIZE);
	track_markers = nh.advertise<visualization_msgs::MarkerArray>(MARKER_ARRAY_TOPIC, QUE_SIZE);

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
