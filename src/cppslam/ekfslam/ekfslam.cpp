#ifndef EKF_CPP
#define EKF_CPP
#include "ekfslam.h"

void ekfslam::launchSubscribers(){
	camCld = nh.subscribe(CAM_TOPIC, QUE_SIZE, &ekfslam::ptcloudclbCam, this);
	lidarCld = nh.subscribe(LIDAR_TOPIC, QUE_SIZE, &ekfslam::ptcloudclbLidar, this);
	control = nh.subscribe("/Cmd_vel", QUE_SIZE, &ekfslam::controlclb, this); 
	return;
}

void ekfslam::launchPublishers(){
	track = nh.advertise<sensor_msgs::PointCloud2>(FILTERED_TOPIC, QUE_SIZE); 
	pose = nh.advertise<geometry_msgs::Pose2D>(SLAM_POSE_TOPIC, QUE_SIZE);	  
	return; 
}

ekfslam::ekfslam(ros::NodeHandle n, int state_size, int hz)
{
	ROS_INFO_STREAM("Extended Kalman filter created");

	int status = 1;
	nh = n;
	STATE_SIZE = state_size;
	dt = 1.0/hz; //define the frequency of the system 
	HZ = hz;

	// defining the state shape at initialization
	px = Eigen::MatrixXf::Zero(1,STATE_SIZE); // predicted mean
	pcv = Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE);// predicted Covariance
	y = Eigen::MatrixXf::Zero(1,STATE_SIZE); //innovation measurement residual
	S = Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE); // innovation covariance
	K = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE); // kalman gain 

	x = Eigen::MatrixXf::Zero(1,STATE_SIZE); // state
	cv = Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE); //state covariance 
	u = Eigen::MatrixXf::Zero(1,2); // Control

	while (ros::ok())
	{
		try
		{
			// Launching Subscribers and Publishers

			launchSubscribers();
			launchPublishers(); 

			if (!status)
			{
				throw "FAILED TO SUBSCRIBE TO SENSORS";
			}
			return; // returns if subscribers succesfully initialised
		}

		catch (const char *msg)
		{
			ROS_ERROR_STREAM(msg);
		}
	}
}
void ekfslam::controlclb(std_msgs::String msg)
{
	ROS_INFO_STREAM("Control callback");
	return; 
}

/* State Definitions
 * x = {x,y,theta,velocity}^T
 * u = {velocity, angular velocity}
 * lm = [lm1, lm2 lm3 ...]
 * */

void ekfslam::runnable()
/* 
	Here is the main loop for the EKF SLAM method  
*/
{

	ros::Rate looprate(HZ);
	while (ros::ok())
	{

		ROS_INFO("Running slam at rate %d!", HZ);
		
		ekfslam::motionModel();

		ros::spinOnce();
		
		looprate.sleep(); //enforce rate
	}
}
void  ekfslam::motionModel()
{
	/*
	MotionModel: 
		Uses basic euler motion integration, will have to use actual system 
		model for higher speeds Where non-linearities become significant.
	*/

	double theta = x(2,0);
	double v = x(3,0); 
	double theta_dot = x(4,0);
	v = theta * theta_dot * v;
	px(0,0) = x(0,0) + dt * v * cos(theta); 
	px(0,1) = x(0,1) + dt * v * sin(theta);
	px(0,2) = theta + dt * theta_dot;
	px(0,3) = u(0,0); // velocity commanded,
	px(0,4) = u(0,1); // angular velocity commanded.
	
	// Landmarks dont need updating

	return;
}


int ekfslam::initialiseSubs()
{
	try
	{
		return 1;
	}
	catch (char **msg)
	{
		ROS_ERROR_STREAM(msg);
		return 0;
	}
}
int stateSizeCalc(Eigen::MatrixXd z, Eigen::MatrixXd x)
{
	return 1; 
}
void ekfslam::ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr &data)
{
	return;
}

void ekfslam::ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data)
{
	return; 
}
#endif
