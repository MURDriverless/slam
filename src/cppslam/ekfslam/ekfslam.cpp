#ifndef EKF_CPP
#define EKF_CPP
#include "ekfslam.h"
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ekfslam::ekfslam(ros::NodeHandle n)
{
	ROS_INFO_STREAM("Extended Kalman filter created");

	int status = 1;
	nh = n;
	while (ros::ok())
	{
		try
		{
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
/* Some definitions of state
 * x = {x,y,theta,velocity}^T
 * u = {velocity, angular velocity}
 * lm = [lm1, lm2 lm3 ...]
 * */
void ekfslam::runnable()
{

	//subscribers
	camCld = nh.subscribe(CAM_TOPIC, que_size, &ekfslam::ptcloudclbCam, this);
	lidarCld = nh.subscribe(LIDAR_TOPIC, que_size, &ekfslam::ptcloudclbLidar, this);

	//Publishers
	stateSize = 4;
	int hz = 10;
	dt = 1.0 / hz;
 	Eigen::MatrixXf px(1,stateSize); // predicted mean
	Eigen::MatrixXf pcv(stateSize,stateSize);// predicted Covariance
       	Eigen::MatrixXf y(1,stateSize); //innovation measurement residual
	Eigen::MatrixXf S(stateSize,stateSize); // innovation covariance
	Eigen::MatrixXf K(stateSize, stateSize); // kalman gain 
	Eigen::MatrixXf x(1,stateSize); // state
	Eigen::MatrixXf cv(stateSize,stateSize); //state covariance 

	ros::Rate looprate(hz);
	while (ros::ok())
	{
		ROS_INFO("Running slam, hopefully at rate %d!", hz);
		//get sensors
		
		ros::spinOnce();
		//do ekf slam things!
		looprate.sleep(); //enforce rate
	}
}
Eigen::Matrix2d ekfslam::motionModel(Eigen::Matrix2d x, Eigen::Matrix2d u)
{
	/*
	MotionModel: 
	Uses basic euler motion integration, will have to use actual system 
	model for higher speeds Where non-linearities become significant.
	*/
	
	Eigen::Matrix2d x_updated(1,4);
	double theta = x(0,2);
	double v = x(0,3); 
	double theta_dot = x(0,4);
	x_updated(0,0) = x(0,0) + dt * v * cos(theta); 
	x_updated(0,1) = x(0,1) + dt * v * sin(theta);
	x_updated(0,2) = theta + dt * theta_dot;
	x_updated(0,3) = u(0,0); // velocity commanded,
	x_updated(0,4) = u(0,1); // angular velocity commanded.

	return x_updated;
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
void ekfslam::ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr& data)
{
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloud_ptr; 
	pcl::PCLPointCloud2 cloud_filtered; 
	pcl_conversions::toPCL(*data, *cloud); 
	return;
}
void ekfslam::ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data)
{
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloud_ptr; 
	pcl::PCLPointCloud2 cloud_filtered; 
	pcl_conversions::toPCL(*data, *cloud); 
	return; 
}
#endif
