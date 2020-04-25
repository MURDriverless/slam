#ifndef EKF_CPP
#define EKF_CPP
#include "ekfslam.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


void ekfslam::launchSubscribers(){
	//subscribers
	camCld = nh.subscribe(CAM_TOPIC, QUE_SIZE, &ekfslam::ptcloudclbCam, this);
	lidarCld = nh.subscribe(LIDAR_TOPIC, QUE_SIZE, &ekfslam::ptcloudclbLidar, this);
	control = nh.subscribe("/Cmd_vel", QUE_SIZE, &ekfslam::controlclb, this); 
	return;
}
void ekfslam::launchPublishers(){
	// track = nh.advertise<>; 
	// pose;  
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
	Eigen::MatrixXf px(1,stateSize); // predicted mean
	Eigen::MatrixXf pcv(stateSize,stateSize);// predicted Covariance

	Eigen::MatrixXf y(1,stateSize); //innovation measurement residual
	Eigen::MatrixXf S(stateSize,stateSize); // innovation covariance
	Eigen::MatrixXf K(stateSize, stateSize); // kalman gain 

	Eigen::MatrixXf x(1,stateSize); // state
	Eigen::MatrixXf cv(stateSize,stateSize); //state covariance 
	Eigen::MatrixXf u(1,2);
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
/* Some definitions of state
 * x = {x,y,theta,velocity}^T
 * u = {velocity, angular velocity}
 * lm = [lm1, lm2 lm3 ...]
 * */
void ekfslam::runnable()
{

	ros::Rate looprate(HZ);
	while (ros::ok())
	{
		ROS_INFO("Running slam at rate %d!", HZ);
		//get sensors
		ekfslam::motionModel(x, u);
		ros::spinOnce();
		
		looprate.sleep(); //enforce rate
	}
}
Eigen::Matrix2d  ekfslam::motionModel(Eigen::MatrixXf x, Eigen::MatrixXf u)
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
	px = Eigen::MatrixXf(x);

	px(0,0) = x(0,0) + dt * v * cos(theta); 
	px(0,1) = x(0,1) + dt * v * sin(theta);
	px(0,2) = theta + dt * theta_dot;
	px(0,3) = u(0,0); // velocity commanded,
	px(0,4) = u(0,1); // angular velocity commanded.
	
	// all landmarks in the state variable list are not needed

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
