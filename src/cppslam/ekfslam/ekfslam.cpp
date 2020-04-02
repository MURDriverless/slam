#ifndef EKF_CPP
#define EKF_CPP
#include "ekfslam.h"

ekfslam::ekfslam(ros::NodeHandle n)
{
	ROS_INFO_STREAM("Extended Kalman filter created");

	int status = 1;
	nh = n;
	while (ros::ok)
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
	camCld = nh.subscribe(CAM_TOPIC, que, &ekfslam::ptcloudclbCam, this);
	lidarCld = nh.subscribe(LIDAR_TOPIC, que, &ekfslam::ptcloudclbLidar, this);

	//Publishers
	stateSize = 4;
	int hz = 10;
	dt = 1.0 / hz;

	ros::Rate looprate(hz);

	// Matrix x(stateSize, 1);			//mean or state estimate
	// Matrix c(stateSize, stateSize); //covariance of state estimate

	// x.CopyMatrix(initialState);
	// c.Identity();
	// x.MultiplyScalar(x, 0.1);

	while (ros::ok)
	{
		ROS_INFO_STREAM("Running slam!");
		//do ekf slam things!
		looprate.sleep(); //enforce rate
	}
}
Matrix ekfslam::motionModel(Matrix &x, Matrix &u)
{
	double theta = x.Get(0, 2);
	Matrix xp(x.GetNRows(), x.GetNColumns());
	xp.CopyMatrix(x);
	xp.Set(0, 0, x.Get(0, 0) + dt * x.Get(0, 3) * cos(theta));
	xp.Set(0, 0, x.Get(0, 1) + dt * x.Get(0, 3) * sin(theta));
	xp.Set(0, 2, x.Get(0, 2) + dt * u.Get(0, 1));
	xp.Set(0, 3, u.Get(0, 0));
	return xp;
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
void ekfslam::ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr &data)
{
	ROS_INFO_STREAM("Camera ptcloud recieved");
	return;
}
void ekfslam::ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data)
{
	ROS_INFO_STREAM("Lidar ptcloud recieved");
	return;
}
#endif
