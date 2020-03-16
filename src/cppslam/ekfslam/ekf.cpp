#ifndef EKF_CPP
#define EKF_CPP
#include "ekf.h"


ekf::ekf(ros::NodeHandle n) 
{
	ROS_INFO_STREAM("Extended Kalman filter created");
	
}
void ekf::runnable(int hz, Matrix initialState)
{
	stateSize = initialState.GetNRows();
	if (initialState.GetNColumns() != 1 )
	{
		ROS_ERROR_STREAM("Incorrect input shape state");
	}
	ros::Rate looprate(hz);
	while(ros::ok)
	{
		ROS_INFO_STREAM("Running slam!");
		//do ekf slam things!
		looprate.sleep(); //enforce rate
	}
	
}
#endif
