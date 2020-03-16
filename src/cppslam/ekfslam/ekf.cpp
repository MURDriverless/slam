#ifndef EKF_CPP
#define EKF_CPP
#include "ekf.h"


ekf::ekf(ros::NodeHandle n) 
{
	ROS_INFO_STREAM("Extended Kalman filter created");
	
	int status;

	nh = n;
	
	while(ros::ok)
	{
		try{
			status = initialiseSubs();	
			if (!status)
			{
				throw "FAILED TO SUBSCRIBE TO SENSORS";
			}
			return; // returns if subscribers succesfully initialised
		}
	

		catch(const char *msg)
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
void ekf::runnable(int hz, Matrix initialState)
{
	stateSize = initialState.GetNRows();
	dt = 1.0/hz;

	if (initialState.GetNColumns() != 1 )
	{
		ROS_ERROR_STREAM("Incorrect input shape state");
	}
	ros::Rate looprate(hz);

	Matrix x(stateSize,1); //mean or state estimate
	Matrix c(stateSize,stateSize);//covariance of state estimate
	
	x.CopyMatrix(initialState);
	c.Identity();
	x.MultiplyScalar(x,0.1);
	
	while(ros::ok)
	{
		ROS_INFO_STREAM("Running slam!");
		//do ekf slam things!
		looprate.sleep(); //enforce rate
	}
	
}
Matrix ekf::motionModel(Matrix &x, Matrix &u)
{
	double theta = x.Get(0,2);
	Matrix xp(x.GetNRows(), x.GetNColumns()); 
	xp.CopyMatrix(x);
	xp.Set(0,0,x.Get(0,0) + dt * x.Get(0,3) * cos(theta) );
	xp.Set(0,0,x.Get(0,1) + dt * x.Get(0,3) * sin(theta) );
	xp.Set(0,2,x.Get(0,2) + dt * u.Get(0,1) );
	xp.Set(0,3,u.Get(0,0) );
}
#endif
