#ifndef EKF_H
#define EFK_H
#include "../matrix/matrix.h"
#include "./ekfSensor.h"
#include "../matrix/matrixUtils.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>

class ekf {

public:
	ekf(ros::NodeHandle n);
	void runnable(int hz,Matrix initialState);

private:
	int initialiseSubs();
	ekfSensor sensorManager; 
	Matrix motionModel(Matrix &x, Matrix &u);
	int stateSize;
	int controlSize;
	ros::NodeHandle nh;
	double dt; 	
};
#endif
