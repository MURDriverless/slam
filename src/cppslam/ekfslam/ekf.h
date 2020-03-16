#ifndef EKF_H
#define EFK_H
#include "../matrix/matrix.h"
#include "../matrix/matrixUtils.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>


class ekf {

public:
	ekf(ros::NodeHandle n);


	void runnable(int hz,Matrix initialState);
private:
	int stateSize;
	int controlSize;
	ros::NodeHandle nh;
};
#endif
