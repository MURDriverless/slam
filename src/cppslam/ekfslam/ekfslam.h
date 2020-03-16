#include "../matrix/matrix.h"
#include "../matrix/matrixUtils.h"
#include "ros/ros.h"

class ekfSlam{
	public:
		ekfSlam(ros::NodeHandle);
		void runSlam();
	private:
		ros::NodeHandle n;
};	
