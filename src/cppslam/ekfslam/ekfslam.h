#ifndef EKF_H
#define EFK_H
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../discreteBayesFilter/discreteBayes.h"
#include "mur_common/cone_msg.h"
#include <cassert>
#include <vector>
#include "../point/point.h"

class ekfslam
{

public:
	ekfslam(ros::NodeHandle n, int state_size, int hz);
	void runnable();

private:
	int launchSubscribers();
	int launchPublishers();

	void ptcloudclbCam(const mur_common::cone_msg &data);
	void ptcloudclbLidar(const mur_common::cone_msg &data);
	void controlclb(std_msgs::String msg);

	ros::Subscriber camCld;
	ros::Subscriber lidarCld;
	ros::Subscriber control;
	ros::Publisher track;
	ros::Publisher pose;

	int initialiseSubs();

	int stateSizeCalc(Eigen::MatrixXd z, Eigen::MatrixXd x);
	void motionModel();
	void computeJacobian();
	void associateMeasurements();

	int STATE_SIZE;
	int controlSize;
	ros::NodeHandle nh;
	double dt;
	int HZ;

	//static message topic names
	std::string CONE_MSG = "/cone_msg";
	std::string CAM_TOPIC = "/camera/cones" + CONE_MSG;
	std::string LIDAR_TOPIC = "/lidar/cones" + CONE_MSG;
	std::string FILTERED_TOPIC = "/slam/map";
	std::string SLAM_POSE_TOPIC = "/slam/odom";

	// Arrays & vectors that define the EKF
	Eigen::MatrixXf px;	 // predicted mean
	Eigen::MatrixXf pcv; // predicted Covariance

	Eigen::MatrixXf y; //innovation measurement residual
	Eigen::MatrixXf S; // innovation covariance
	Eigen::MatrixXf K; // kalman gain

	Eigen::MatrixXf x;	// state
	Eigen::MatrixXf cv; //state covariance
	Eigen::MatrixXf u;

	Eigen::MatrixXf F; // Jacobian Matrix
	Eigen::MatrixXf Q; //

	Eigen::MatrixXf z_cam;
	Eigen::MatrixXf z_lid;

	int lm_num; // keeps track of the number of landmarks

	std::vector<std::vector<discreteBayes>> coneExistence;

	// std::vector<std::vector <Point<float>> > test;

	static const int QUE_SIZE = 1;
};
#endif
