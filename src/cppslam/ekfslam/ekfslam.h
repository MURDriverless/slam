#ifndef EKF_H
#define EFK_H
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"  
#include "../discreteBayesFilter/discreteBayes.h"
#include "../point/point.h"

class ekfslam
{
	
public:
	ekfslam(ros::NodeHandle n, int state_size, int hz);
	void runnable();

private:
	void launchSubscribers();
	void launchPublishers(); 

	void ptcloudclbCam(const sensor_msgs::PointCloud2ConstPtr &data);
	void ptcloudclbLidar(const sensor_msgs::PointCloud2ConstPtr &data);
	void controlclb(std_msgs::String msg);
	
	ros::Subscriber camCld;
	ros::Subscriber lidarCld;
	ros::Subscriber control;
	ros::Publisher track; 
	ros::Publisher pose;  

	int initialiseSubs();
	
	int stateSizeCalc(Eigen::MatrixXd z, Eigen::MatrixXd x);
	void motionModel();
	
	
	int stateSize;
	int controlSize;
	ros::NodeHandle nh;
	double dt;
	int HZ; 

	//static message topic names
	std::string CAM_TOPIC = "/camera/cones";
	std::string LIDAR_TOPIC = "/lidar/cones";
	std::string FILTERED_TOPIC = "/slam/map";
	std::string SLAM_POSE_TOPIC = "/slam/odom";


	int STATE_SIZE;
	// Arrays & vectors that define the EKF 
	Eigen::MatrixXf px; // predicted mean
	Eigen::MatrixXf pcv;// predicted Covariance

	Eigen::MatrixXf y; //innovation measurement residual
	Eigen::MatrixXf S; // innovation covariance
	Eigen::MatrixXf K; // kalman gain 

	Eigen::MatrixXf x; // state
	Eigen::MatrixXf cv; //state covariance 
	Eigen::MatrixXf u;

	discreteBayes coneExistence[500]; 

	static const int QUE_SIZE = 1;

};
#endif
