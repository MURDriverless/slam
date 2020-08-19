#ifndef EKF_H
#define EFK_H
#include "ros/ros.h"
#include <math.h>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../discreteBayesFilter/discreteBayes.h"
#include "mur_common/cone_msg.h"
#include "mur_common/mur_drive_cmd.h"
#include <cassert>
#include <vector>
#include "../point/point.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "../discreteBayesFilter/discreteBayes.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"


#define SIMULATED_MODE

#define MAX_DISTANCE  0.3
#define LM_SIZE 2

#define BLUE 0
#define YELLOW 1
#define ORANGE 2
#define UNKNOWN 3

#define BLUE_STR "BLUE"
#define ORANGE_STR "ORANGE"
#define YELLOW_STR "YELLOW"
#define UNKNOWN_STR "na"

#define PUBLISH_MARKERS


const double PI = 3.141592653589793238463;

double pi2pi(double val);
void printEigenMatrix(Eigen::MatrixXf mat);

typedef struct
{
	// red value
	float r; 
	// green value 
	float g; 
	// blue value
	float b; 
} rgb_t;

class ekfslam
{

public:
	ekfslam(ros::NodeHandle n, int state_size, int hz);
	ekfslam(ros::NodeHandle n, int state_size);
	void runnableStableRate();
	void runnableTrigger(int reading_type);

private:
	int launchSubscribers();
	int launchPublishers();

	void ptcloudclbCam(const mur_common::cone_msg &data);
	void ptcloudclbLidar(const mur_common::cone_msg &data);
	void odomclb(const geometry_msgs::Pose2D &data);
	void controlclb(const geometry_msgs::Twist &data);
	
	ros::Subscriber camCld;
	ros::Subscriber lidarCld;
	ros::Subscriber odomSub;
	ros::Subscriber controlSub;

	ros::Publisher track;
	ros::Publisher pose;
	ros::Publisher track_markers;

	int getCorrespondingLandmark(double x, double y);
	void UpdateCovariance();
	void motionModel();
	void computeJacobian();
	void associateMeasurements();

	void Jacob_H(double q, Eigen::MatrixXf delta, int idx);

	void publishPose();
	void publishTrack();

	Point<double> getAbsolutePose(Point<double> p);



	int STATE_SIZE;
	int controlSize;

	bool TRIGGER_MODE;

	ros::NodeHandle nh;
	double dt;
	int HZ;
	double time;
	//static message topic names

	#ifdef SIMULATED_MODE
		std::string ODOM_TOPIC = "/odom";
		std::string CAM_TOPIC = "/camera/cones";
		std::string LIDAR_TOPIC = "/cone_messages";
		std::string FILTERED_TOPIC = "/slam/map";
		std::string SLAM_POSE_TOPIC = "/slam/odom";
		std::string CONTROL_TOPIC = "/cmd_vel";
		std::string MARKER_ARRAY_TOPIC = "/map_markers";
	#endif

	#ifdef REAL_MODE
		std::string ODOM_TOPIC = "/mur/odom";
		std::string CAM_TOPIC = "/mur/cones/stereo";
		std::string LIDAR_TOPIC = "/mur/cones/lidar";
		std::string FILTERED_TOPIC = "/slam/map";
		std::string SLAM_POSE_TOPIC = "/slam/odom";
		std::string CONTROL_TOPIC = "/mur/control_desired";
		std::string MARKER_ARRAY_TOPIC = "/map_markers";
	#endif
	// Arrays & vectors that define the EKF
	Eigen::MatrixXf px;	 // predicted mean
	Eigen::MatrixXf pcv; // predicted Covariance

	Eigen::MatrixXf y; //innovation measurement residual
	Eigen::MatrixXf S; // innovation covariance
	Eigen::MatrixXf K; // kalman gain

	Eigen::MatrixXf x;	// state
	Eigen::MatrixXf cv; //state covariance
	Eigen::MatrixXf u;
	Eigen::MatrixXf G;
	
	Eigen::MatrixXf H; 


	Eigen::MatrixXf F; // Jacobian Matrix
	Eigen::MatrixXf Q; //

	Eigen::MatrixXf z_cam;
	Eigen::MatrixXf z_lid;

	Eigen::MatrixXf z;

	Eigen::MatrixXf colur_odds;

	int lm_num; // keeps track of the number of landmarks
	discreteBayes coneColourFilter;

	// std::vector<std::vector <Point<float>> > test;

	static const int QUE_SIZE = 1;
	// RGB colours
	rgb_t orange; 
	rgb_t blue; 
	rgb_t yellow;
	rgb_t white;

	std::vector<std::string> lidar_colors; 
	std::vector<std::string> camera_colors; 
	
};
#endif
