#ifndef FS2_H
#define FS2_H

#include "ros/ros.h"
#include <math.h>
#include <Eigen/Dense>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "../discreteBayesFilter/discreteBayes.h"
#include "mur_common/cone_msg.h"
#include <cassert>
#include <vector>
#include <algorithm>
#include "../point/point.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"

#include "../discreteBayesFilter/discreteBayes.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include "particle.h"
#include "motionmodel.h"
#include "Observation.h"


#define BLUE 0
#define YELLOW 1
#define ORANGE 2
#define BIG 3 
#define UNKNOWN 4

#define BLUE_STR "BLUE"
#define ORANGE_STR "ORANGE"
#define YELLOW_STR "YELLOW"
#define BIG_STR "BIG"
#define UNKNOWN_STR "na"

#define QUE_SIZE 1 

#define M 100 
#define STATE_SIZE 4

#define COV_CONST 0.001

#define HZ 20
#define DT 1.0/HZ

#define P0 0.001

const double PI = 3.141592653589793238463;

class fastslamtwo
{
    public:

        fastslamtwo(ros::NodeHandle n, int state_size, int hz); 
        fastslamtwo(ros::NodeHandle n, int state_size);     
        void run(std::vector<Observation> Observations, Eigen::Vector2d u, double dt); 
    private:
        std::vector<std::vector<double>> calc_samp_dist(particle &p, std::vector<Observation> zs, Eigen::Vector2d u, double dt);
        std::pair<Eigen::Matrix2d, Eigen::Matrix<double, 2, STATE_SIZE>> calculate_jacobians(cone lm, Eigen::VectorXf particle_pose);

        Observation predict_observation(cone lm, Eigen::VectorXf pose);
        cone predict_observation_inverse(Observation z, Eigen::VectorXf pose);
        
        ros::NodeHandle nh; 

        std::vector<particle> particles;

        int launchSubscribers();
        int launchPublishers();
        void initSubscribers();

        void steeringcallback(const geometry_msgs::Twist &data);
        void accelerationcallback(const geometry_msgs::Accel &data);
        void cameracallback(const mur_common::cone_msg &data);
        void lidarcallback(const mur_common::cone_msg &data);
        void odomcallback(const nav_msgs::Odometry &data);
        
    	ros::Subscriber camCld;
        ros::Subscriber lidarCld;
        ros::Subscriber odomSub;
        ros::Subscriber steeringSub;
        ros::Subscriber accelSub;

        ros::Publisher track;
        ros::Publisher pose;
        ros::Publisher track_markers;

		std::string ODOM_TOPIC = "/mur/Odom";
		std::string CAM_TOPIC = "/camera/cones";
		std::string LIDAR_TOPIC = "/conepose/cone_messages_sim";
		std::string FILTERED_TOPIC = "/mur/slam/cones";
		std::string SLAM_POSE_TOPIC = "/mur/slam/Odom";
		std::string STEERING_TOPIC = "/mur/control/desired";
		std::string MARKER_ARRAY_TOPIC = "/mur/slam/map_markers";
        std::string ACCEL_TOPIC     = "/mur/accel_desired";

        Eigen::Matrix2d m_R_T;
        Eigen::Matrix3d m_P_T;
};

#endif