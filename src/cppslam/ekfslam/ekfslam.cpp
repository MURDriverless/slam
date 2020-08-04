#ifndef EKF_CPP
#define EKF_CPP
#include "ekfslam.h"

int ekfslam::launchSubscribers(){
	try {
	camCld = nh.subscribe(CAM_TOPIC, QUE_SIZE, &ekfslam::ptcloudclbCam, this);
	lidarCld = nh.subscribe(LIDAR_TOPIC, QUE_SIZE, &ekfslam::ptcloudclbLidar, this);
	// control = nh.subscribe("/Cmd_vel", QUE_SIZE, &ekfslam::controlclb, this); 
	}
	catch (const char *msg){
		ROS_ERROR_STREAM(msg);
		return 0; // failure
	}
	return 1;
}

int ekfslam::launchPublishers(){
	try {
	track = nh.advertise<sensor_msgs::PointCloud2>(FILTERED_TOPIC, QUE_SIZE); 
	pose = nh.advertise<geometry_msgs::Pose2D>(SLAM_POSE_TOPIC, QUE_SIZE);	  
	}
	catch(const char *msg){
		ROS_ERROR_STREAM(msg);
		return 0; //failure
	}
	return 1; 
}
void ekfslam::processMeasurements(){
	// /* This function processes the measurements and associates them into a 
	// 	h mapping
	//  */
	//  double x_val,y_val;
	//  int length = z_lid.rows();

	//  std::vector<int> h_assoc_lidar;
	//  int idx;
	//  for (int i = 0; i<length; i++)
	//  {
	// 	 x_val = z_lid(0,i);
	// 	 y_val = z_lid(1,i);
	// 	 idx = ekfslam::getCorrespondingLandmark(x_val,y_val);
	// 	 if (idx >= lm_num){
	// 		 // New landmark discovered
	// 		ROS_INFO_STREAM("New landmark detected");
	// 		lm_num++;
	// 		//TODO: Look into ways to do this in place.
	// 		Eigen::Map<Eigen::MatrixXf> x_tmp(x.data(),1,2+x.size());
	// 		x = x_tmp;
	// 		Eigen::Map<Eigen::MatrixXf> cv_tmp(cv.data(),cv.size() + 2,cv.size() + 2);
	// 		cv = cv_tmp;
	// 	 }
	// }
	// associateMeasurements(h_assoc_lidar);

	return; 
}
int ekfslam::getCorrespondingLandmark(double x_val, double y_val){
	/* 
		Obtains the landmark associated with a measurement
		Currently uses direct distance.
	*/
	std::vector<double> distance; 
	double x_lm, y_lm, r; 
	for (int i = 0; i<lm_num; i++){
		
		x_lm = x(0,STATE_SIZE + i);
		y_lm = x(0,STATE_SIZE + i +1);

		r = sqrt((x_lm - x_val)+(y_lm -y_val));
		ROS_INFO("Distance: %lf", r);
		distance.push_back(r);
	}
	distance.push_back(MAX_DISTANCE);
	int min = *std::min_element(distance.begin(), distance.end());
	return min; 
}
void ekfslam::associateMeasurements(){
	/* Construct a measurement vector */
	int length = z_lid.rows();
	z = Eigen::MatrixXf::Zero(1,LM_SIZE * length + STATE_SIZE);
	
	for (int i = 0; i<length;i++){
		z(0,STATE_SIZE + LM_SIZE * i) = z_lid(i,0);
		z(0,STATE_SIZE + LM_SIZE * i + 1) = z_lid(i,1);
	}
	return;
}

ekfslam::ekfslam(ros::NodeHandle n, int state_size, int hz)
{
	ROS_INFO_STREAM("Extended Kalman filter created");

	nh = n;
	int status = 1;
	
	lm_num = 0;
	LM_SIZE = 2;
	STATE_SIZE = state_size;
	dt = 1.0/hz; //define the frequency of the system 
	HZ = hz;
	MAX_DISTANCE = 0.2; 
	
	// defining the state shape at initialization
	px = Eigen::MatrixXf::Zero(1,STATE_SIZE); // predicted mean
	pcv = 0.1 * Eigen::MatrixXf::Identity(STATE_SIZE,STATE_SIZE);// predicted Covariance
	y = Eigen::MatrixXf::Zero(1,STATE_SIZE); //innovation measurement residual
	S = Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE); // innovation covariance
	K = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE); // kalman gain 

	x = Eigen::MatrixXf::Zero(1,STATE_SIZE); // state
	cv = 0.1 * Eigen::MatrixXf::Identity(STATE_SIZE,STATE_SIZE); //state covariance 
	u = Eigen::MatrixXf::Zero(1,2); // Control
	Q = 0.1 * Eigen::MatrixXf::Identity(STATE_SIZE,STATE_SIZE);
	while (ros::ok())
	{
		try
		{
			// Launching Subscribers and Publishers

			status *= launchSubscribers();
			status *= launchPublishers(); 

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

void ekfslam::ProcessPoseMeasurements(){
	for (int i = 0; i <STATE_SIZE; i++){
		z(1,i) = 0.0; // not sure how to handle this curently
	}
}

void ekfslam::controlclb(std_msgs::String msg)
{
	ROS_INFO_STREAM("Control callback");
	return; 
}

/* State Definitions
 * x = {x,y,theta,velocity}^T
 * u = {velocity, angular velocity}
 * lm = [lm1, lm2 lm3 ...]
 * */
void ekfslam::odomclb(const nav_msgs::Odometry &data){
	return;
}
void ekfslam::runnable()
/* 
	Here is the main loop for the EKF SLAM method  
*/
{
	int newMeasurements, idx;
	ros::Rate looprate(HZ);
	ros::Subscriber odomSub;
	odomSub =nh.subscribe("/odom", QUE_SIZE, &ekfslam::odomclb, this);

	while (ros::ok())
	{
		// predict Step
		ekfslam::motionModel(); // predicts px
		ekfslam::computeJacobian(); // Computes Jacobian "F"
		Eigen::MatrixXf F_x = Eigen::MatrixXf::Zero(STATE_SIZE,lm_num * LM_SIZE + STATE_SIZE);
		F_x(0,0) =1;
		F_x(1,1) =1;
		F_x(2,2) =1;
		F_x(3,3) =1;
		F_x(4,4) =1;
		pcv = F * cv * F.transpose() + F_x * Q * F_x.transpose(); // predicts Covariance
		// // Sensor Processing (only lidar for now)
		newMeasurements = z_lid.cols();
		ROS_INFO("Measurements: %d ",newMeasurements);
		// associateMeasurements(); 
		y = z_lid - z; // this needs to be reviewed.
		for (int i = 0; i<newMeasurements; i++){
		ROS_INFO("new measurement \n");
		// do data association
		idx = ekfslam::getCorrespondingLandmark(z_lid(i,0),z_lid(i,0));
		ROS_INFO("Index: %d",idx);

		if (idx >= lm_num){
		// 		// New landmark discovered
				ROS_INFO_STREAM("New landmark detected");
				lm_num++;
		// 		// resize state arrays
				Eigen::Map<Eigen::MatrixXf> x_tmp(x.data(),1,2+x.rows());
				x = x_tmp;
				Eigen::Map<Eigen::MatrixXf> px_tmp(px.data(),1,2+px.rows());
				px = px_tmp;
				Eigen::Map<Eigen::MatrixXf> cv_tmp(x.data(),cv.cols() + 2,cv.rows() + 2);
				cv = cv_tmp;
				Eigen::Map<Eigen::MatrixXf> pcv_tmp(pcv.data(),pcv.cols() + 2,pcv.rows() + 2);
				pcv = cv_tmp;
		}
			
			// Compute sensor Jacobian and F matrix 
			Eigen::MatrixXf F_j =   Eigen::MatrixXf::Zero(STATE_SIZE+LM_SIZE,STATE_SIZE + lm_num * LM_SIZE);
			F_j(0,0) = 1; 
			F_j(1,1) = 1; 
			F_j(2,2) = 1; 
			F_j(3,3) = 1; 
			F_j(4,4) = 1;
			F_j(5,(idx+1)*LM_SIZE + STATE_SIZE) = 1;
			F_j(6,(idx+1)*LM_SIZE + STATE_SIZE) = 1;
			Eigen::MatrixXf H_j =   Eigen::MatrixXf::Zero(lm_num,STATE_SIZE+LM_SIZE);
			H_j(0,0) = 1; 
			H_j(1,1) = 1; 
			H_j(0,STATE_SIZE) = 1; 
			H_j(1,STATE_SIZE + 1) = 1; 
			H = H_j * F;
			K = pcv * H.transpose() * (H * pcv * H.transpose() + Q).inverse();
			px = px + K * y;
			Eigen::MatrixXf I = Eigen::MatrixXf::Identity(pcv.rows(),pcv.rows()); 
			pcv = (I - K * H) * pcv;  
		}
		// x = px; 
		// cv = pcv;
		// publishTrack();
		// publishPose();

		ros::spinOnce();
		
		looprate.sleep(); //enforce rate
	}
}
Point<double> ekfslam::getAbsolutePose(Point<double> p){
	Point<double> correctedLM(p.x,p.y,p.z);
	correctedLM.x += x(0,0);
	correctedLM.y += y(0,0);
	return correctedLM; 

}
void ekfslam::calcInnovation(int idx, Point<double> lm){
	Eigen::MatrixXf xpos(1,2);
	Eigen::MatrixXf zpos(1,2);
	Eigen::MatrixXf y(1,2);
	Eigen::MatrixXf delta(1,2); 

	double x_pos  = x(0,0);
	double y_pos = x(0,1);
	double q = x_pos*x_pos + y_pos*y_pos;
	// zpos(0,0) = z(0,STATE_SIZE + LM_SIZE*idx);
	// zpos(0,1) = z(0,1+ STATE_SIZE + LM_SIZE*idx);
	zpos(0,0) = lm.x;
	zpos(0,1) = lm.y;
	
	delta(0,0) = lm.x - x(0,0);
	delta(0,1) = lm.y - x(0,1);
	
	y = xpos - zpos;
	ekfslam::Jacob_H(q,delta,idx);
	
	return;

}
void ekfslam::Jacob_H(double q, Eigen::MatrixXf delta, int idx){
	H = Eigen::MatrixXf::Identity(5,5);

	return;
}
void  ekfslam::motionModel()
{
	/*
	MotionModel: 
		Uses basic euler motion integration, will have to use actual system 
		model for higher speeds Where non-linearities become significant.
	*/
	double theta = x(2,0);
	double v = x(3,0); 
	double theta_dot = x(4,0);
	v = theta * theta_dot * v;
	px(0,0) = x(0,0) + dt * v * cos(theta); 
	px(0,1) = x(0,1) + dt * v * sin(theta);
	px(0,2) = theta + dt * theta_dot;
	px(0,3) = u(0,0); // velocity commanded,
	px(0,4) = u(0,1); // angular velocity commanded.	
	// Landmarks dont need updating
	return;
}

void ekfslam::computeJacobian(){
	// computes Jacobian of state
	F = Eigen::MatrixXf::Identity(STATE_SIZE + lm_num,STATE_SIZE + lm_num );
	double v = x(0,3);
	double theta = x(0,2);
	F(0,2) = -dt * v * sin(theta);
	F(0,3) = dt * sin(theta);
	F(1,2) = dt * v * cos(theta);
	F(1,3) = dt * sin(theta);
	F(2,4) = dt; 
	return;
}


void ekfslam::ptcloudclbCam(const mur_common::cone_msg &data)
{
	int length_x = data.x.size();
	int length_y = data.y.size(); 

	// test here for length equality, otherwise bugs will occur. 
	if (length_x == 0) return;
	assert (length_x == length_y);
	z_cam = Eigen::MatrixXf::Zero(3,length_x);

	for (int i = 0; i <length_x; i++){
		z_cam(0,i) = data.x[i];
		// ROS_INFO("[ %f, %f, %f]",data.x[i],data.y[i],0.0 );
		z_cam(1,i) = data.y[i];
		z_cam(2,i) = 0;
	}
	return;
}

void ekfslam::ptcloudclbLidar(const mur_common::cone_msg &data)
{
	int length_x = data.x.size();
	int length_y = data.y.size();

	// test here for length equality, otherwise bugs will occur. 
	if (length_x == 0 || length_y == 0) return;
	assert (length_x == length_y);

	z_lid = Eigen::MatrixXf::Zero(3,length_x);
	for (int i = 0; i <length_x; i++){
		z_lid(0,i) = data.x[i];
		z_lid(1,i) = data.y[i];
		z_lid(2,i) = 0;
		// ROS_INFO("[ %f, %f, %f]",data.x[i],data.y[i],0.0 );
	}
	return;
}
void ekfslam::publishPose()
{
	// publish pose estimate
	geometry_msgs::Pose2D pose_pub;
	pose_pub.x = x(0,0);
	pose_pub.y = x(0,1);
	pose_pub.theta = x(0,2);
	pose.publish(pose_pub); 
}

void ekfslam::publishTrack()
{
	mur_common::cone_msg cone_msg;
	ros::Time current_time = ros::Time::now();

	cone_msg.header.frame_id = "/map";
	cone_msg.header.stamp = current_time;

	std::vector<float> x_cones;
	std::vector<float> y_cones; 
	x_cones.reserve(lm_num);
	y_cones.reserve(lm_num);
	for (int i = 0; i<lm_num; i++){
		x_cones.at(i) =  x(0,STATE_SIZE + i*LM_SIZE);
		y_cones.at(i) =  x(0,STATE_SIZE + i*LM_SIZE + 1);
	} 
	cone_msg.x = x_cones; 
	cone_msg.y = y_cones;
	// Including a colour vector so its not empty, (but it is)
	std::vector<std::string> cone_colour; 
	cone_msg.colour = cone_colour;
	// TODO: Add header 
	track.publish(cone_msg);
}
#endif
