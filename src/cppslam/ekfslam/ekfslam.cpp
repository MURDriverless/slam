#ifndef EKF_CPP
#define EKF_CPP
#include "ekfslam.h"

int ekfslam::launchSubscribers(){
	try {
	camCld = nh.subscribe(CAM_TOPIC, QUE_SIZE, &ekfslam::ptcloudclbCam, this);
	lidarCld = nh.subscribe(LIDAR_TOPIC, QUE_SIZE, &ekfslam::ptcloudclbLidar, this);
	odomSub = nh.subscribe(ODOM_TOPIC, QUE_SIZE, &ekfslam::odomclb, this);
	controlSub = nh.subscribe(CONTROL_TOPIC, QUE_SIZE, &ekfslam::controlclb, this); 
	}
	catch (const char *msg){
		ROS_ERROR_STREAM(msg);
		return 0; // failure
	}
	return 1;
}

int ekfslam::launchPublishers(){
	try {
	track = nh.advertise<mur_common::cone_msg>(FILTERED_TOPIC, QUE_SIZE); 
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
	double x_lm, y_lm, r_min, r;
	int min_idx = lm_num;
	r_min = MAX_DISTANCE;
	for (int i = 0; i<lm_num; i++){
		
		x_lm = px(STATE_SIZE + LM_SIZE*i,0);
		y_lm = px(STATE_SIZE + LM_SIZE * i +1,0);

		r = sqrt((x_lm - x_val)*(x_lm - x_val)+(y_lm -y_val)*(y_lm -y_val));
		if (r < r_min){
			r_min = r;
			min_idx = i;  
		}
	}
	return min_idx; 
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
	
	STATE_SIZE = state_size;
	dt = 1.0/hz; //define the frequency of the system 
	HZ = hz;

	// defining the state shape at initialization
	px = Eigen::MatrixXf::Zero(STATE_SIZE,1); // predicted mean
	pcv = 0.1 * Eigen::MatrixXf::Identity(STATE_SIZE,STATE_SIZE);// predicted Covariance
	S = Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE); // innovation covariance

	x = Eigen::MatrixXf::Zero(STATE_SIZE,1); // state
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

/* State Definitions
 * x = {x,y,theta,velocity}^T
 * u = {velocity, angular velocity}
 * lm = [lm1, lm2 lm3 ...]
 * */
void ekfslam::odomclb(const geometry_msgs::Pose2D &data){
	
	return;
}
void ekfslam::runnable()
/* 
	Here is the main loop for the EKF SLAM method  
*/
{
	int newMeasurements, idx, new_size, rows;
	double xlm, ylm, xr, yr, theta_p; 
	ros::Rate looprate(HZ);
	


	while (ros::ok())
	{
		// predict Step
		ekfslam::motionModel(); // predicts px
		ekfslam::computeJacobian(); // Computes Jacobian "F"

		Q = 0.1 * Eigen::MatrixXf::Identity(pcv.rows(),pcv.rows());
		
		ekfslam::UpdateCovariance();
		// pcv = F * cv * F.transpose() + F_x * Q * F_x.transpose(); // predicts Covariance
		// printf("Predict cv");
		// printEigenMatrix(pcv);
		// Sensor Processing (only lidar for now)
		newMeasurements = z_lid.cols();
		// printf("Lidar");
		// printEigenMatrix(z_lid);

		// ROS_INFO("Measurements: %d ",newMeasurements);
		// associateMeasurements(); 
		for (int i = 0; i<newMeasurements; i++){
			// ROS_INFO("New measurement");
			// do data association
			xr = z_lid(0,i); 
			yr = z_lid(1,i); 
			theta_p = px(2,0);
			xlm = px(0,0) + xr * cos(theta_p) + yr * sin(theta_p);
			ylm = px(1,0) - xr * sin(theta_p) + yr * cos(theta_p);
			ROS_INFO("XLM: %lf", xlm);
			ROS_INFO("YLM: %lf", ylm);
			
			idx = ekfslam::getCorrespondingLandmark(xlm,ylm);
			ROS_INFO("Index: %d",idx);

			if (idx >= lm_num){
					// New landmark discovered
					ROS_INFO_STREAM("New landmark detected");
					lm_num++;
					// resize state arrays
					new_size = px.rows() + LM_SIZE; 
					rows = px.rows();
					// ROS_INFO("New size: %d", new_size);
					x.conservativeResizeLike(Eigen::MatrixXf::Zero(new_size,1));
					px.conservativeResizeLike(Eigen::MatrixXf::Zero(new_size,1));
					px(rows,0) = xlm;
					px(rows+1,0) = ylm;

					cv.conservativeResizeLike(Eigen::MatrixXf::Zero(new_size,new_size));
					pcv.conservativeResizeLike(Eigen::MatrixXf::Zero(new_size,new_size));
					pcv(new_size-1,new_size-1) = 0.1;
					pcv(new_size-2,new_size-2) = 0.1;
					
					
					// ROS_INFO("Resizing complete");
			}
			y = Eigen::MatrixXf::Zero(2,1);
			y(0,0) = xlm - (px(STATE_SIZE + idx * LM_SIZE, 0)); 
			y(1,0) = ylm - (px(STATE_SIZE + idx * LM_SIZE + 1,0));

			printf("y");
			printEigenMatrix(y);	
			// Compute sensor Jacobian and F matrix 
			Eigen::MatrixXf F_j =   Eigen::MatrixXf::Zero(STATE_SIZE+LM_SIZE,STATE_SIZE + lm_num * LM_SIZE);
			Eigen::MatrixXf H_j =   Eigen::MatrixXf::Zero(LM_SIZE,STATE_SIZE+LM_SIZE);
			F_j(0,0) = 1; 
			F_j(1,1) = 1; 
			F_j(2,2) = 1; 
			F_j(3,3) = 1; 
			F_j(4,4) = 1;
			F_j(5,(idx)*LM_SIZE + STATE_SIZE) = 1;
			F_j(6,(idx)*LM_SIZE + STATE_SIZE + 1) = 1;

			H_j(0,0) = 1;
			H_j(0,2) = -xr * sin(theta_p) + yr * cos(theta_p);  
			H_j(0,STATE_SIZE) = cos(theta_p);
			H_j(0,STATE_SIZE+1) = sin(theta_p);

			H_j(0,STATE_SIZE) = cos(theta_p);
			H_j(0,STATE_SIZE+1) = sin(theta_p);
			 
			H_j(1,1) = 1;
			H_j(1,2) = -xr * cos(theta_p) - yr * sin(theta_p);
			H_j(1,STATE_SIZE) = -sin(theta_p);
			H_j(1,STATE_SIZE+1) = cos(theta_p);
			H_j = H_j * - 1;
			H = Eigen::MatrixXf::Zero(2,7);
			H = H_j * F_j;
			printf("F_j");
			printEigenMatrix(F_j);
			printf("H_j");

			printEigenMatrix(H_j);
			printf("H");

			printEigenMatrix(H);
			printf("Pcv");
			printEigenMatrix(pcv);

			K = Eigen::MatrixXf::Zero(STATE_SIZE + LM_SIZE,STATE_SIZE + LM_SIZE);
			Eigen::MatrixXf k_tmp; 
			Eigen::MatrixXf Q_small; 
			Q_small = Eigen::MatrixXf::Identity(LM_SIZE, LM_SIZE)*0.01;
			k_tmp = (H * pcv * H.transpose() + Q_small).inverse();
			// printf("K_temp");
			// printEigenMatrix(k_tmp);
			K = pcv * H.transpose() * k_tmp;
			printf("K");
			printEigenMatrix(K);


			px = px +  K*y;
			printf("px");
			printEigenMatrix(px);

			Eigen::MatrixXf I = Eigen::MatrixXf::Identity(pcv.rows(),pcv.rows()); 
			pcv = (I - K * H) * pcv;  
			// printf("Pcv");
			// printEigenMatrix(pcv);
		}
		x = px; 
		cv = pcv;
		printf("X"); 
		printEigenMatrix(x);
		// printf("CV"); 
		// printEigenMatrix(cv);
		

		publishTrack();
		publishPose();

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
	double v = u(0,0); 
	double theta_dot = u(0,1);
	px(0,0) = x(0,0) + dt * v * cos(theta); 
	px(1,0) = x(1,0) + dt * v * sin(theta);
	px(2,0) = theta + dt * theta_dot;
	// px(2,0) = pi2pi(px(2,0));
	px(3,0) = u(0,0); // velocity commanded,
	px(4,0) = u(0,1); // angular velocity commanded.	
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
	if (length_x == 0 || length_y == 0){
		z_lid = Eigen::MatrixXf::Zero(0,0);
		return;
	};
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
	pose_pub.x = px(0,0);
	pose_pub.y = px(1,0);
	pose_pub.theta = px(2,0);
	pose.publish(pose_pub); 
}

void ekfslam::publishTrack()
{
	mur_common::cone_msg cone_msg;
	if (lm_num == 0) return;
	ros::Time current_time = ros::Time::now();

	cone_msg.header.frame_id = "/base_link";
	cone_msg.header.stamp = current_time;

	std::vector<float> x_cones(lm_num);
	std::vector<float> y_cones(lm_num);

	for (int i = 0; i<lm_num; i++){
		x_cones[i] =  px(STATE_SIZE + i*LM_SIZE,0);
		y_cones[i] = px(STATE_SIZE + i*LM_SIZE + 1,0);
	} 
	cone_msg.x = x_cones; 
	cone_msg.y = y_cones;
	// Including a colour vector so its not empty, (but it is)
	std::vector<std::string> cone_colour; 
	cone_msg.colour = cone_colour;
	// TODO: Add header 
	
	track.publish(cone_msg);
}
void ekfslam::UpdateCovariance(){
	Eigen::MatrixXf cv_tmp= Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE);
	Eigen::MatrixXf cv_upper= Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE);

	Eigen::MatrixXf G;
	Eigen::MatrixXf I = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE); 
	Eigen::MatrixXf jf= Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE);
	Eigen::MatrixXf F = Eigen::MatrixXf::Zero(STATE_SIZE,STATE_SIZE + LM_SIZE * lm_num);
	F(0,0) =1;
	F(1,1) =1;
	F(2,2) =1;
	F(3,3) =1;
	F(4,4) =1;
	double theta = x(3,0);
	double v = x(4,0);
	jf(0,2) = -dt * v * sin(theta); 
	jf(1,2) = dt * v * cos(theta);
	jf(0,3) = v*cos(theta);
	jf(1,3) = v*cos(theta);
	jf(2,4) = dt; 

	G = I + F.transpose() * jf * F;
	 
	cv_tmp = Eigen::MatrixXf::Zero(STATE_SIZE, STATE_SIZE);
	for (int i = 0; i < STATE_SIZE; i++){
		for (int j = 0; j <STATE_SIZE; j++){
			cv_tmp(i,j) = pcv(i,j);
		}
	}
	cv_upper = G.transpose() * cv_tmp * G + F.transpose() * (I) * F;

	for (int i = 0; i < STATE_SIZE; i++){
		for (int j = 0; j <STATE_SIZE; j++){
			pcv(i,j) = cv_upper(i,j);
		}
	}
	return;
}

void ekfslam::controlclb(const mur_common::mur_drive_cmd &data)
{
	u(0,0) = data.vel;
	u(0,1) = data.omega;
	return;
}
double pi2pi(double val){
	if (val> PI){
		return -(PI -std::fmod(val,PI));  
	}
	else if (val < -PI)
	{
		return (PI - std::fmod(val,PI) );
	}
	else{
		return val;
	}
}
void printEigenMatrix(Eigen::MatrixXf mat){
	printf("\n\n");
	if (mat.rows() == 0){
		ROS_WARN("ARRAY is empty");
	}
	int rows = mat.rows(); 
	int cols = mat.cols(); 
	printf("Rows: %d || Cols: %d \n\n", rows, cols);
	printf("[");
	for (int i = 0 ; i<mat.rows() ;i++){
			printf("[");
		for (int j = 0; j< mat.cols(); j++){
			printf("%f ,", mat(i,j));
		}
		printf("]\n");
	}
	printf("]\n\n");
	return;
}
#endif
