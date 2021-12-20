#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <iiwa_tools/GetFK.h>
#include <cstdlib>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <armadillo>
#include <bits/stdc++.h>
#include <chrono> 

#include"src/error_costfl.hpp"
#include"src/SAC.hpp"
#include"src/rk4_int.hpp"
#include"src/koopsys.hpp"
#include"src/iiwabasis2.hpp"

//desire traj
arma::vec xdk(double t){
	arma::vec dtraj = arma::zeros(212);
	dtraj[0] = 0.6;						//x
	dtraj[1] = 0.25*cos((1/5)*t);		//y
	dtraj[2] = 0.1*sin((2/5)*t)+0.7;	//z
	dtraj[3] = 1;						//wx
	dtraj[4] = 0;						//wy
	dtraj[5] = 0;						//wz
	dtraj[6] = 0;						//w
	return dtraj;
}

// nomial input u_norm
arma::vec unom(double t){
        return 0.1*arma::ones(7);};

class figureight{
	ros::NodeHandle *nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::ServiceClient client;
	iiwa_tools::GetFK srv;
	std_msgs::Float64MultiArray seed1;
	std_msgs::Float64MultiArray output;
	geometry_msgs::Pose RobotPoses;
	arma::vec pose = {0., 0., 0., 0., 0., 0., 0.};
	arma::vec pos;
	arma::vec vel;
	arma::vec Xcurr;
	arma::vec Ucurr;
	KoopSys<iiwaBasis2> *systKPointer;
	iiwaBasis2 *iiwaBasisPointer;
	sac<KoopSys<iiwaBasis2>, errorcostfl<KoopSys<iiwaBasis2>>> *sacsysKPointer;
		
	public:
	figureight(ros::NodeHandle *_nh, KoopSys<iiwaBasis2> *_systKPointer, iiwaBasis2 *_iiwaBasisPointer, sac<KoopSys<iiwaBasis2>, errorcostfl<KoopSys<iiwaBasis2>>> *_sacsysKPointer){
		
		nh = _nh;
		systKPointer = _systKPointer;
		iiwaBasisPointer = _iiwaBasisPointer;
		sacsysKPointer = _sacsysKPointer;
		Xcurr = arma::zeros(21);
		Ucurr = arma::zeros(7);
		systKPointer->Xcurr = iiwaBasisPointer->zx(Xcurr);//std::cout<<(iiwaBasisPointer->zx(Xcurr));
		systKPointer->Ucurr = Ucurr;
		seed1.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(0)
		seed1.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(1)
		seed1.layout.dim[0].size = 1;
		seed1.layout.dim[1].size = 7;
		output.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(0)
		output.layout.dim[0].size = 7;
		output.data = {0., 0., 0., 0., 0., 0., 0.};
		client = nh->serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
		
		pub = nh->advertise<std_msgs::Float64MultiArray>("/iiwa/TorqueController/command",10);
		sub = nh->subscribe("/iiwa/joint_states", 1000, &figureight::chatterCallback, this);
		
		
	};
	
	void chatterCallback(const sensor_msgs::JointState& jointState){
		//std::cout<<jointState;
		pos = jointState.position;
		vel = jointState.velocity;
		seed1.data = jointState.position;
		srv.request.joints = seed1; //std::cout<<srv.request.joints;
	};
	
	void calc_u(const ros::TimerEvent& event){
		
		if(client.call(srv)){
			auto begin = std::chrono::high_resolution_clock::now();
			
			RobotPoses = srv.response.poses[0];//std::cout<<RobotPoses.position;
			pose[0] = RobotPoses.position.x;
			pose[1] = RobotPoses.position.y;
			pose[2] = RobotPoses.position.z;
			pose[3] = RobotPoses.orientation.x;
			pose[4] = RobotPoses.orientation.y;
			pose[5] = RobotPoses.orientation.z;
			pose[6] = RobotPoses.orientation.w;//std::cout<<pose<<std::endl;
			
			Xcurr = arma::join_cols(pos, vel, pose);//std::cout<<Xcurr<<std::endl;
			
			//update & calcluate Koopman Operator
			systKPointer->calc_K(Xcurr,Ucurr);//std::cout<<(systKPointer->hx)<<std::endl;
			
			//calculate SAC
			sacsysKPointer->SAC_calc();
			
			Ucurr = sacsysKPointer->ulist.col(0); //std::cout<<Ucurr;
			sacsysKPointer->unom_shift();
			
			output.data.clear();
			output.data.insert(output.data.end(), Ucurr.begin(), Ucurr.end()); //std::cout<<output;
			//std::cout<<"update_u"<<std::endl;
			auto end = std::chrono::high_resolution_clock::now();
			std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count() << "ns" << std::endl;
			
		}
		else{
			std::cout<<"wait for service"<<std::endl;
			//std::cout<<srv.request.joints;
		};
	};
	
	void publishing(const ros::TimerEvent& event){
		pub.publish(output);
	}
	
};


int main(int argc, char **argv){
	ros::init(argc, argv,"figure8");
	ros::NodeHandle nh;
	const double DT=0.05;
	const double DTP = 0.002;
	const double t_calc = 0.1;
	const double T = 0.5;
	iiwaBasis2 iiwaBasis;
	KoopSys<iiwaBasis2> systK (0.01, &iiwaBasis);
	
	// define maximun input
	arma::vec umax = {20, 60, 15, 15, 5, 4, 0.5};  
	
	// define cost function gain
	arma::mat R = 0.001*arma::eye(7,7);
 	arma::mat Qk = arma::zeros(iiwaBasis.xdim,iiwaBasis.xdim);
	arma::vec Qvec = {1,1,1,1,1,1,1};
	Qk.submat(0,0,6,6)=200*arma::diagmat(Qvec);//std::cout<<Qk;
	arma::mat Qf = arma::zeros<arma::mat>(size(Qk));
	
	// define cost function errorcostfl
	arma::vec noisecov = 1.0*arma::ones(iiwaBasis.xdim);
	errorcostfl<KoopSys<iiwaBasis2>> costK (Qk,R,xdk,&systK,noisecov);
	
	// define sac controllor
	sac<KoopSys<iiwaBasis2>,errorcostfl<KoopSys<iiwaBasis2>>> sacsysK (&systK,&costK,t_calc,T,umax,unom);
	

	// multithread for each CPU core
	ros::MultiThreadedSpinner spinner(0);
	
	figureight feight = figureight(&nh, &systK, &iiwaBasis, &sacsysK);
	ros::Timer timer = nh.createTimer(ros::Duration(DT), &figureight::calc_u, &feight);
	ros::Timer timerP = nh.createTimer(ros::Duration(DTP), &figureight::publishing, &feight);
	spinner.spin();
	return 0;
}