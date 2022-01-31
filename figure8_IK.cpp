#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <iiwa_tools/GetFK.h>
#include <iiwa_tools/GetIK.h>
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
using namespace std;

#include"src/error_cost.hpp"
#include"src/error_cost_IK.hpp"
#include"src/SAC.hpp"
#include"src/rk4_int.hpp"
#include"src/koopsys.hpp"
#include"src/iiwabasis1.hpp"
#include"src/iiwabasis3.hpp"
#include"src/LQR.hpp"
#include"src/fisher_cost.hpp"
#include"src/ActLearnK.hpp"
#include"src/f8traj_IK.hpp"

// nomial input u_norm
arma::vec unom(double t){
	return arma::zeros(7);};

class figureight{
	
	public:
	double t_dJS = 0.;		//time counter for desire joint state
	int counter = 10;		//counter for initial extra step
	ros::NodeHandle *nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::ServiceClient client;
	iiwa_tools::GetIK srv;
	geometry_msgs::Pose desire_Pose;
	std_msgs::Float64MultiArray seed1;
	std_msgs::Float64MultiArray output;
	arma::vec dJS = {0., 0., 0., 0., 0., 0., 0.};
	arma::vec pos;
	arma::vec vel;
	arma::vec Xcurr;
	arma::vec Ucurr;
	ofstream *myfile;
	iiwaBasis3 *iiwaBasisPointer;
	KoopSys<iiwaBasis3> *systKPointer;
	f8traj_IK<KoopSys<iiwaBasis3>> *trajPointer;
	sac<KoopSys<iiwaBasis3>, errorcostIK<KoopSys<iiwaBasis3>, f8traj_IK<KoopSys<iiwaBasis3>>>> *sacsysKPointer;
		
	figureight(ros::NodeHandle *_nh, iiwaBasis3 *_iiwaBasisPointer, KoopSys<iiwaBasis3> *_systKPointer, f8traj_IK<KoopSys<iiwaBasis3>> *_trajPointer, sac<KoopSys<iiwaBasis3>, errorcostIK<KoopSys<iiwaBasis3>, f8traj_IK<KoopSys<iiwaBasis3>>>> *_sacsysKPointer, ofstream *_myfile){
		// initialization
		nh = _nh;
		iiwaBasisPointer = _iiwaBasisPointer;
		systKPointer = _systKPointer;
		trajPointer = _trajPointer;
		sacsysKPointer = _sacsysKPointer;
		myfile = _myfile;
		Xcurr = arma::zeros(14);
		Ucurr = arma::zeros(7);
		systKPointer->Xcurr = iiwaBasisPointer->zx(Xcurr);//std::cout<<(iiwaBasisPointer->zx(Xcurr));
		systKPointer->Ucurr = Ucurr;
		seed1.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(0)
		seed1.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(1)
		seed1.layout.dim[0].size = 1;
		seed1.layout.dim[1].size = 7;
		seed1.data = {-0.162, 0.70, 0.07, -0.67, 0.10, 1.59, 0.};		// seed for IK solver
		output.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(0)
		output.layout.dim[0].size = 7;
		output.data = {0., 0., 0., 0., 0., 0., 0.};
		client = nh->serviceClient<iiwa_tools::GetIK>("/iiwa/iiwa_ik_server");
		
		pub = nh->advertise<std_msgs::Float64MultiArray>("/iiwa/TorqueController/command",10);
		sub = nh->subscribe("/iiwa/joint_states", 1000, &figureight::chatterCallback, this);
		
		
		// output data info to excel
		(*myfile)<<"time, joint state 1 error, joint state 2 error, joint state 3 error, joint state 4 error, joint state 5 error, joint state 6 error, joint state 7 error,";
		(*myfile)<<"vel 1, vel 2, vel 3, vel 4, vel 5, vel 6, vel 7, input 1, input 2, input 3, input 4, input 5, input 6, input 7, SAC predict cost QR, SAC predict cost fisher, SAC predict cost boundary,";
		(*myfile)<<"joint state 1, joint state 2, joint state 3, joint state 4, joint state 5, joint state 6, joint state 7, djoint state 1, djoint state 2, djoint state 3, djoint state 4, djoint state 5, djoint state 6, djoint state 7,current fisher_cost,";

		(*myfile)<<"\n";
		
	}; 
	
	void traj2Jointstate(){
		int Jointstate_counter = 0.;
		int total_step = trajPointer->total_step;
		while (Jointstate_counter < total_step){
			
			//desire trajectory
			arma::vec dpose = trajPointer->desire_traj(Jointstate_counter);
			desire_Pose.position.x = dpose[0];
			desire_Pose.position.y = dpose[1];
			desire_Pose.position.z = dpose[2];
			desire_Pose.orientation.x = dpose[3];
			desire_Pose.orientation.y = dpose[4];
			desire_Pose.orientation.z = dpose[5];
			desire_Pose.orientation.w = dpose[6];//std::cout<<desire_Pose<<std::endl;
		
			// Using IK client to solve desire joint state
			srv.request.poses = {desire_Pose};		// {} is necessary here for geometry_msgs::Pose	
			srv.request.seed_angles = seed1;//std::cout<<srv.request<<std::endl;
			client.waitForExistence();		//Wait for client existence
			if(client.call(srv)){		//Call the IK service 
				dJS = srv.response.joints.data;//std::cout<<dJS<<std::endl;
				trajPointer->djsmat.col(Jointstate_counter) = dJS;//std::cout<<trajPointer->djsmat.col(Jointstate_counter)<<std::endl;
				//std::cout<<trajPointer->desire_jointstate(Jointstate_counter*0.05);
				seed1.data = srv.response.joints.data;		//Update initial position for IK solver
				Jointstate_counter++;
			}
			else{
				std::cout<<"Error"<<std::endl;
			};
			
			//std::cout<<Jointstate_counter<<std::endl;
		};
	};
	
	void chatterCallback(const sensor_msgs::JointState& jointState){
		//std::cout<<jointState;
		pos = jointState.position;
		vel = jointState.velocity;
	};
	
	void calc_u(const ros::TimerEvent& event){
		// current state
		Xcurr = arma::join_cols(pos, vel);//std::cout<<Xcurr<<std::endl;
		
		//update & calcluate Koopman Operator
		systKPointer->calc_K(Xcurr,Ucurr);//std::cout<<(systKPointer->K)<<std::endl;

		if (counter){
			
			// extra initial movement for Koopman operator
			Ucurr = {30, -60, 40, -40, 10, 10, 5};
			output.data.clear();
			output.data = {30, -60, 40, -40, 10, 10, 5}; 
			counter--;
		}
		else {
			//calculate SAC
			sacsysKPointer->SAC_calc();
			Ucurr = sacsysKPointer->ulist.col(0); //std::cout<<Ucurr<<std::endl;
			sacsysKPointer->unom_shift(); 
			output.data.clear(); 
			output.data.insert(output.data.end(), Ucurr.begin(), Ucurr.end()); 
		}
		std::cout<<output<<std::endl;
		
		// output time, current state error and current output to excel
        (*myfile)<<systKPointer->tcurr<<",";
		arma::vec djs = sacsysKPointer->cost->traj->desire_jointstate(systKPointer->tcurr);
		for (int i=0; i<14; i++){
			(*myfile)<<Xcurr(i)-djs(i)<<",";
		}
		for (int i=0; i<7; i++){
			(*myfile)<<Ucurr(i)<<",";
		}
		
		// output sac final predict cost of QR, fisher, boundary 
		(*myfile)<<sacsysKPointer->cost->QR<<",";
		(*myfile)<<sacsysKPointer->cost->fisher<<",";
		(*myfile)<<sacsysKPointer->cost->boundary<<",";
		
		// output current state and desire state
		for (int i=0; i<7; i++){
			(*myfile)<<Xcurr(i)<<",";
		}
		for (int i=0; i<7; i++){
			(*myfile)<<djs(i)<<",";
		}
		
		// output fisher cost @ t
		(*myfile)<<sacsysKPointer->cost->fisher_cost(Xcurr, Ucurr, systKPointer->tcurr)<<",";
		
		(*myfile)<<"\n";
		
	};
	
	
	void publishing(const ros::TimerEvent& event){
		pub.publish(output);
	}
	
};


int main(int argc, char **argv){
	ros::init(argc, argv,"f8IK");
	ros::NodeHandle nh;
	const double DT=0.05;		//Time Duration for calculate torque
	const double DTP = 0.002;	//Time Duration for publishing torque
	const double t_calc = 0.1;
	const double T = 0.25;		// Horizon for SAC	
	
	// define iiwa basis function class
	iiwaBasis3 iiwaBasis;
	
	// define Koopman Operator class
	KoopSys<iiwaBasis3> systK(DT, &iiwaBasis);
	systK.K.save("/home/zxl5344/test/src/alei/robotdata/KO_initial.csv", arma::csv_ascii);
	
	// define desire traj class
	f8traj_IK<KoopSys<iiwaBasis3>> f8traj(&systK);
	
	// define file for storing measurment
	ofstream myfile;
	myfile.open ("/home/zxl5344/test/src/alei/robotdata/f8IK.csv");
	
	// define final time, initial time and time step
	double T_final = 10;
	double t_curr = 0;
	
	// define maximun input
	arma::vec umax = {60, 70, 50, 50, 10, 10, 5}; 
	//arma::vec umax = {500, 500, 500, 500, 500, 500, 500};
	
	// define cost function gain
	arma::mat R = 0.001*arma::eye(7,7);
 	arma::mat Qk = arma::zeros(iiwaBasis.xdim,iiwaBasis.xdim);
	arma::vec Qvec = arma::ones(14);
	Qk.submat(0,0,13,13)=200*arma::diagmat(Qvec);//std::cout<<Qk;
	arma::mat Qf = arma::zeros<arma::mat>(size(Qk));
	arma::vec noisecov = 1.0*arma::ones(iiwaBasis.xdim);	// diagonal for noise convariance for active learning
	//cout<<Boundarycoe<<endl;
	
	// define joint limit
	arma::vec Joint_limit = {2.94, 2.09, 2.96, 2.09, 2.96, 2.09, 3.05}; 
	
	// define cost function errorcost
	errorcostIK<KoopSys<iiwaBasis3>, f8traj_IK<KoopSys<iiwaBasis3>>> costK(Qk,R,&f8traj,&systK,noisecov, Joint_limit);
	
	// define sac controllor
	sac<KoopSys<iiwaBasis3>,errorcostIK<KoopSys<iiwaBasis3>,
				f8traj_IK<KoopSys<iiwaBasis3>>>> sacsysK (&systK,&costK,t_calc,T,umax,unom);
	
	// multithread for each CPU core
	//ros::MultiThreadedSpinner spinner(0);// multithread cause issues when using IK solver
	
	// initialize the feight class
	figureight feight = figureight(&nh, &iiwaBasis, &systK, &f8traj, &sacsysK, &myfile);
	
	// convert desire trajectory to desire joint state
	feight.traj2Jointstate();
	
	// save the desire trajectory
	feight.trajPointer->djsmat.save("/home/zxl5344/test/src/alei/robotdata/f8djs.csv", arma::csv_ascii);
	
	// calculate Koopman Operator and use SAC calculate torque
	ros::Timer timer = nh.createTimer(ros::Duration(DT), &figureight::calc_u, &feight);

	// publish torque
	ros::Timer timerP = nh.createTimer(ros::Duration(DTP), &figureight::publishing, &feight);
	
	ros::spin();		//spin command
	//spinner.spin();	//spin command for multithread
	
	// close the file
	myfile.close();
	
	return 0;
}