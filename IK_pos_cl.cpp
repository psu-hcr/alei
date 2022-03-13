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
#include"src/fisher_cost.hpp"
#include"src/traj_IK.hpp"
#include"src/dot_model.hpp"
#include"src/dkl_cost.hpp"

const double PI = 3.1415926535987;

double phid(const arma::vec& x){
	arma::vec Mu= {{0.01}, {0.02}, {0.01}}; 
	arma::vec Mu2 = {{-0.03}, {-0.015},  {0.04}}; 
	arma::vec Mu3 = {{0.05}, {0.01},  {-0.035}}; 
	arma::vec sigdiag = {0.0001, 0.0001, 0.0001};
	arma::mat Sig = arma::diagmat(sigdiag);
	arma::vec sigdiag2 = {0.0001, 0.0001, 0.0001};
	arma::mat Sig2 = arma::diagmat(sigdiag2);
	double dist1 = 0.3*arma::as_scalar(arma::expmat(-0.5*(x-Mu).t()*Sig.i()*(x-Mu))/pow(pow(2*PI,3)*arma::det(Sig),0.5));
	double dist2 = 0.3*arma::as_scalar(arma::expmat(-0.5*(x-Mu2).t()*Sig2.i()*(x-Mu2))/pow(pow(2*PI,3)*arma::det(Sig2),0.5));
	double dist3 = 0.4*arma::as_scalar(arma::expmat(-0.5*(x-Mu3).t()*Sig.i()*(x-Mu3))/pow(pow(2*PI,3)*arma::det(Sig),0.5));
	return dist1+dist2+dist3;
};

arma::vec unom(double t){
        return arma::zeros(3,1);
};

class figureight{
	
	public:
	double t_dJS = 0.;		//time counter for desire joint state
	int counter = 2;		//counter for initial extra step
	ros::NodeHandle *nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::ServiceClient clientFK;
	ros::ServiceClient clientIK;
	iiwa_tools::GetIK srvIK;
	iiwa_tools::GetFK srvFK;
	geometry_msgs::Pose desire_Pose;
	geometry_msgs::Pose current_Pose;
	std_msgs::Float64MultiArray seed1;
	std_msgs::Float64MultiArray output;
	std_msgs::Float64MultiArray current_state;
	arma::vec dJS = {0., 0., 0., 0., 0., 0., 0.};
	arma::vec pos = {0., 0., 0., 0., 0., 0., 0.};
	arma::vec Pose = {0.5, 0, 0.1, 0, 0.7, 0};
	arma::vec vel;
	arma::vec Xcurr;
	arma::vec Ucurr;
	//arma::vec shift = {0.5, 0, 0.1, 0, 0.7, 0};
	arma::vec shift = {0.5, 0, 0.3, 0, 0.7, 0};
	ofstream *myfile;
	dot_model *sys1Pointer;
	dklcost<dot_model> *costPointer;
	sac<dot_model,dklcost<dot_model>> *sacPointer;
		
	figureight(ros::NodeHandle *_nh,  dot_model *_sys1Pointer,dklcost<dot_model> *_costPointer, 
		sac<dot_model,dklcost<dot_model>> *_sacPointer, ofstream *_myfile){
		// initialization
		nh = _nh;
		sys1Pointer = _sys1Pointer;
		costPointer = _costPointer;
		sacPointer = _sacPointer;
		myfile = _myfile;
		Ucurr = arma::zeros(7);
		seed1.layout.dim.push_back(std_msgs::MultiArrayDimension());	// setup dim(0)
		seed1.layout.dim.push_back(std_msgs::MultiArrayDimension());	// setup dim(1)
		seed1.layout.dim[0].size = 1;
		seed1.layout.dim[1].size = 7;
		seed1.data = {0.85, 0.65, -0.85, -0.74, 0.5, 1.88, -2.8};		// seed for IK solver pos (0.5, 0.3, 0.7)
		//seed1.data = {-0.37, 0.45, 0.96, -1.05, -0.37, 1.7, -2.};		// seed for IK solver pos (0.5, 0.1, 0.7)
		//seed1.data = {-0.3, 0.4, 0.9, -1.0, -0.3, 1.5, -2.};
		//seed1.data = {-0.2, 0.2, 0.2, -0.5, -0.3, 0.5, -1.};
		current_state.layout.dim.push_back(std_msgs::MultiArrayDimension());		// setup dim(0)
		current_state.layout.dim.push_back(std_msgs::MultiArrayDimension());		// setup dim(1)
		current_state.layout.dim[0].size = 1;
		current_state.layout.dim[1].size = 7;
		current_state.data = {0., 0., 0., 0., 0., 0., 0.};		// robot state for FK solver
		output.layout.dim.push_back(std_msgs::MultiArrayDimension());	// setup dim(0)
		output.layout.dim[0].size = 7;
		output.data = {0., 0., 0., 0., 0., 0., 0.};
		clientIK = nh->serviceClient<iiwa_tools::GetIK>("/iiwa/iiwa_ik_server");
		clientFK = nh->serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
		pub = nh->advertise<std_msgs::Float64MultiArray>("/iiwa/PositionController/command",10);
		sub = nh->subscribe("/iiwa/joint_states", 1000, &figureight::chatterCallback, this);
		
		// output data info to excel
		(*myfile)<<"x_sim, y_sim, z_sim,";
		(*myfile)<<"x, y, z,";
		(*myfile)<<"DJS 1, DJS 2, DJS 3, DJS 4, DJS 5, DJS 6, DJS 7,";
		(*myfile)<<"JS 1, JS 2, JS 3, JS 4, JS 5, JS 6, JS 7,";
		(*myfile)<<"\n";
		
	}; 
	
	arma::vec traj2Jointstate(arma::vec dpose){
		// Using IK client to convert pose to joint_sates
		desire_Pose.position.x = dpose[0];
		desire_Pose.position.y = dpose[2];
		desire_Pose.position.z = dpose[4];
		desire_Pose.orientation.x = 1;
		desire_Pose.orientation.y = 0;
		desire_Pose.orientation.z = 0;
		desire_Pose.orientation.w = 0;
		
		srvIK.request.poses = {desire_Pose};		// { } is necessary here for geometry_msgs::Pose	
		srvIK.request.seed_angles = seed1;		
		clientIK.waitForExistence();						// Wait for client existence
		if(clientIK.call(srvIK)){								// Call the IK service 
			dJS = srvIK.response.joints.data;	
			//seed1.data = srvIK.response.joints.data;		// comment out to fix ikseed
		}
		else{
			std::cout<<"Error in IK solver"<<std::endl;
		};
		return dJS;
	};
	
	arma::vec Jointstate2traj(arma::vec measure){
		// Using FK client to convert joint_states into pose
		current_state.data.clear(); 
		current_state.data.insert(current_state.data.end(), measure.begin(), measure.end());
		
		srvFK.request.joints = current_state;		
		clientFK.waitForExistence();		//Wait for client existence
		if(clientFK.call(srvFK)){					//Call the FK service 
			current_Pose = srvFK.response.poses[0];
		}
		else{
			std::cout<<"Error in FK solver"<<std::endl;
		};
		Pose[0] = current_Pose.position.x;
		Pose[2] = current_Pose.position.y;
		Pose[4] = current_Pose.position.z;
		return Pose;
	}
	
	void chatterCallback(const sensor_msgs::JointState& jointState){
		//std::cout<<jointState;
		pos = jointState.position;
		vel = jointState.velocity;
	};
	
	void calc(const ros::TimerEvent& event){
		if (t_dJS< 2.0){
		}
		else{
			(*myfile)<<sys1Pointer->Xcurr(0)<<","<<sys1Pointer->Xcurr(2)<<","<<sys1Pointer->Xcurr(4)<<",";
			arma::vec dot_curr = Jointstate2traj(pos) - shift;		//cout<<"sys1Pointer->Xcurr"<<endl;
			(*myfile)<<dot_curr(0)<<","<<dot_curr(2)<<","<<dot_curr(4)<<",";
			sys1Pointer->Xcurr(0) = dot_curr(0);
			sys1Pointer->Xcurr(2) = dot_curr(2);
			sys1Pointer->Xcurr(4) = dot_curr(4);
			costPointer->xmemory(sys1Pointer->Xcurr);				//cout<<"costPointer->xmemory"<<endl;
			sys1Pointer->step();													//cout<<"sys1Pointer->step()"<<endl;		
			sacPointer->SAC_calc();											//cout<<"sacPointer->SAC_calc();"<<endl;
			sys1Pointer->Ucurr = sacPointer->ulist.col(0); 	
			sacPointer->unom_shift();
			traj2Jointstate(sys1Pointer->Xcurr + shift);				//cout<<"traj2Jointstate"<<endl;
			for (int i=0; i<7; i++){
				(*myfile)<<dJS(i)<<",";
			}
			for (int i=0; i<7; i++){
				(*myfile)<<pos(i)<<",";
			}
			(*myfile)<<"\n";
		}
	}
	
	void publishing(const ros::TimerEvent& event){
		if (t_dJS< 2.0){
			t_dJS += 0.002;
			//dJS = {-0.37, 0.45, 0.96, -1.05, -0.37, 1.7, -2.};
			dJS = {0.6, 0.5, -0.6, -0.6, 0.5, 1.88, -2.8};
			//dJS = {-0.3, 0.4, 0.9, -1.0, -0.3, 1.5, -2.};
			//dJS = {-0.2, 0.2, 0.3, -0.5, -0.3, 0.5, -1.};
			//dJS = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		}
		output.data.clear(); 
		output.data.insert(output.data.end(), dJS.begin(), dJS.end());
		pub.publish(output);
	}
	
};


int main(int argc, char **argv){
	ros::init(argc, argv,"IK_pos_cl");
	ros::NodeHandle nh;
	const double DTP = 0.002;	//Time Duration for publishing position
	const double DT = 0.05;		//Time Duration for calculating desire joint state
	
	dot_model syst1 (DT);
    syst1.Ucurr = unom(0); 
    random_device rd; mt19937 eng(rd());
    uniform_real_distribution<> distr(-0.4,0.4);
    syst1.Xcurr = {-0.5,0.0,0.1,0.0, 0.1, 0.0};
    //syst1.Xcurr = {distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng)};//must be initialized before instantiating cost
    arma::mat R = 0.001*arma::eye(3,3); double q=1.;
    arma::vec umax = {5.0, 5.0, 5.0};
    double T = 0.6;
	double xbound = 0.25,ybound = 0.25,zbound = 0.25;
    arma::mat SIGMA = 0.01*arma::eye(3,3);	
    dklcost<dot_model> cost (q,R,100,SIGMA,0,2,4, phid,xbound,ybound,zbound,T,&syst1);	
    sac<dot_model,dklcost<dot_model>> sacsys (&syst1,&cost,0.,T,umax,unom);
	
	// define file for storing measurment
	ofstream myfile;
	myfile.open ("/home/zxl5344/test/src/alei/robotdata/IK_pos_cl.csv");
	
	// initialize the feight class
	figureight feight = figureight(&nh,&syst1, &cost, &sacsys, &myfile);	//std::cout<<"feight"<<std::endl;
	
	/*
	arma::vec trajectory = {0.6, 0.0, 0.7, 1, 0, 0, 0};
	// convert desire trajectory to desire joint state
	feight.traj2Jointstate(trajectory);		std::cout<<feight.traj2Jointstate(trajectory)<<std::endl;
	
	arma::vec Jointstate = {0.6, 0.0, 0.7, 1, 0, 0, 0};
	// convert desire trajectory to desire joint state
	feight.Jointstate2traj(Jointstate);		std::cout<<feight.Jointstate2traj(Jointstate)<<std::endl;
	*/
	
	// publish position
	ros::Timer timerP = nh.createTimer(ros::Duration(DTP), &figureight::publishing, &feight);
	
	// calculate desire position
	ros::Timer timer = nh.createTimer(ros::Duration(DT), &figureight::calc, &feight);
	
	ros::spin();		//spin command
	
	// close the file
	myfile.close();
	
	return 0;
}