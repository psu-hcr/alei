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
#include"src/traj_IK.hpp"

// nomial input u_norm
arma::vec unom(double t){
	return arma::zeros(7);};

class figureight{
	
	public:
	double t_dJS = 0.;		//time counter for desire joint state
	int counter = 2;		//counter for initial extra step
	ros::NodeHandle *nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::ServiceClient client;
	ros::ServiceClient clientFK;
	iiwa_tools::GetIK srv;
	iiwa_tools::GetFK srvFK;
	geometry_msgs::Pose desire_Pose;
	geometry_msgs::Pose current_Pose;
	std_msgs::Float64MultiArray seed1;
	std_msgs::Float64MultiArray output;
	std_msgs::Float64MultiArray current_state;
	arma::vec dJS = {0., 0., 0., 0., 0., 0., 0.};
	arma::vec Pose = {0.5, 0, 0.1, 0, 0.7, 0};
	arma::vec shift = {0.5, 0, 0.3, 0, 0.7, 0};
	arma::vec pos;
	arma::vec vel;
	arma::vec Xcurr;
	arma::vec Ucurr;
	ofstream *myfile;
	traj_IK *trajPointer;
		
	figureight(ros::NodeHandle *_nh,  traj_IK *_trajPointer, ofstream *_myfile){
		// initialization
		nh = _nh;
		trajPointer = _trajPointer;
		myfile = _myfile;
		Ucurr = arma::zeros(7);
		seed1.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(0)
		seed1.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(1)
		seed1.layout.dim[0].size = 1;
		seed1.layout.dim[1].size = 7;
		seed1.data = {0.33, 0.59, 0.5, -0.68, 2.85, 1.7, -2.};		// seed for IK solver
		current_state.layout.dim.push_back(std_msgs::MultiArrayDimension());		// setup dim(0)
		current_state.layout.dim.push_back(std_msgs::MultiArrayDimension());		// setup dim(1)
		current_state.layout.dim[0].size = 1;
		current_state.layout.dim[1].size = 7;
		current_state.data = {0., 0., 0., 0., 0., 0., 0.};		// robot state for FK solver
		output.layout.dim.push_back(std_msgs::MultiArrayDimension());	//setup dim(0)
		output.layout.dim[0].size = 7;
		output.data = {0., 0., 0., 0., 0., 0., 0.};
		client = nh->serviceClient<iiwa_tools::GetIK>("/iiwa/iiwa_ik_server");
		clientFK = nh->serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
		pub = nh->advertise<std_msgs::Float64MultiArray>("/iiwa/PositionController/command",10);
		sub = nh->subscribe("/iiwa/joint_states", 1000, &figureight::chatterCallback, this);
		
		// output data info to excel
		//(*myfile)<<"joint state 1, joint state 2, joint state 3, joint state 4, joint state 5, joint state 6, joint state 7,";
		//(*myfile)<<"DJS 1, DJS 2, DJS 3, DJS 4, DJS 5, DJS 6, DJS 7,";
		(*myfile)<<"x,y,z,";
		(*myfile)<<"\n";
		
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
	
	void traj2Jointstate(){
		int Jointstate_counter = 1;
		int total_step = trajPointer->total_step;
		while (Jointstate_counter < total_step){
			
			//desire trajectory
			arma::vec dpose = trajPointer->desire_traj(Jointstate_counter);	
			desire_Pose.position.x = dpose[1];
			desire_Pose.position.y = dpose[2];
			desire_Pose.position.z = dpose[3];
			desire_Pose.orientation.x = dpose[4];
			desire_Pose.orientation.y = dpose[5];
			desire_Pose.orientation.z = dpose[6];
			desire_Pose.orientation.w = dpose[7];//std::cout<<desire_Pose<<std::endl;
		
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
	
	void publishing(const ros::TimerEvent& event){
		Ucurr = trajPointer->desire_jointstate(t_dJS);
		t_dJS = t_dJS + 0.002;
		arma::vec dot_curr = Jointstate2traj(pos) - shift;	
		(*myfile)<<dot_curr(0)<<","<<dot_curr(2)<<","<<dot_curr(4)<<",";
		(*myfile)<<"\n";
		output.data.clear(); 
		output.data.insert(output.data.end(), Ucurr.begin(), Ucurr.end());
		pub.publish(output);
	}
	
};


int main(int argc, char **argv){
	ros::init(argc, argv,"f8IK");
	ros::NodeHandle nh;
	const double DTP = 0.002;	//Time Duration for publishing torque
	
	// traj data
	arma::mat Data;
	Data.load("/home/zxl5344/test/src/alei/robotdata/dklsac_dot.csv"); 	//std::cout<<"data.load"<<std::endl;
	
	// define desire traj class
	traj_IK traj(0.01, Data);	//std::cout<<"traj"<<std::endl;
	
	// define file for storing measurment
	ofstream myfile;
	myfile.open ("/home/zxl5344/test/src/alei/robotdata/f8IK_pos.csv");
	
	// initialize the feight class
	figureight feight = figureight(&nh,&traj,&myfile);	//std::cout<<"feight"<<std::endl;
	
	// convert desire trajectory to desire joint state
	feight.traj2Jointstate();		//std::cout<<"traj2Jointstate"<<std::endl;
	std::cout<<"traj2Jointstate is completed"<<std::endl;
	
	// save the desire joint_states
	feight.trajPointer->djsmat.save("/home/zxl5344/test/src/alei/robotdata/f8djs.csv", arma::csv_ascii);

	// publish torque
	ros::Timer timerP = nh.createTimer(ros::Duration(DTP), &figureight::publishing, &feight);
	
	ros::spin();		//spin command
	//spinner.spin();	//spin command for multithread
	
	// close the file
	myfile.close();
	
	return 0;
}