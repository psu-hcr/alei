#include <iostream>
#include <fstream>
#include<math.h>
#include<armadillo>
using namespace std;
#include"src/twolink.hpp"
#include"src/lineartl.hpp"
#include"src/rk4_int.hpp"
#include"src/koopsys.hpp"
#include"src/twolinkbase.hpp"
#include"src/twolinktraj.hpp"
#include"src/LQR_traj.hpp"
#include"src/fisher_cost.hpp"
#include"src/ActLearnK.hpp"

// This code use data and Koopman operator calculated by that data to do simulation
// It can also use data to calculate a Koopman 

int main(){
	
	// define base function
 	twolinkbase tlbasis;
		
	// import KO from file 
	arma::mat K;
	K.load("/home/zxl5344/test/src/alei/robotdata/LQR_twolink_KO_final.csv");
	
	// import data from file
	// Note: if first row of cvs is header, actual data start from second row
	arma::mat Data;
	Data.load("/home/zxl5344/test/src/alei/robotdata/twolink.csv"); 
	
	// time step
	double dt = Data(1,0);
	
	// K_x and K_u
	arma::mat K_x = K.submat(0,0,tlbasis.xdim-1,tlbasis.xdim-1);
	arma::mat K_u = K.submat(0,tlbasis.xdim,tlbasis.xdim-1,K.n_cols-1);
	
	// initalize X and input
	arma::vec X_init = {Data(1,1), Data(1,2), Data(1,3), Data(1,4)};
	
	// convert X and u into zx and zu
	arma::vec zx = tlbasis.zx(X_init);
	
	// open a file to store simulation output
	ofstream myfile;
    myfile.open ("/home/zxl5344/test/src/alei/robotdata/KO_test_simulation.csv");
	myfile<<"time,theta1,thetadot1,theta2,thetadot2\n";
	
	// counter for simulation
	int counter = 1;
	
	// simulation
	while(counter<Data.n_rows){
		
		// update zu
		arma::vec U_curr = {Data(counter,5), Data(counter,6)};
		arma::vec zu = tlbasis.zu(X_init, U_curr);
		
		// simulate zx
		arma::vec f1 = K_x*zx + K_u*zu;				//cout<<"f1"<<endl;
		arma::vec f2 = K_x*(zx+1/2*f1) + K_u*zu; 	//cout<<"f2"<<endl;
		arma::vec f3 = K_x*(zx+1/2*f2) + K_u*zu;	//cout<<"f3"<<endl;
		arma::vec f4 = K_x*(zx+f3) + K_u*zu;		//cout<<"f4"<<endl;
		zx = zx + ((f1/6.0)+(f2/3.0)+(f3/3.0)+(f4/6.0))*dt;
		
		// write data into file
		myfile<<counter<<",";
		myfile<<zx(0)<<","<<zx(1)<<","<<zx(2)<<","<<zx(3);
		myfile<<"\n";
		
		counter++;
	}
		
}