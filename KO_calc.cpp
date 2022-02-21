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

// This code use data to calculate a Koopman operator 

int main(){
	
	// import data from file
	// Note: if first row of cvs is header, actual data start from second row
	arma::mat Data;
	Data.load("/home/zxl5344/test/src/alei/robotdata/twolink.csv"); 
	
	// time step
	double dt = Data(1,0);
	
	// define base function
 	twolinkbase tlbasis;
	
	// define koopman system
 	KoopSys<twolinkbase> systK(dt,&tlbasis); 
	
	// initial condition
 	arma::vec Xinit = {Data(1,1), Data(1,2), Data(1,3), Data(1,4)};
	arma::vec Uinit = {0, 0};
 	systK.Xcurr = tlbasis.zx(Xinit);
 	systK.Ucurr = tlbasis.zu(Xinit, Uinit);
	
	// open a file to store simulation output
	ofstream myfile;
    myfile.open ("/home/zxl5344/test/src/alei/robotdata/KO_test_simulation.csv");
	myfile<<"time,theta1,thetadot1,theta2,thetadot2\n";
	
	// counter for simulation
	int counter = 1;
	
	// simulation
	while(counter<Data.n_rows){
		
		arma::vec Xcurr = {Data(counter,1), Data(counter,2), Data(counter,3), Data(counter,4)};
		arma::vec Ucurr = {Data(counter,5), Data(counter,6)};
		
		// calculate Koopman operator
		systK.calc_K(Xcurr,Ucurr); //cout<<"KO"<<endl;
		
		counter++;
	}
	
	// save final Koopman opeartor
	systK.K.save("/home/zxl5344/test/src/alei/robotdata/KO_calc.csv", arma::csv_ascii);
		
}