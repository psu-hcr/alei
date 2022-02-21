#include <iostream>

#include <fstream>
#include<math.h>
#include<armadillo>
using namespace std;

#include"src/twolink.hpp"
#include"src/error_cost_IK.hpp"
#include"src/SAC.hpp"
#include"src/rk4_int.hpp"
#include"src/koopsys.hpp"
#include"src/twolinkbase.hpp"
#include"src/twolinktraj.hpp"

arma::vec unom(double t){
        return arma::zeros(2);};

int main()
{   ofstream myfile;
    myfile.open ("/home/zxl5344/test/src/alei/robotdata/twolink.csv");
 	const double t_calc = 0.1;	// time for calculation
 	const double T = 0.25;		// Horizon for SAC
 	
 	// time step 
 	double dt = 0.05;
 
 	// define base function
 	twolinkbase tlbasis;
 	
 	// define koopman system
 	KoopSys<twolinkbase> systK(dt,&tlbasis);
 	
 	// define desire traj class
	twolinktraj tltraj(dt, tlbasis.xdim);
 
 	// initial condition
 	arma::vec Xinit = {0, 0, 0, 0};
 	systK.Xcurr = Xinit;
 	systK.Ucurr = {0, 0};
 
 
 	// define system
    twoLink syst1(dt, Xinit);
 
 	// define maximun input
	arma::vec umax = {0.001, 0.001};
 
 	// define error cost gain
    arma::mat R = 0.001*arma::eye(tlbasis.udim, tlbasis.udim);
 	arma::mat Qk = arma::zeros(tlbasis.xdim,tlbasis.xdim);
	arma::vec Qvec = arma::ones(4);
	Qk.submat(0,0,3,3)=200*arma::diagmat(Qvec);//std::cout<<Qk;
	arma::mat Qf = arma::zeros<arma::mat>(size(Qk));
	arma::vec noisecov = 1.0*arma::ones(tlbasis.xdim);	// diagonal for noise convariance for active learning
 
 	// define joint limit
	arma::vec Joint_limit = {2.94, 2.09};
 
 	// define cost function 
	errorcostIK<KoopSys<twolinkbase>,twolinktraj> costK(Qk,R,&tltraj,&systK,noisecov, Joint_limit);
 
	// define sac controllor
	sac<KoopSys<twolinkbase>,errorcostIK<KoopSys<twolinkbase>,
 			twolinktraj>> sacsysK (&systK,&costK,t_calc,T,umax,unom);
 

    myfile<<"time,theta1,thetadot1,theta2,thetadot2,u1,u2,dtheta1,dtheta2, dthetadot1, dthetadot2\n";
 	
    while (syst1.tcurr<50*dt){
		syst1.step(); 
		systK.calc_K(syst1.Xcurr,syst1.Ucurr);
		
		/*
		// cosine input
		syst1.Ucurr = {0.0001*cos(syst1.tcurr), 0.0001*cos(syst1.tcurr)};
		*/
		/*
		// SAC input
		sacsysK.SAC_calc(); 
		syst1.Ucurr = sacsysK.ulist.col(0); cout<<syst1.Ucurr<<endl;
		sacsysK.unom_shift();
		*/
		
		/*
		// Cosine input, after 100dt change to SAC
		if (syst1.tcurr<100*dt){
			syst1.Ucurr = {0.0001*cos(syst1.tcurr), 0.0001*cos(syst1.tcurr)};
		}
		else{
			sacsysK.SAC_calc(); 
			syst1.Ucurr = sacsysK.ulist.col(0); cout<<syst1.Ucurr<<endl;
			sacsysK.unom_shift();
		}*/
		
		cout<<"Time: "<<syst1.tcurr<<"\n";
		myfile<<syst1.tcurr<<",";
		myfile<<syst1.Xcurr(0)<<","<<syst1.Xcurr(1)<<",";
		myfile<<syst1.Xcurr(2)<<","<<syst1.Xcurr(3)<<",";
		myfile<<syst1.Ucurr(0)<<","<<syst1.Ucurr(1)<<",";
		arma::vec desire = tltraj.desire_jointstate(syst1.tcurr);
		myfile<<desire(0)<<","<<desire(1)<<",";
		myfile<<desire(2)<<","<<desire(3)<<",";
		myfile<<"\n";
    } 
    
    myfile.close();
}

