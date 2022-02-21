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

arma::vec unom(double t){
        return arma::zeros(2);};

int main(){   
	ofstream myfile;
        myfile.open ("/home/zxl5344/test/src/alei/robotdata/twolink.csv");
 	const double t_calc = 0.01;	// time for calculation
 	double T = 0.1;				// time horizon
 
 	// time step 
 	double dt = 1e-4;
 	
 	// define base function
 	twolinkbase tlbasis;
 	
 	// define koopman system
 	KoopSys<twolinkbase> systK(dt,&tlbasis); 
 	
 	// define desire traj class
	// for sys1
	twolinktraj tltraj(dt, 4);
	// for sysK
	//twolinktraj tltraj(dt, tlbasis.xdim);
 
 	// initial condition
 	arma::vec Xinit = {0.01, 0, 0.01, 0};
	arma::vec Uinit = {0, 0};
 	systK.Xcurr = tlbasis.zx(Xinit);
 	systK.Ucurr = tlbasis.zu(Xinit, Uinit);
 
 	// define system
        LinearTL syst1(dt, Xinit); // a linear system
 	//twoLink syst1(dt, Xinit);	// a nonlinear system
 
 	// define maximun input
	arma::vec umax = {1e99, 1e99};
 	
 	// define error cost gain 
	// for sys1
        arma::mat R = 1*arma::eye(2,2);
        arma::vec Qvec = 10*arma::ones(4);
 	arma::mat Qk = arma::diagmat(Qvec);	//std::cout<<Qk;
	arma::mat Qf = arma::zeros<arma::mat>(size(Qk));
	
	/*
	// for sysK
	arma::mat R = 0.001*arma::eye(tlbasis.udim, tlbasis.udim);
 	arma::mat Qk = arma::zeros(tlbasis.xdim,tlbasis.xdim);
	arma::vec Qvec = arma::ones(4);
	Qk.submat(0,0,3,3)=200*arma::diagmat(Qvec);//std::cout<<Qk;
	arma::mat Qf = arma::zeros<arma::mat>(size(Qk));
	*/
 
 	// define joint limit
	arma::vec Joint_limit = {2.94, 2.09};
 
 	// define LQR controller
	LQR_traj<twolinktraj> lqrK(Qk,R,Qf,round(T/(1e-4)),umax,&tltraj,1e-4);
	
	/*
 	// linearlize the system around {0, 0, 0, 0}
 	//arma::mat A = {{0, 1, 0, 0},{0, 0, 0, 0},{0, 0, 0, 1}, {0, 0, 0, 0}}; cout<<A<<endl;
 	//arma::mat B = {{0, 0}, {400, 0}, {0, 0}, {0, 26.67}}; cout<<B<<endl;
	arma::vec X_0 = {0,0,0,0};
 	arma::mat A = syst1.dfdx(X_0); cout<<A<<endl;
 	arma::mat B = syst1.hx(X_0); cout<<B<<endl;
	*/

        myfile<<"time,theta1,thetadot1,theta2,thetadot2,u1,u2,dtheta1,dthetadot1,dtheta2,dthetadot2 \n";
 	
	/*
 	// calculate lqr gain
	lqrK.calc_gains(A,B,systK.tcurr); cout<<"lqr"<<endl; // move it inside loop
	*/
 	
        while (syst1.tcurr<10e4*dt){
		
		// print out current time
		cout<<"Time: "<<syst1.tcurr<<"\n";
		
		// step function
		syst1.step(); //cout<<"step"<<endl;
		
		// calculate Koopman operator
		systK.calc_K(syst1.Xcurr,syst1.Ucurr); //cout<<"KO"<<endl;
		
		// calculate lqr gain using current dfdx and hx
		// for sys1
		lqrK.calc_gains(syst1.dfdx(syst1.Xcurr),syst1.hx(syst1.Xcurr,syst1.Ucurr),syst1.tcurr); //cout<<"lqr"<<endl;
		// calculate lqr gain using Kx and Ku
		// for sysK
		//lqrK.calc_gains(systK.Kx,systK.Ku,systK.tcurr); //cout<<"lqr"<<endl;
		
		// calculate input
		// for sys1
		syst1.Ucurr = lqrK.mu(syst1.Xcurr,syst1.tcurr); //cout<<"syst1.Ucurr"<<endl;
		// for sysK
		//syst1.Ucurr = lqrK.mu(systK.Xcurr,systK.tcurr); //cout<<"syst1.Ucurr"<<endl;
		//cout<<lqrK.K(syst1.tcurr)<<endl;
		
		// check input is not nan
		if(syst1.Ucurr(0)!=syst1.Ucurr(0)){
			cout<<"returned a nan"<<endl;
			syst1.Ucurr = unom(syst1.tcurr);
		}
		
		
		myfile<<syst1.tcurr<<",";
		myfile<<syst1.Xcurr(0)<<","<<syst1.Xcurr(1)<<",";
		myfile<<syst1.Xcurr(2)<<","<<syst1.Xcurr(3)<<",";
		myfile<<syst1.Ucurr(0)<<","<<syst1.Ucurr(1)<<",";
		arma::vec desire = tltraj.desire_jointstate(syst1.tcurr);
		myfile<<desire(0)<<","<<desire(1)<<",";
		myfile<<desire(2)<<","<<desire(3);
		myfile<<"\n";
		//cout<<lqrK.dmudz(systK.Xcurr,systK.tcurr)<<endl;
		
		/*
		// test eigen value
		arma::mat test = A-B*lqrK.K(syst1.tcurr);
		arma::cx_vec eigval; arma::cx_mat eigvec;
		eig_gen(eigval, eigvec, test);
		cout<<"eigval"<<eigval<<endl;*/
		
		// check controllability
		/*
		arma::mat controlmat = arma::zeros(13, 13*2);
		controlmat.submat(0,0,12,1) = systK.Ku;
		controlmat.submat(0,2,12,3) = systK.Kx*systK.Ku;
		controlmat.submat(0,4,12,5) = pow(systK.Kx,2)*systK.Ku;
		controlmat.submat(0,6,12,7) = pow(systK.Kx,3)*systK.Ku;
		controlmat.submat(0,8,12,9) = pow(systK.Kx,4)*systK.Ku;
		controlmat.submat(0,10,12,11) = pow(systK.Kx,5)*systK.Ku;
		controlmat.submat(0,12,12,13) = pow(systK.Kx,6)*systK.Ku;
		controlmat.submat(0,14,12,15) = pow(systK.Kx,7)*systK.Ku;
		controlmat.submat(0,16,12,17) = pow(systK.Kx,8)*systK.Ku;
		controlmat.submat(0,18,12,19) = pow(systK.Kx,9)*systK.Ku;
		controlmat.submat(0,20,12,21) = pow(systK.Kx,10)*systK.Ku;
		controlmat.submat(0,22,12,23) = pow(systK.Kx,11)*systK.Ku;
		controlmat.submat(0,24,12,25) = pow(systK.Kx,12)*systK.Ku;
		cout<<"rank"<<arma::rank(controlmat)<<endl;*/
        }
	
        // save final Koopman opeartor
	systK.K.save("/home/zxl5344/test/src/alei/robotdata/LQR_twolink_KO_final.csv", arma::csv_ascii);
    
        myfile.close();
}

