#include <iostream>

#include <fstream>
#include<math.h>
#include<armadillo>
using namespace std;

#include"src/twolink.hpp"
#include"src/rk4_int.hpp"
#include"src/koopsys.hpp"
#include"src/twolinkbase.hpp"
#include"src/twolinktraj.hpp"
#include"src/LQR_traj.hpp"
#include"src/fisher_cost.hpp"
#include"src/ActLearnK.hpp"

arma::vec unom(double t){
        return arma::zeros(2);};

int main()
{   ofstream myfile;
    myfile.open ("/home/zxl5344/test/src/alei/robotdata/twolink.csv");
 	const double t_calc = 0.01;	// time for calculation
 	double T = 0.25;				// time horizon
 
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
	arma::vec umax = {1., 1};
 
 	// define error cost gain
    arma::mat R = 0.001*arma::eye(tlbasis.udim, tlbasis.udim);
 	arma::mat Qk = arma::zeros(tlbasis.xdim,tlbasis.xdim);
	arma::vec Qvec = arma::ones(4);
	Qk.submat(0,0,3,3)=200*arma::diagmat(Qvec);//std::cout<<Qk;
	arma::mat Qf = arma::zeros<arma::mat>(size(Qk));
	arma::vec noisecov = 1.0*arma::ones(tlbasis.xdim);	// diagonal for noise convariance for active learning
 	arma::mat Rtil = 1.*arma::eye(systK.Ucurr.n_rows,systK.Ucurr.n_rows);
 
 	// define joint limit
	arma::vec Joint_limit = {2.94, 2.09};
 
 	// define LQR controller
	LQR_traj<twolinktraj> lqrK(Qk,R,Qf,round(T/dt),umax,&tltraj,dt);
 	
 	// define fisher cost and active learning controller
 	fishcost<KoopSys<twolinkbase>,LQR_traj<twolinktraj>> costFI (&systK,&lqrK,noisecov);
 	alk<KoopSys<twolinkbase>,fishcost<KoopSys<twolinkbase>,
 			LQR_traj<twolinktraj>>,LQR_traj<twolinktraj>> ALpol(&systK,&costFI,&lqrK,T,umax,Rtil);
 
 

    myfile<<"time,theta1,thetadot1,theta2,thetadot2,u1,u2,dtheta1,dtheta2, dthetadot1, dthetadot2, mu1, mu2\n";
 	
    while (syst1.tcurr<800*dt){
		// step function
		syst1.step(); //cout<<"step"<<endl;
		
		// calculate Koopman operator
		systK.calc_K(syst1.Xcurr,syst1.Ucurr); //cout<<"KO"<<endl;
		
		// calculate lqr gain
		lqrK.calc_gains(systK.Kx,systK.Ku,systK.tcurr); //cout<<"lqr"<<endl;
		
		// compute ustar
		syst1.Ucurr = ALpol.ustar_calc(); //cout<<"ALpol"<<endl;
		
		costFI.infw = 1000.0 * pow(0.8,systK.tcurr); //cout<<"costFI.infw"<<endl;
		
		// check input is not nan
		if(syst1.Ucurr(0)!=syst1.Ucurr(0)){
			cout<<"returned a nan"<<endl;
			syst1.Ucurr = unom(syst1.tcurr);
		}
		
		cout<<"Time: "<<syst1.tcurr<<"\n";
		myfile<<syst1.tcurr<<",";
		myfile<<syst1.Xcurr(0)<<","<<syst1.Xcurr(1)<<",";
		myfile<<syst1.Xcurr(2)<<","<<syst1.Xcurr(3)<<",";
		myfile<<syst1.Ucurr(0)<<","<<syst1.Ucurr(1)<<",";
		arma::vec desire = tltraj.desire_jointstate(syst1.tcurr);
		myfile<<desire(0)<<","<<desire(1)<<",";
		myfile<<desire(2)<<","<<desire(3)<<",";
		arma::vec mu = lqrK.mu(systK.Xcurr,systK.tcurr);
		myfile<<mu(0)<<","<<mu(1);
		myfile<<"\n";
		//cout<<lqrK.dmudz(systK.Xcurr,systK.tcurr)<<endl;
		
		/*
		// test eigen value
		arma::mat test = systK.Kx+systK.Ku*lqrK.dmudz(systK.Xcurr,systK.tcurr);
		arma::cx_vec eigval; arma::cx_mat eigvec;
		eig_gen(eigval, eigvec, test);
		cout<<"eigval"<<eigval<<endl;*/
		
		// check controllability
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
		cout<<"rank"<<arma::rank(controlmat)<<endl;
    } 
    
    myfile.close();
}

