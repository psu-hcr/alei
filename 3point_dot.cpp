#include <iostream>
#include<string>
#include <fstream>
#include<math.h>
#include<armadillo>

using namespace std;

#include"src/dot_model.hpp"
#include"src/dkl_cost.hpp"
#include"src/SAC.hpp"
#include"src/rk4_int.hpp"
const double PI = 3.1415926535987;

arma::vec unom(double t){
        return arma::zeros(3);};
		
int main(){   
	ofstream myfile;
    myfile.open ("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample3.csv");
    dot_model syst1 (1./100.);
    syst1.Ucurr = unom(0); 
    random_device rd; mt19937 eng(rd());
    uniform_real_distribution<> distr(-0.4,0.4);
	uniform_real_distribution<> noise_distr(-0.01,0.01);
    syst1.Xcurr = {-0.2,0.0,0.1,0.0, 0.1, 0.0};
    //syst1.Xcurr = {distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng)};//must be initialized before instantiating cost
    arma::mat R = 0.001*arma::eye(3,3); double q=1.;
    arma::vec umax = {5.0, 5.0, 5.0};
 
    arma::vec xwrap;
	arma::vec goal = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
	arma::mat K = {
		{3., 3., 0., 0., 0., 0.},
		{0., 0., 3., 3, 0., 0.},
		{0., 0., 0., 0., 3., 3.},
	};
	
    //myfile<<"time,x,y,z,wx,wy,wz, w,\n";
 
    while (syst1.tcurr<30.){
		if(fmod(syst1.tcurr,2)<syst1.dt){
			cout<<"Time: "<<syst1.tcurr<<"\n"<<syst1.Xcurr<<"\n";
		}
		xwrap = syst1.proj_func(syst1.Xcurr); 
		myfile<<syst1.tcurr<<",";
		// move sys to desire location
		xwrap(0) = xwrap(0);
		xwrap(2) = xwrap(2);
		xwrap(4) = xwrap(4);
		myfile<<xwrap(0)<<","<<xwrap(2)<<","<<xwrap(4)<<",";
		myfile<<1<<","<<0<<","<<0<<","<<0<<",";
		myfile<<"\n";	
		syst1.step();	//cout<<"syst1.step()"<<endl;
		arma::vec noise = {noise_distr(eng),noise_distr(eng),noise_distr(eng),noise_distr(eng),noise_distr(eng),noise_distr(eng)};
		syst1.Xcurr = syst1.Xcurr + noise;
		syst1.Ucurr = -K*(syst1.Xcurr-goal);	//cout<<"syst1.Ucurr"<<endl;
		
		// switch goal state
		if(syst1.tcurr<5.){
			goal = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
		}
		else if (syst1.tcurr<10.){
			goal = {2.0, 0.0, -1.0, 0.0, -1.0, 0.0};
		}
		else if (syst1.tcurr<15.){
			goal = {-1.0, 0.0, 1.0, 0.0, -1.0, 0.0};
		}
		else if(syst1.tcurr<20.){
			goal = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
		}
		else if (syst1.tcurr<25.){
			goal = {2.0, 0.0, -1.0, 0.0, -1.0, 0.0};
		}
		else if (syst1.tcurr<30.){
			goal = {-1.0, 0.0, 1.0, 0.0, -1.0, 0.0};
		}
    } 
      
	myfile.close();
    
}

