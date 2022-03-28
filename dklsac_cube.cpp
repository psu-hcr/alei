#include <iostream>
#include<string>
#include <fstream>
#include<math.h>
#include<armadillo>

using namespace std;

#include"src/dot_model.hpp"
#include"src/dkl_cost_cube.hpp"
#include"src/SAC.hpp"
#include"src/rk4_int.hpp"
const double PI = 3.1415926535987;

arma::vec unom(double t){
        return arma::zeros(3,1);
};

int main(){   
	ofstream myfile;
    myfile.open ("/home/zxl5344/test/src/alei/Gaussian_traj/cube_test.csv");
	arma::cube phid;
	phid.load("/home/zxl5344/test/src/alei/Gaussian_traj/subtask0.csv");
	cout<<arma::accu(phid)<<endl;
	cout<<phid(128, 139, 104)<<endl;
    dot_model syst1 (1./100.);
    syst1.Ucurr = unom(0); 
    random_device rd; mt19937 eng(rd());
    uniform_real_distribution<> distr(-0.4,0.4);
    syst1.Xcurr = {0.0,0.0,0.0,0.0,0.0,0.0};
    //syst1.Xcurr = {distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng)};//must be initialized before instantiating cost
    arma::mat R = 0.001*arma::eye(3,3); double q=1.;
    arma::vec umax = {5.0, 5.0, 5.0};
    double T = 0.6;
	double xbound = 0.5,ybound = 0.5,zbound = 1.2;
    arma::mat SIGMA = 0.01*arma::eye(3,3);	
    dkl_cost_cube<dot_model> cost (q,R,100,SIGMA,0,2,4, phid,xbound,ybound,zbound,T,&syst1);	
    sac<dot_model,dkl_cost_cube<dot_model>> sacsys (&syst1,&cost,0.,T,umax,unom);
 
    arma::vec xwrap;
	arma::vec shift = {0, 0, 0};
	//arma::vec shift = {0.5, 0.3, 0.7}
	
    //myfile<<"time,x,y,z,wx,wy,wz, w,\n";
 
    while (syst1.tcurr<5.){
		//double start_time = omp_get_wtime();
		cost.xmemory(syst1.Xcurr);	//cout<<"cost.xmemory"<<endl;
		//cout <<"resamp time: "<< 1000 * (omp_get_wtime() - start_time)<<endl;
		
		xwrap = syst1.proj_func(syst1.Xcurr); 
		myfile<<syst1.tcurr<<",";
		// move sys to desire location
		xwrap(0) = xwrap(0)+shift(0);
		xwrap(2) = xwrap(2)+shift(1);
		xwrap(4) = xwrap(4)+shift(2);
		myfile<<xwrap(0)<<","<<xwrap(2)<<","<<xwrap(4)<<",";
		myfile<<1<<","<<0<<","<<0<<","<<0<<",";
		myfile<<"\n";	
		syst1.step();	//cout<<"syst1.step()"<<endl;
		sacsys.SAC_calc();	//cout<<"sacsys.SAC_calc"<<endl;
		syst1.Ucurr = sacsys.ulist.col(0); 	//cout<<"syst1.Ucurr"<<endl;
		sacsys.unom_shift();  
		
		if(fmod(syst1.tcurr,2)<syst1.dt){
			cout<<"Time: "<<syst1.tcurr<<"\n"<<syst1.Xcurr<<"\n";
			cout<<"cost is "<<sacsys.J1new<<endl;
		}
    } 
      
	myfile.close();
	ofstream samples;
	samples.open("/home/zxl5344/test/src/alei/robotdata/Domain_samples.csv");
	cost.domainsamps.save(samples,arma::csv_ascii);
	arma::mat temp = cost.ps_i.t();
	temp.save(samples,arma::csv_ascii);
	samples.close();
    
}

