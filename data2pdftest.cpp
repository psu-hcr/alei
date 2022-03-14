#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <armadillo>
#include <bits/stdc++.h>
#include <chrono> 
using namespace std;

#include"src/data2pdf.hpp"
#include"src/dot_model.hpp"
#include"src/dkl_cost_pdf.hpp"
#include"src/SAC.hpp"

arma::vec unom(double t){
        return arma::zeros(3,1);
};

int main(){ 
	arma::mat Data1, Data2, Data3;
	Data1.load("/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample3.csv"); 	
	double L1 = 0.8;
	double L2 = 0.5;
	double L3 = 1.3;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	data2pdf phid(Data1, Data2, Data3, L1, L2, L3, dL1, dL2, dL3);
	phid.calcpdf();
	//cout<<phid.phi(90, 113, 225)<<endl;
	cout<<"sum of phi: "<<arma::accu(phid.phi)<<endl;
	//phid.phi.save("/home/zxl5344/test/src/alei/Sweeping data/phi.csv", arma::arma_ascii);
	arma::vec pos = {0., 0.,  0.};
	cout<<"read phi at(0., 0.,  0.): "<<phid.readpdf(pos)<<endl;
	
	dot_model syst1 (1./100.);
    syst1.Ucurr = unom(0); 
    random_device rd; mt19937 eng(rd());
    uniform_real_distribution<> distr(-0.4,0.4);
    //syst1.Xcurr = {-0.2,0.0,0.1,0.0};
    syst1.Xcurr = {distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng)};//must be initialized before instantiating cost
    arma::mat R = 0.001*arma::eye(3,3); double q=1.;
    arma::vec umax = {5.0, 5.0, 5.0};
    double T = 0.6;
    arma::mat SIGMA = 0.01*arma::eye(3,3);	
    dklcost_pdf<dot_model, data2pdf> cost (q,R,100,SIGMA,0,2,4, &phid, L1,L2,L3,T,&syst1);	
    sac<dot_model, dklcost_pdf<dot_model, data2pdf>> sacsys (&syst1,&cost,0.,T,umax,unom);
	
	ofstream myfile;
    myfile.open ("/home/zxl5344/test/src/alei/robotdata/data2pdftest.csv");
	
	arma::vec xwrap;
	myfile<<"time,x,y,z,wx,wy,wz, w,\n";
	
	while (syst1.tcurr<100.){
		//double start_time = omp_get_wtime();
		cost.xmemory(syst1.Xcurr);	//cout<<"cost.xmemory"<<endl;
		//cout <<"resamp time: "<< 1000 * (omp_get_wtime() - start_time)<<endl;
		if(fmod(syst1.tcurr,2)<syst1.dt){
			cout<<"Time: "<<syst1.tcurr<<"\n"<<syst1.Xcurr<<"\n";
		}
		xwrap = syst1.proj_func(syst1.Xcurr); 
		myfile<<syst1.tcurr<<",";
		// move sys to desire location
		myfile<<xwrap(0)<<","<<xwrap(2)<<","<<xwrap(4)<<",";
		myfile<<1<<","<<0<<","<<0<<","<<0<<",";
		myfile<<"\n";	
		syst1.step();	//cout<<"syst1.step()"<<endl;
		sacsys.SAC_calc();	//cout<<"sacsys.SAC_calc"<<endl;
		syst1.Ucurr = sacsys.ulist.col(0); 	//cout<<"syst1.Ucurr"<<endl;
		sacsys.unom_shift();  
    } 
}