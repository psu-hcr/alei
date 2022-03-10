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
	double L1 = 1;
	double L2 = 1;
	double L3 = 1.5;
	double dL1 = 0.005;
	double dL2 = 0.005;
	double dL3 = 0.005;
	data2pdf phid(Data1, Data2, Data3, L1, L2, L3, dL1, dL2, dL3);
	phid.calcpdf();
	//cout<<phid.phi(90, 113, 225)<<endl;
	cout<<"sum of phi: "<<arma::accu(phid.phi)<<endl;
	//phid.phi.save("/home/zxl5344/test/src/alei/Sweeping data/phi.csv", arma::arma_ascii);
	arma::vec pos = {0.609, -0.029,  0.793};
	cout<<"read phi at(0.609, -0.029,  0.793): "<<phid.readpdf(pos)<<endl;
	
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
}