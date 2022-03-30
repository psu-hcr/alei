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
#include"src/data2pdf_auto.hpp"
#include"src/dot_model.hpp"
#include"src/dkl_cost_pdf.hpp"
#include"src/SAC.hpp"

arma::vec unom(double t){
        return arma::zeros(3,1);
};

int main(){ 
	
	arma::mat Data1, Data2, Data3;

	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_screw4_nonstop.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_screw5_nonstop.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_screw6_nonstop.csv"); 
	
	double L1 = 0.5;
	double L2 = 0.5;
	double L3 = 0.5;
	double dL1 = 0.005;
	double dL2 = 0.005;
	double dL3 = 0.005;
	arma::vec new_origin = {0, 0, 0.7};
	
	/*
	ofstream KL1, KL2, KL3;
	KL1.open("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL1.csv");
	KL2.open("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL2.csv");
	KL3.open("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL3.csv");
	
	data2pdf_auto phid1(Data1,L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_auto phid2(Data2,L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_auto phid3(Data3,L1, L2, L3, dL1, dL2, dL3, new_origin);
	int T = 0.5*50; // Sampling freq is 50HZ 
	arma::cube P, Q;
	
	
	P = phid1.pdf_t(0, T);
	for (int i =1; i<Data1.n_rows;i++){

		Q = phid1.pdf_t(i, T+i);
		double cost = phid1.KL(P, Q) + phid1.KL(Q, P);
		KL1<<i<<","<<cost;
		KL1<<"\n";
	}
	
	
	P = phid2.pdf_t(0, T);
	for (int i =1; i<Data2.n_rows;i++){

		Q = phid2.pdf_t(i, T+i);
		double cost = phid2.KL(P, Q) + phid2.KL(Q, P);
		KL2<<i<<","<<cost;
		KL2<<"\n";
	}
	
	P = phid3.pdf_t(0, T);
	for (int i =1; i<Data3.n_rows;i++){

		Q = phid3.pdf_t(i, T+i);
		double cost = phid3.KL(P, Q) + phid3.KL(Q, P);
		KL3<<i<<","<<cost;
		KL3<<"\n";
	}
	
	P = phid1.pdf_t(0, T)+phid2.pdf_t(0, T)+phid3.pdf_t(0, T);
	P = P/arma::accu(P);	// normalize
	for (int i =1; i<Data1.n_rows;i++){
		if( (i<Data2.n_rows)&&(i<Data3.n_rows)){
			Q = phid1.pdf_t(i, T+i)+phid2.pdf_t(i, T+i)+phid3.pdf_t(i, T+i);
			Q = Q/arma::accu(Q);
			double cost = phid1.KL(P, Q) + phid1.KL(Q, P);
			KL1<<i<<","<<cost;
			KL1<<"\n";
		}
		
	}
	
	KL1.close();
	KL2.close();
	KL3.close();
	*/
	
	dot_model syst1 (1./100.);
    syst1.Ucurr = unom(0); 
    random_device rd; mt19937 eng(rd());
    uniform_real_distribution<> distr(-0.4,0.4);
    syst1.Xcurr = {distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng)};//must be initialized before instantiating cost
    arma::mat R = 0.001*arma::eye(3,3); double q=1.;
    arma::vec umax = {5.0, 5.0, 5.0};
    double T = 0.6;
    arma::mat SIGMA = 0.01*arma::eye(3,3);
	data2pdf phid(Data1,Data2,Data3,L1, L2, L3, dL1, dL2, dL3, new_origin);
	int samples = 1.0*50; // Sampling freq is 50HZ
	cout<<"samples size: "<<samples<<endl;
	phid.calcpdf(samples);
	dklcost_pdf<dot_model, data2pdf> cost(q,R,10000000,SIGMA,0,2,4, &phid,L1,L2,L3,T,&syst1);
	ofstream KL1;
	KL1.open("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL1.csv");
	arma::vec Xcurr = {Data2(0, 1), 0, Data2(0, 2), 0,Data2(0, 3)-0.7, 0};
	cost.xmemory(Xcurr);
	for (int i =1; i<Data2.n_rows-samples;i++){

		arma::mat traj = arma::zeros(6, samples);
		for (int j=0; j<traj.n_cols;j++){
			arma::vec new_Xcurr = {Data2(i+j, 1), 0, Data2(i+j, 2), 0,Data2(i+j, 3)-0.7, 0};
			traj.col(j) = new_Xcurr;
		}
		arma::mat Umat = arma::zeros(3, samples);					//cout<<"Umat"<<endl;
		double C = cost.calc_cost(traj, Umat);									//cout<<"calc_cost"<<endl;
		if (C>0.00){
			KL1<<i<<","<<C;
			KL1<<"\n";
		}
		
		if(fmod(i,10)==0){
			cout<<"i: "<<i<<"\n"<<"cost: "<<C<<"\n";
		}
	}
	KL1.close();
	
}