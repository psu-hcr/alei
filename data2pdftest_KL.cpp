#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <armadillo>
#include <bits/stdc++.h>
#include <chrono> 
using namespace std;

#include"src/data2pdf_KL.hpp"
#include"src/dot_model.hpp"
#include"src/dkl_cost_pdf.hpp"
#include"src/SAC.hpp"

arma::vec unom(double t){
        return arma::zeros(3,1);
};

int main(){ 
	arma::mat Data1, Data2, Data3;
	/*
	// gaussian traj data
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample3.csv"); 	
	*/
	/*
	// 3point data without noise
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample1_no_noise.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample2_no_noise.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample3_no_noise.csv"); 
	*/
	/*
	// 3point data
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample3.csv"); 	
	*/
	// Camera data
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv"); 	
	
	double L1 = 2.5;
	double L2 = 2.5;
	double L3 = 2.5;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	arma::vec new_origin = {0., 0., 0.};
	
	ofstream myfile;
	myfile.open ("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL.csv");
	myfile<<"row, cost,";
	myfile<<"\n";
	
	data2pdf_KL phid(Data1, Data2, Data3, L1, L2, L3, dL1, dL2, dL3, new_origin);
	arma::cube Q = phid.calcpdf();
	
	// define window size
	int w = 20;
	
	int i = 0;
	double previous_cost = 0;
	//arma::cube prev_P = arma::zeros(size(Q));
	arma::cube prev_P = phid.pdf_t(i, i+w);
	
	// calculate PDF
	while(i < Data1.n_rows){
		arma::cube P = phid.pdf_t(i, i+w);	
		//arma::cube P = phid.pdf_t(0, i+w);
		//double cost = phid.KL(P,Q);
		double cost = phid.KL(prev_P,P);
		double result = cost;
		//cout<<result<<endl;
		myfile<<i<<","<<result<<",";
		
		if(cost-previous_cost > 12) cout<<"a new task at "<<i<<endl;
		
		previous_cost = cost;
		prev_P = P;
		myfile<<"\n";
		i += w;
	}
	
}