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
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample3.csv"); 	
	*/
	
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/3dotsample3.csv"); 	
	
	double L1 = 2.5;
	double L2 = 2.5;
	double L3 = 2.5;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	arma::vec new_origin = {0, 0, 0};
	
	ofstream myfile;
	myfile.open ("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL.csv");
	myfile<<"row, cost,";
	myfile<<"\n";
	
	data2pdf_KL phid(Data1, Data2, Data3, L1, L2, L3, dL1, dL2, dL3, new_origin);
	arma::cube Q = phid.calcpdf();
	
	// define window size
	int w = 50;
	
	int i = w;
	double previous_cost = 0;
	
	// calculate PDF
	while(i < Data1.n_rows){
		arma::cube P = phid.pdf_t(i);
		double cost = phid.KL(P,Q);
		cout<<cost<<endl;
		myfile<<i<<","<<cost<<",";
		
		previous_cost = cost;
		myfile<<"\n";
		i +=w;
	}
	
}