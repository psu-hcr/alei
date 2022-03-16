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
	
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/sample3.csv"); 	
	
	double L1 = 0.5;
	double L2 = 0.5;
	double L3 = 0.5;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	arma::vec new_origin = {0, 0, 0};
	
	data2pdf_KL phid(Data1, Data2, Data3, L1, L2, L3, dL1, dL2, dL3, new_origin);
	phid.calcpdf();
	//cout<<phid.phi(90, 113, 225)<<endl;
	cout<<"sum of phi: "<<arma::accu(phid.phi)<<endl;
	//phid.phi.save("/home/zxl5344/test/src/alei/Sweeping data/phi.csv", arma::arma_ascii);
	arma::vec pos = {0., 0.,  0.};
	cout<<"read phi at(0., 0.,  0.): "<<phid.readpdf(pos)<<endl;
	
}