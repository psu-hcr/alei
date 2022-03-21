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

int main(){ 
	arma::mat Data1, Data2, Data3;
	arma::mat seg1, seg2, seg3;

	// Camera data
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv"); 	
	
	// seg data
	seg1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1_seg.csv"); 	
	seg2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2_seg.csv"); 	
	seg3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3_seg.csv"); 	
	
	double L1 = 2.5;
	double L2 = 2.5;
	double L3 = 2.5;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	arma::vec new_origin = {0., 0., 0.};
	
	data2pdf_KL phid1(Data1, Data1, Data1, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_KL phid2(Data2, Data2, Data2, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_KL phid3(Data3, Data3, Data3, L1, L2, L3, dL1, dL2, dL3, new_origin);
	
	int n_seg = seg1.n_rows+seg2.n_rows+seg3.n_rows;
	
	arma::mat distance_mat = arma::zeros(n_seg, n_seg);
	
	cout<<n_seg<<endl;
	
}