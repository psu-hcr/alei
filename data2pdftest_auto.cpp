#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <armadillo>
#include <bits/stdc++.h>
#include <chrono> 
#include <string>
using namespace std;

#include"src/data2pdf_auto.hpp"

arma::vec unom(double t){
        return arma::zeros(3,1);
};

int main(){ 
	arma::mat Data1;
	
	string a = "/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv";
	// Camera data
	Data1.load(a); 	
 	
	double L1 = 2.5;
	double L2 = 2.5;
	double L3 = 2.5;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	arma::vec new_origin = {0., 0., 0.};
	
	data2pdf_auto seg1(Data1, L1, L2, L3, dL1, dL2, dL3, new_origin);
	
	string path_to_cost = "/home/zxl5344/test/src/alei/robotdata/data2pdf_KL3.csv";
	string path_to_seg = "/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3_seg.csv";
	
	int n_seg = seg1.autoSeg(path_to_cost, path_to_seg);
	cout<<"n_seg "<<n_seg<<endl;
}