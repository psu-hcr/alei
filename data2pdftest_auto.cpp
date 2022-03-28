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

int main(){ 
	arma::mat Data1, Data2, Data3;
	
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1.csv"); 	
 	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2.csv");
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv");
	
	double L1 = 1.2;
	double L2 = 1.2;
	double L3 = 1.2;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	arma::vec new_origin = {0., 0., 1.1};
	
	data2pdf_auto seg1(Data1, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_auto seg2(Data2, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_auto seg3(Data3, L1, L2, L3, dL1, dL2, dL3, new_origin);
	
	string path_to_cost1 = "/home/zxl5344/test/src/alei/robotdata/data2pdf_KL1.csv";
	string path_to_cost2 = "/home/zxl5344/test/src/alei/robotdata/data2pdf_KL2.csv";
	string path_to_cost3 = "/home/zxl5344/test/src/alei/robotdata/data2pdf_KL3.csv";
	
	string path_to_seg1 = "/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1_seg.csv";
	string path_to_seg2 = "/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2_seg.csv";
	string path_to_seg3 = "/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3_seg.csv";
	
	cout<<"task 1"<<"\n"<<endl;
	int n_seg1 = seg1.autoSeg(path_to_cost1, path_to_seg1);
	cout<<"task 2"<<"\n"<<endl;
	int n_seg2 = seg2.autoSeg(path_to_cost2, path_to_seg2);
	cout<<"task 3"<<"\n"<<endl;
	int n_seg3 = seg3.autoSeg(path_to_cost3, path_to_seg3);
	
	cout<<"n_seg1 "<<n_seg1<<endl;
	cout<<"n_seg2 "<<n_seg2<<endl;
	cout<<"n_seg3 "<<n_seg3<<endl;

}