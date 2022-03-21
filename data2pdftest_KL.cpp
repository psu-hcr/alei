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
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv"); 	
	
	double L1 = 2.5;
	double L2 = 2.5;
	double L3 = 2.5;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	arma::vec new_origin = {0., 0., 0.};
	
	//ofstream myfile, segmentation;
	//myfile.open ("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL.csv");
	//segmentation.open ("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_seg.csv");
	//myfile<<"row, cost,";
	//myfile<<"\n";
	segmentation<<"task1 seg, task2 seg, task3 seg,";
	segmentation<<"\n";
	
	data2pdf_KL phid1(Data1, Data1, Data1, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_KL phid2(Data2, Data2, Data2, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_KL phid3(Data3, Data3, Data3, L1, L2, L3, dL1, dL2, dL3, new_origin);
	
	// define window size
	int w = 20;
	
	// define overlap size
	int s = 0;
	
	// define threshold value
	double threshold1 = 10.;
	double threshold2 = 14.;
	double threshold3 = 15.;
	
	// segmentation counter
	int counter = 0;
	int counter1 = 0;
	int counter2 = 0;
	int counter3 = 0;
	
	int i = 0;
	double previous_cost1 = 0;
	double previous_cost2 = 0;
	double previous_cost3 = 0;
	arma::cube prev_P1 = phid1.pdf_t(i, i+w);
	arma::cube prev_P2 = phid2.pdf_t(i, i+w);
	arma::cube prev_P3 = phid3.pdf_t(i, i+w);
	
	// calculate PDF
	while(i < Data1.n_rows){
		arma::cube P1 = phid1.pdf_t(i, i+w);	
		arma::cube P2 = phid2.pdf_t(i, i+w);	
		arma::cube P3 = phid3.pdf_t(i, i+w);	
		double cost1 = phid1.KL(prev_P1,P1);
		double cost2 = phid2.KL(prev_P2,P2);
		double cost3 = phid3.KL(prev_P3,P3);
		
		if(cost1-previous_cost1 > threshold1){
			cout<<"a new segmentation of task 1 at "<<i<<endl;
			counter++;
			counter1++;
		}
		
		if(cost2-previous_cost2 > threshold2){
			cout<<"a new segmentation of task 2 at "<<i<<endl;
			counter++;
			counter2++;
		}
		
		if(cost3-previous_cost3 > threshold3){
			cout<<"a new segmentation of task 3 at "<<i<<endl;
			counter++;
			counter3++;
		}
		
		previous_cost1 = cost1;
		previous_cost2 = cost2;
		previous_cost3 = cost3;
		prev_P1 = P1;
		prev_P2 = P2;
		prev_P3 = P3;
		i += (w-s);
		segmentation<<"\n";
	}
	
	cout<<"total segmentations are "<<counter<<endl;
	cout<<"task1 segmentations are "<<counter1<<endl;
	cout<<"task2 segmentations are "<<counter2<<endl;
	cout<<"task3 segmentations are "<<counter3<<endl;
	
}