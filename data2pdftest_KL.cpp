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
	/*
	// Camera data
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording3.csv");
 	*/
	
	// Camera data
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new3.csv"); 	
	
	double L1 = 2.5;
	double L2 = 2.5;
	double L3 = 2.5;
	double dL1 = 0.05;
	double dL2 = 0.05;
	double dL3 = 0.05;
	arma::vec new_origin = {0., 0., 0.};
	
	ofstream myfile1, myfile2, myfile3;
	myfile1.open ("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL1.csv");
	myfile2.open ("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL2.csv");
	myfile3.open ("/home/zxl5344/test/src/alei/robotdata/data2pdf_KL3.csv");
	myfile1<<"row, cost";
	myfile1<<"\n";
	myfile2<<"row, cost";
	myfile2<<"\n";
	myfile3<<"row, cost";
	myfile3<<"\n";
	
	ofstream seg1, seg2, seg3;
	seg1.open("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1_seg.csv");
	seg2.open("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2_seg.csv"); 	
	seg3.open("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new3_seg.csv"); 	 	
	seg1<<0<<"\n";
	seg2<<0<<"\n";
	seg3<<0<<"\n";
	
	data2pdf_KL phid1(Data1, Data1, Data1, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_KL phid2(Data2, Data2, Data2, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_KL phid3(Data3, Data3, Data3, L1, L2, L3, dL1, dL2, dL3, new_origin);
	
	// define window size
	int w = 10;
	
	// define overlap size
	int s = 5;
	
	// define threshold value
	double threshold1 = 4.;
	double threshold2 = 4.;
	double threshold3 = 4.;
	
	// segmentation counter
	int counter = 0;
	int counter1 = 0;
	int counter2 = 0;
	int counter3 = 0;
	
	int i = 0;
	double previous_cost1 = 0;
	double previous_cost1_raw = 0;
	double previous_cost2 = 0;
	double previous_cost2_raw = 0;
	double previous_cost3 = 0;
	double previous_cost3_raw = 0;
	arma::cube prev_P1 = phid1.pdf_t(i, i+w);
	arma::cube prev_P2 = phid2.pdf_t(i, i+w);
	arma::cube prev_P3 = phid3.pdf_t(i, i+w);
	
	double sum1 =0.0;
	double sum2 =0.0;
	double sum3 =0.0;
	
	// calculate PDF
	while(i < Data1.n_rows){
		arma::cube P1 = phid1.pdf_t(i, i+w);
		double cost1_raw = phid1.KL(prev_P1,P1);
		double cost1 = 0.25*(cost1_raw+previous_cost1_raw) + 0.5*previous_cost1;	// low pass filter
		//double cost1 = phid1.KL(prev_P1,P1);
		
		sum1 += cost1;
		
		myfile1<<i<<","<<cost1;
		myfile1<<"\n";
		previous_cost1 = cost1;
		previous_cost1_raw = cost1_raw;
		prev_P1 = P1;
		i += (w-s);
		counter1++;
		
	}
	
	i = 0;
	while(i < Data2.n_rows){
		arma::cube P2 = phid2.pdf_t(i, i+w);	
		double cost2_raw = phid2.KL(prev_P2,P2);
		double cost2 = 0.25*(cost2_raw+previous_cost2_raw) + 0.5*previous_cost2;	// low pass filter
		//double cost2 = phid2.KL(prev_P2,P2);
		
		sum2 += cost2;
		
		myfile2<<i<<","<<cost2;
		myfile2<<"\n";
		previous_cost2 = cost2;
		prev_P2 = P2;
		previous_cost2_raw = cost2_raw;
		i += (w-s);
		counter2++;
	}
	
	i = 0;
	while(i < Data3.n_rows){
		arma::cube P3 = phid3.pdf_t(i, i+w);
		double cost3_raw = phid3.KL(prev_P3,P3);
		double cost3 = 0.25*(cost3_raw+previous_cost3_raw) + 0.5*previous_cost3;	// low pass filter
		//double cost3 = phid3.KL(prev_P3,P3);
		
		sum3 += cost3;
		
		myfile3<<i<<","<<cost3;
		myfile3<<"\n";
		previous_cost3 = cost3;
		prev_P3 = P3;
		previous_cost3_raw = cost3_raw;
		i += (w-s);
		counter3++;
	}
	
	// calculate average
	double mean1 = sum1/counter1;	cout<<mean1 <<endl;
	double mean2 = sum2/counter2;	cout<<mean2 <<endl;
	double mean3 = sum3/counter3;	cout<<mean3 <<endl;
	
	counter1 = 0.;
	counter2 = 0.;
	counter3 = 0.;
	
	// segment task
	i = 0;
	previous_cost1 = 0.;
	while(i < Data1.n_rows){
		arma::cube P1 = phid1.pdf_t(i, i+w);
		double cost1_raw = phid1.KL(prev_P1,P1);
		double cost1 = 0.25*(cost1_raw+previous_cost1_raw) + 0.5*previous_cost1;	// low pass filter
		//double cost1 = phid1.KL(prev_P1,P1);
		
		if(cost1> mean1){
				if(previous_cost1<=mean1){
					cout<<"a new segmentation of task 1 at "<<i<<endl;
					seg1<<i<<"\n";
					counter++;
					counter1++;
				}
		}
		
		previous_cost1 = cost1;
		previous_cost1_raw = cost1_raw;
		prev_P1 = P1;
		i += (w-s);
		
	}
	
	i = 0;
	previous_cost2 = 0.;
	while(i < Data2.n_rows){
		arma::cube P2 = phid2.pdf_t(i, i+w);	
		double cost2_raw = phid2.KL(prev_P2,P2);
		double cost2 = 0.25*(cost2_raw+previous_cost2_raw) + 0.5*previous_cost2;	// low pass filter
		//double cost2 = phid2.KL(prev_P2,P2);
		
		if(cost2> mean2){
				if(previous_cost2<=mean2){
					cout<<"a new segmentation of task 2 at "<<i<<endl;
					seg2<<i<<"\n";
					counter++;
					counter2++;
				}
		}
		
		previous_cost2 = cost2;
		prev_P2 = P2;
		previous_cost2_raw = cost2_raw;
		i += (w-s);
		
	}
	
	i = 0;
	previous_cost3 = 0.;
	while(i < Data3.n_rows){
		arma::cube P3 = phid3.pdf_t(i, i+w);
		double cost3_raw = phid3.KL(prev_P3,P3);
		double cost3 = 0.25*(cost3_raw+previous_cost3_raw) + 0.5*previous_cost3;	// low pass filter
		//double cost3 = phid3.KL(prev_P3,P3);
		
		if(cost3> mean3){
				if(previous_cost3<=mean3){
					cout<<"a new segmentation of task 3 at "<<i<<endl;
					seg3<<i<<"\n";
					counter++;
					counter3++;
				}
		}
		
		previous_cost3 = cost3;
		prev_P3 = P3;
		previous_cost3_raw = cost3_raw;
		i += (w-s);
		
	}
	
	cout<<"total segmentations are "<<counter<<endl;
	cout<<"task1 segmentations are "<<counter1<<endl;
	cout<<"task2 segmentations are "<<counter2<<endl;
	cout<<"task3 segmentations are "<<counter3<<endl;
	myfile1.close();
	myfile2.close();
	myfile3.close();
}