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
#include"src/k_medoids.hpp"

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
	
	/*
	// Camera data
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1_f.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2_f.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new3_f.csv"); 	

	// seg data
	seg1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1_seg.csv"); 	
	seg2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2_seg.csv"); 	
	seg3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new3_seg.csv"); 	
	*/
	
	
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
	
	int n_seg = seg1.n_rows-1+seg2.n_rows-1+seg3.n_rows-1;
	
	arma::mat distance_mat = arma::zeros(n_seg, n_seg);
	cout<<n_seg<<endl;
	arma::cube P;
	arma::cube Q;
	
	for(int i = 0; i<n_seg; i++){
		for(int j = i; j<n_seg; j++){
			if( i<seg1.n_rows-1){
				//cout<<i<<"task1"<<endl;
				//cout<<"seg"<<seg1(i)<<" "<<seg1(i+1)<<endl;
				P = phid1.pdf_t(seg1(i), seg1(i+1));
			}
			else if( i<seg1.n_rows+seg2.n_rows-2){
				//cout<<i<<"task2"<<endl;
				//cout<<"seg"<<seg2(i-seg1.n_rows+1)<<" "<<seg2(i-seg1.n_rows+2)<<endl;
				P = phid2.pdf_t(seg2(i-seg1.n_rows+1), seg2(i-seg1.n_rows+2));
			}
			else{
				//cout<<i<<"task3"<<endl;
				//cout<<"seg"<<seg3(i-seg1.n_rows-seg2.n_rows+2)<<" "<<seg3(i-seg1.n_rows-seg2.n_rows+3)<<endl;
				P = phid3.pdf_t(seg3(i-seg1.n_rows-seg2.n_rows+2), seg3(i-seg1.n_rows-seg2.n_rows+3));
			}
		
			if( j<seg1.n_rows-1){
				//cout<<j<<"task1"<<endl;
				//cout<<"seg"<<seg1(j)<<" "<<seg1(j+1)<<endl;
				Q = phid1.pdf_t(seg1(j), seg1(j+1));
			}
			else if( j<seg1.n_rows+seg2.n_rows-2){
				//cout<<j<<"task2"<<endl;
				//cout<<"seg"<<seg2(j-seg1.n_rows+1)<<" "<<seg2(j-seg1.n_rows+2)<<endl;
				Q = phid2.pdf_t(seg2(j-seg1.n_rows+1), seg2(j-seg1.n_rows+2));
			}
			else{
				//cout<<j<<"task3"<<endl;
				//cout<<"seg"<<seg3(j-seg1.n_rows-seg2.n_rows+2)<<" "<<seg3(j-seg1.n_rows-seg2.n_rows+3)<<endl;
				Q = phid3.pdf_t(seg3(j-seg1.n_rows-seg2.n_rows+2), seg3(j-seg1.n_rows-seg2.n_rows+3));
			}
			double cost = 0.5*(phid1.KL(P, Q) +phid1.KL(Q, P));
			distance_mat(i,j) = cost;
			distance_mat(j,i) = cost;
		}
	}
	ofstream coeff;
	coeff.open("/home/zxl5344/test/src/alei/Gaussian_traj/distance_mat.csv");
	distance_mat.save(coeff,arma::csv_ascii);
	coeff.close();
	
	cout<<"complete calc distance_mat"<<endl;
	
	k_medoids k_medoids(distance_mat, 3);
	k_medoids.cluster();
	
}