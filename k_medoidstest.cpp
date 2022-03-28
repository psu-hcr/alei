#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <armadillo>
#include <bits/stdc++.h>
#include <chrono> 
using namespace std;

#include"src/data2pdf_auto.hpp"
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
	Data1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new3.csv"); 	

	// seg data
	seg1.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new1_seg.csv"); 	
	seg2.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new2_seg.csv"); 	
	seg3.load("/home/zxl5344/test/src/alei/Gaussian_traj/CameraRecording_new3_seg.csv"); 	
	*/
	double L1 = 1.2;
	double L2 = 1.2;
	double L3 = 1.2;
	double dL1 = 0.01;
	double dL2 = 0.01;
	double dL3 = 0.01;
	arma::vec new_origin = {0., 0., 1.1};
	
	data2pdf_auto phid1(Data1, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_auto phid2(Data2, L1, L2, L3, dL1, dL2, dL3, new_origin);
	data2pdf_auto phid3(Data3, L1, L2, L3, dL1, dL2, dL3, new_origin);
	
	int n_seg = seg1.n_rows-1+seg2.n_rows-1+seg3.n_rows-1;
	
	arma::mat distance_mat = arma::zeros(n_seg, n_seg);
	cout<<n_seg<<endl;
	arma::cube P;
	arma::cube Q;
	
	for(int i = 0; i<n_seg; i++){
		for(int j = i; j<n_seg; j++){
			if( i<seg1.n_rows-1){
				P = phid1.pdf_t(seg1(i), seg1(i+1));
			}
			else if( i<seg1.n_rows+seg2.n_rows-2){
				P = phid2.pdf_t(seg2(i-seg1.n_rows+1), seg2(i-seg1.n_rows+2));
			}
			else{
				P = phid3.pdf_t(seg3(i-seg1.n_rows-seg2.n_rows+2), seg3(i-seg1.n_rows-seg2.n_rows+3));
			}
		
			if( j<seg1.n_rows-1){
				Q = phid1.pdf_t(seg1(j), seg1(j+1));
			}
			else if( j<seg1.n_rows+seg2.n_rows-2){
				Q = phid2.pdf_t(seg2(j-seg1.n_rows+1), seg2(j-seg1.n_rows+2));
			}
			else{
				Q = phid3.pdf_t(seg3(j-seg1.n_rows-seg2.n_rows+2), seg3(j-seg1.n_rows-seg2.n_rows+3));
			}
			double cost = phid1.KL(P, Q) +phid1.KL(Q, P);
			distance_mat(i,j) = cost;
			distance_mat(j,i) = cost;
		}
	}
	
	ofstream coeff;
	coeff.open("/home/zxl5344/test/src/alei/Gaussian_traj/distance_mat.csv");
	distance_mat.save(coeff,arma::csv_ascii);
	coeff.close();
	
	cout<<"complete calc distance_mat"<<endl;
	
	// calculate cluster
	k_medoids k_medoids(distance_mat, 3);
	k_medoids.cluster();
	arma::vec classifcation_1 = arma::zeros(seg1.n_rows-1);
	arma::vec classifcation_2 = arma::zeros(seg2.n_rows-1);
	arma::vec classifcation_3 = arma::zeros(seg3.n_rows-1);
	for(int i =0; i<k_medoids.classifcation.n_rows; i++){
		if(i<classifcation_1.n_rows){
			classifcation_1(i) = k_medoids.classifcation(i);
		}
		else if (i<classifcation_1.n_rows+classifcation_2.n_rows){
			classifcation_2(i-classifcation_1.n_rows) = k_medoids.classifcation(i);
		}
		else{
			classifcation_3(i-classifcation_1.n_rows-classifcation_2.n_rows) = k_medoids.classifcation(i);
		}
	}
	cout<<"classification_1 is "<<endl;
	cout<<classifcation_1<<endl;
	cout<<"classification_2 is "<<endl;
	cout<<classifcation_2<<endl;
	cout<<"classification_3 is "<<endl;
	cout<<classifcation_3<<endl;
	ofstream c;
	c.open("/home/zxl5344/test/src/alei/Gaussian_traj/classifcation.csv");
	k_medoids.classifcation.save(c,arma::csv_ascii);
	c.close();
	
	// generate task list
	cout<<"start recreate tasklists"<<endl;
	int n_subtasks = 3; int n_start_subtask = 0;
	arma::vec task_list = k_medoids.task_gen(classifcation_1, classifcation_2, classifcation_3, n_subtasks, n_start_subtask);
	
	
	// combine seg into subtask
	// number of subtask must change maunally 
	arma::cube subtask0 = arma::zeros(size(P));
	arma::cube subtask1 = arma::zeros(size(P));
	arma::cube subtask2 = arma::zeros(size(P));
	
	for(int i = 0; i<n_seg; i++){
		if( i<seg1.n_rows-1){
			int start = seg1(i);
			int end = seg1(i+1);
			if(k_medoids.classifcation(i)==0){
				subtask0 = phid1.sum_pdf_t(start, end, subtask0);
				//cout<<i<<endl;
			}
			else if(k_medoids.classifcation(i)==1){
				subtask1 = phid1.sum_pdf_t(start, end, subtask1);
				//cout<<i<<endl;
			}
			else if(k_medoids.classifcation(i)==2){
				subtask2 = phid1.sum_pdf_t(start, end, subtask2);
				//cout<<i<<endl;
			}
			else{
				cout<<"error: n_classifictaion is wrong"<<endl;
			}
				
		}
		else if( i<seg1.n_rows+seg2.n_rows-2){
			int start = seg2(i-seg1.n_rows+1);
			int end = seg2(i-seg1.n_rows+2);
			if(k_medoids.classifcation(i)==0){
				subtask0 = phid2.sum_pdf_t(start, end, subtask0);
				//cout<<i<<endl;
			}
			else if(k_medoids.classifcation(i)==1){
				subtask1 = phid2.sum_pdf_t(start, end, subtask1);
				//cout<<i<<endl;
			}
			else if(k_medoids.classifcation(i)==2){
				subtask2 = phid2.sum_pdf_t(start, end, subtask2);
				//cout<<i<<endl;
			}
			else{
				cout<<"error: n_classifictaion is wrong"<<endl;
			}
			
		}
		else{
			int start = seg3(i-seg1.n_rows-seg2.n_rows+2);
			int end = seg3(i-seg1.n_rows-seg2.n_rows+3);
			if(k_medoids.classifcation(i)==0){
				subtask0 = phid3.sum_pdf_t(start, end, subtask0);
				//cout<<i<<endl;
			}
			else if(k_medoids.classifcation(i)==1){
				subtask1 = phid3.sum_pdf_t(start, end, subtask1);
				//cout<<i<<endl;
			}
			else if(k_medoids.classifcation(i)==2){
				subtask2 = phid3.sum_pdf_t(start, end, subtask2);
				//cout<<i<<endl;
			}
			else{
				cout<<"error: n_classifictaion is wrong"<<endl;
			}
		}
		
	}
	
	cout<<"complete task"<<endl;
}