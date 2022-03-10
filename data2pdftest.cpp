#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <armadillo>
#include <bits/stdc++.h>
#include <chrono> 
using namespace std;

#include"src/data2pdf.hpp"

int main(){ 
	arma::mat Data1, Data2, Data3;
	Data1.load("/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample1.csv"); 	
	Data2.load("/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample2.csv"); 	
	Data3.load("/home/zxl5344/test/src/alei/Sweeping data/ArucoPositionSample3.csv"); 	
	double L1 = 1;
	double L2 = 1;
	double L3 = 1.5;
	double dL1 = 0.005;
	double dL2 = 0.005;
	double dL3 = 0.005;
	data2pdf data2pdf(Data1, Data2, Data3, L1, L2, L3, dL1, dL2, dL3);
	data2pdf.calcpdf();
	//cout<<data2pdf.phi(90, 113, 225)<<endl;
	cout<<"sum of phi"<<arma::accu(data2pdf.phi)<<endl;
	//data2pdf.phi.save("/home/zxl5344/test/src/alei/Sweeping data/phi.csv", arma::arma_ascii);
	cout<<"read phi at(0.609, -0.029,  0.793)"<<data2pdf.readpdf(0.609, -0.029,  0.793)<<endl;
}