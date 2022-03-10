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
	double L3 = 2;
	double dL1 = 0.01;
	double dL2 = 0.01;
	double dL3 = 0.01;
	data2pdf(Data1, Data2, Data3, L1, L2, L3);
}