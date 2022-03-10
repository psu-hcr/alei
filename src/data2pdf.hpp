#ifndef data2pdf_hpp
#define data2pdf_hpp
#include<armadillo>
#include<math.h>

using namespace std;

class data2pdf {
	arma::mat data1, data2, data3;
	arma::cube phi;
	int n_rows, n_cols, n_slices;
	double L1, L2, L3;
	double dL1, dL2, dL3;
	data2pdf (arma::mat _data1, arma::mat _data2, arma::mat _data3, double _L1, double _L2, double _L3, double _dL1, double _dL2, double _dL3){
		data1 = _data1; data2 = _data2; data3 = _data3;	
		L1 = _L1; L2 = _L2; L3 = _L3; 	// Boundary for PDF
		dL1 = _dL1;  dL2 = _dL2; dL3 = _dL3;	// size of grid
		n_rows = 2*int(L1/dL1);
		n_cols = 2*int(L2/dL2);
		n_slices = 2*int(L3/dL3);
		phi = arma::zeros( n_rows, n_cols, n_slices );
	};
	
};