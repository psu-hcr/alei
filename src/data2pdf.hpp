#ifndef data2pdf_hpp
#define data2pdf_hpp
#include<armadillo>
#include<math.h>

using namespace std;

class data2pdf {
	ofstream *datafile1;
	ofstream *datafile2;
	ofstream *datafile3;
	arma::cube phi;
	int n_rows, n_cols, n_slices;
	double L1, L2, L3;
	double dL1, dL2, dL3;
	data2pdf (ofstream *_myfile1, ofstream *_myfile2, ofstream *_myfile3, double _L1, double _L2, double _L3, double _dL1, double _dL2, double _dL3){
		datafile1 = _myfile1; datafile2 = _myfile2; datafile2 = _myfile2;	// cvs file contain data
		L1 = _L1; L2 = _L2; L3 = _L3; 
		dL1 = _dL1;  dL2 = _dL2; dL3 = _dL3;
		n_rows = 2*int(L1/dL1);
		n_cols = 2*int(L2/dL2);
		n_slices = 2*int(L3/dL3);
		phi = arma::zeros( n_rows, n_cols, n_slices );
	}
	
}