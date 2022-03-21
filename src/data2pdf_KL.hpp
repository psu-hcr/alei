#ifndef data2pdf_KL_hpp
#define data2pdf_KL_hpp
#include <iostream>
#include<armadillo>
#include<math.h>
#include <algorithm>

using namespace std;

class data2pdf_KL {
	public:
	arma::mat data1, data2, data3;
	arma::cube phi, phi_t;
	arma::vec new_origin;
	int n_rows, n_cols, n_slices;
	double L1, L2, L3;
	double dL1, dL2, dL3;
	data2pdf_KL (arma::mat _data1, arma::mat _data2, arma::mat _data3, double _L1, double _L2, double _L3, double _dL1, double _dL2, double _dL3, arma::vec _origin){
		// this is a class convert data from 3 datasheets into one PDF wrt to _origin. 
		// This class can calculate KL between two PDF 
		
		data1 = _data1; data2 = _data2; data3 = _data3;	
		new_origin = _origin;
		L1 = _L1; L2 = _L2; L3 = _L3; 	// Boundary for PDF
		dL1 = _dL1;  dL2 = _dL2; dL3 = _dL3;	// size of grid
		n_rows = 2*int(L1/dL1);	//cout<<"n_rows"<<n_rows<<endl;
		n_cols = 2*int(L2/dL2);	//cout<<"n_cols"<<n_cols<<endl;
		n_slices = 2*int(L3/dL3);	//cout<<"n_slices"<<n_slices<<endl;
		phi = arma::zeros( n_rows, n_cols, n_slices );
		phi_t = arma::zeros( n_rows, n_cols, n_slices );
	};
	
	arma::cube calcpdf(){
		// calculate pdf based on input data
		
		// add data for datasheet 1
		//cout<<"data1"<<endl;
		for (int i = 0; i < data1.n_rows; i++){
			//cout<<"data1_i"<<i<<endl;
			
			// shift x y z off center
			double x = data1(i, 1) + L1 - new_origin(0);
			double y = data1(i, 2) + L2 - new_origin(1);
			double z = data1(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"data1n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"data1n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"data1n_z"<<n_z<<endl;
			phi(n_x, n_y, n_z)++;	//cout<<"phi(n_x, n_y, n_z)"<<phi(n_x, n_y, n_z)<<endl;
		};
		
		// add data for datasheet 2
		//cout<<"data2"<<endl;
		for (int i = 0; i < data2.n_rows; i++){
			//cout<<"data2_i"<<i<<endl;
			
			// shift x y z off center
			double x = data2(i, 1) + L1 - new_origin(0);
			double y = data2(i, 2) + L2 - new_origin(1);
			double z = data2(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"data2n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"data2n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"data2n_z"<<n_z<<endl;
			phi(n_x, n_y, n_z)++;	//cout<<"phi(n_x, n_y, n_z)"<<phi(n_x, n_y, n_z)<<endl;
		};
		
		// add data for datasheet 3
		//cout<<"data3"<<endl;
		for (int i = 0; i < data3.n_rows; i++){
			//cout<<"data3_i"<<i<<endl;
			
			// shift x y z off center
			double x = data3(i, 1) + L1 - new_origin(0);
			double y = data3(i, 2) + L2 - new_origin(1);
			double z = data3(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"data3n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"data3n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"data3n_z"<<n_z<<endl;
			phi(n_x, n_y, n_z)++;	//cout<<"phi(n_x, n_y, n_z)"<<phi(n_x, n_y, n_z)<<endl;
		};
		
		// normalize phi
		phi = phi/(data1.n_rows+data2.n_rows+data3.n_rows);
		
		return phi;
	};
	
	double readpdf(arma::vec pos){
		// read pdf at pos
		
		double x = pos(0) + L1;
		double y = pos(1) + L2;
		double z = pos(2) + L3;
			
		int n_x = int(x/dL1);	//cout<<"n_x"<<n_x<<endl;
		int n_y = int(y/dL2);	//cout<<"n_y"<<n_y<<endl;
		int n_z = int(z/dL3);	//cout<<"n_z"<<n_z<<endl;
		
		return phi(n_x, n_y, n_z);
	};
	
	arma::cube pdf_t(int start, int col){
		// calculate pdf till col
		
		// reinitialze phi_t
		phi_t = arma::zeros( n_rows, n_cols, n_slices );
		
		if(col > data1.n_rows) col = data1.n_rows;
		if(col > data2.n_rows) col = data2.n_rows;
		if(col > data3.n_rows) col = data3.n_rows;
		
		// add data for datasheet 1
		//cout<<"data1"<<endl;
		for (int i = start; i < col; i++){
			// shift x y z off center
			double x = data1(i, 1) + L1 - new_origin(0);
			double y = data1(i, 2) + L2 - new_origin(1);
			double z = data1(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"n_z"<<n_z<<endl;
			phi_t(n_x, n_y, n_z)++;	//cout<<"phi_t(n_x, n_y, n_z)"<<phi_t(n_x, n_y, n_z)<<endl;
		};
		
		// add data for datasheet 2
		//cout<<"data2"<<endl;
		for (int i = start; i < col; i++){
			// shift x y z off center
			double x = data2(i, 1) + L1 - new_origin(0);
			double y = data2(i, 2) + L2 - new_origin(1);
			double z = data2(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"n_z"<<n_z<<endl;
			phi_t(n_x, n_y, n_z)++;	//cout<<"phi_t(n_x, n_y, n_z)"<<phi_t(n_x, n_y, n_z)<<endl;
		};
		
		// add data for datasheet 3
		//cout<<"data3"<<endl;
		for (int i = start; i < col; i++){
			// shift x y z off center
			double x = data3(i, 1) + L1 - new_origin(0);
			double y = data3(i, 2) + L2 - new_origin(1);
			double z = data3(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"n_z"<<n_z<<endl;
			phi_t(n_x, n_y, n_z)++;	//cout<<"phi_t(n_x, n_y, n_z)"<<phi_t(n_x, n_y, n_z)<<endl;
		};
		
		// normalize phi
		phi_t = phi_t/(3*(col-start));
		
		
		return phi_t;
	};
	
	double KL(arma::cube P, arma::cube Q){
		
		// add small number to P, Q to prevent sigular
		P = P+1e-9;
		Q = Q+1e-9;
		
		// Normalize P&Q
		P = P/arma::accu(P);
		Q = Q/arma::accu(Q);
		
		// calculate KL(P||Q)
		return arma::accu(P%arma::log(P/Q));
	}
};

#endif