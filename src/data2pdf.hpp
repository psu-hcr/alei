#ifndef data2pdf_hpp
#define data2pdf_hpp
#include<armadillo>
#include<math.h>

using namespace std;

class data2pdf {
	public:
	arma::mat data1, data2, data3;
	arma::cube phi;
	arma::vec new_origin;
	int n_rows, n_cols, n_slices;
	double L1, L2, L3;
	double dL1, dL2, dL3;
	data2pdf (arma::mat _data1, arma::mat _data2, arma::mat _data3, double _L1, double _L2, double _L3, double _dL1, double _dL2, double _dL3, arma::vec _origin){
		// this is a class convert data from 3 datasheets into one PDF wrt to _origin. 
		
		data1 = _data1; data2 = _data2; data3 = _data3;	
		new_origin = _origin;
		L1 = _L1; L2 = _L2; L3 = _L3; 	// Boundary for PDF
		dL1 = _dL1;  dL2 = _dL2; dL3 = _dL3;	// size of grid
		n_rows = 2*int(L1/dL1);	cout<<"n_rows"<<n_rows<<endl;
		n_cols = 2*int(L2/dL2);	cout<<"n_cols"<<n_cols<<endl;
		n_slices = 2*int(L3/dL3);	cout<<"n_slices"<<n_slices<<endl;
		phi = arma::zeros( n_rows, n_cols, n_slices );
	};
	
	arma::cube calcpdf(int T){
		// calculate pdf based on input data
		
		// add data for datasheet 1
		//cout<<"data1"<<endl;
		for (int i = 0; i < T; i++){
			// shift x y z off center
			double x = data1(i, 1) + L1 - new_origin(0);
			double y = data1(i, 2) + L2 - new_origin(1);
			double z = data1(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"n_z"<<n_z<<endl;
			phi(n_x, n_y, n_z)++;	//cout<<"phi(n_x, n_y, n_z)"<<phi(n_x, n_y, n_z)<<endl;
		};
		
		// add data for datasheet 2
		//cout<<"data2"<<endl;
		for (int i = 0; i < T; i++){
			// shift x y z off center
			double x = data2(i, 1) + L1 - new_origin(0);
			double y = data2(i, 2) + L2 - new_origin(1);
			double z = data2(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"n_z"<<n_z<<endl;
			phi(n_x, n_y, n_z)++;	//cout<<"phi(n_x, n_y, n_z)"<<phi(n_x, n_y, n_z)<<endl;
		};
		
		// add data for datasheet 3
		//cout<<"data3"<<endl;
		for (int i = 0; i < T; i++){
			// shift x y z off center
			double x = data3(i, 1) + L1 - new_origin(0);
			double y = data3(i, 2) + L2 - new_origin(1);
			double z = data3(i, 3) + L3 - new_origin(2);
			
			int n_x = int(x/dL1);	//cout<<"n_x"<<n_x<<endl;
			int n_y = int(y/dL2);	//cout<<"n_y"<<n_y<<endl;
			int n_z = int(z/dL3);	//cout<<"n_z"<<n_z<<endl;
			phi(n_x, n_y, n_z)++;	//cout<<"phi(n_x, n_y, n_z)"<<phi(n_x, n_y, n_z)<<endl;
		};
		
		// normalize phi
		phi = phi/(3*T);
		
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
};

#endif