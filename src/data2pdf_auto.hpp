#ifndef data2pdf_auto_hpp
#define data2pdf_auto_hpp
#include <iostream>
#include<armadillo>
#include<math.h>
#include <algorithm>
#include <string>

using namespace std;

class data2pdf_auto {
	public:
	arma::mat data1;
	arma::cube phi, phi_t;
	arma::vec new_origin;
	int n_rows, n_cols, n_slices;
	double L1, L2, L3;
	double dL1, dL2, dL3;
	data2pdf_auto (arma::mat _data1, double _L1, double _L2, double _L3, double _dL1, double _dL2, double _dL3, arma::vec _origin){
		// this is a class convert data from a datasheet into a 3 dim PDF wrt to _origin. 
		// This class can automatically segment task
		// This class can calculate KL between two PDF 
		
		data1 = _data1; 	
		new_origin = _origin;
		L1 = _L1; L2 = _L2; L3 = _L3; 	// Boundary for PDF
		dL1 = _dL1;  dL2 = _dL2; dL3 = _dL3;	// size of grid
		n_rows = 2*int(L1/dL1);	//cout<<"n_rows"<<n_rows<<endl;
		n_cols = 2*int(L2/dL2);	//cout<<"n_cols"<<n_cols<<endl;
		n_slices = 2*int(L3/dL3);	//cout<<"n_slices"<<n_slices<<endl;
		phi = arma::zeros( n_rows, n_cols, n_slices );
		phi_t = arma::zeros( n_rows, n_cols, n_slices );
	};
	
	arma::cube pdf_t(int start, int col){
		// calculate pdf till col
		
		// reinitialze phi_t
		phi_t = arma::zeros(n_rows, n_cols, n_slices);
		
		if(col > data1.n_rows) col = data1.n_rows;
		
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
		
		// normalize phi
		phi_t = phi_t/(col-start);
		
		return phi_t;
	};
	
	arma::cube sum_pdf_t(int start, int col, arma::cube phi_init){
		// calculate pdf till col
		
		// reinitialze phi_t
		phi_t = phi_init;
		
		if(col > data1.n_rows) col = data1.n_rows;
		
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
		
		// normalize phi
		phi_t = phi_t/arma::accu(phi_t);
		
		return phi_t;
	};
	
	int autoSeg(string path_to_cost, string path_to_seg){
		
		int w = 300;
		int n_seg = 100;
		int prev_seg = 0;
		int pprev_seg = 50;
		int ppprev_seg = 30;
		
		ofstream costfile, segfile;
		while((n_seg != prev_seg)||( prev_seg != pprev_seg)||( pprev_seg != ppprev_seg)){
			
			int step = w/5;
			
			ppprev_seg = pprev_seg;
			pprev_seg = prev_seg;
			prev_seg = n_seg;
			
			cout<<"currr w "<<w<<endl;
			
			// calc cost
			costfile.open(path_to_cost);
			int s = w/10;
			double sum1 =0.0;
			double previous_cost = 0.;
			arma::vec cost_vec = arma::zeros((data1.n_rows/(w-s))+2);	//cout<<"cost_n "<<cost_vec.n_rows<<endl;
			int i = 0;
			int counter = 0;
			arma::cube prev_P1 = pdf_t(i, i+w);
			while(i < data1.n_rows){
				arma::cube P1 = pdf_t(i, i+w);
				double cost = KL(prev_P1,P1);
				cost_vec(i/(w-s)) = cost;	
				sum1 += cost;
				costfile<<i<<","<<cost;
				costfile<<"\n";
				previous_cost = cost;
				prev_P1 = P1;
				i += (w-s);	//cout<<i<<endl;
				counter++;
			}
			costfile.close();
			
			double mean1 = sum1/counter;	cout<<"mean "<<mean1 <<endl;
			
			segfile.open(path_to_seg);
			segfile<<0<<"\n";
			i = 1;
			n_seg = 0;
			while(i<cost_vec.n_rows){
				if(cost_vec(i)> mean1){
					if(cost_vec(i-1)<=mean1){
						cout<<"a new segmentation of task at "<<i*(w-s)<<endl;
						segfile<<i*(w-s)<<"\n";
						n_seg++;
					}
				}
				i++;
			}
			segfile.close();
			
			cout<<"curr n_seg "<<n_seg<<endl;
			cout<<"\n";
			
			w -=step;
			
		}
		
	return n_seg;
	}
	
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