#ifndef k_medoids_hpp
#define k_medoids_hpp
#include <iostream>
#include<armadillo>
#include<math.h>

using namespace std;

class k_medoids {
	public:
	arma::mat distance_mat;
	int k;
	arma::vec medoids;
	double cost;
	k_medoids (arma::mat _distance_mat, int _k){
		// this is a class do simplest k_medoids algorithm to find k-cluster
		
		distance_mat = _distance_mat;
		k = _k;
		
		// initialze medoids
		medoids = arma::zeros(k);
		for(int i=0;i<k;i++){
			medoids(i) = i;
		}
		
	};
	
	void cluster(){
		calc_cost();
		cout<<cost<<endl;
	}
	
	void calc_cost(){
		cost = 0;
		for(int i = 0; i<distance_mat.n_cols;i++){
			double dot_cost = 1e9;
			for(int j=0;j<k;j++){
				double currdot_cost = distance_mat(i,j);
				if (currdot_cost<dot_cost){
					dot_cost = currdot_cost;
				}
			}
			cost += dot_cost;
		}
		
	}
	
	
	
};

#endif