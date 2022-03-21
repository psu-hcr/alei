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
	}
	
	
	
};

#endif