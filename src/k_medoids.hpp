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
	arma::vec classifcation;
	k_medoids (arma::mat _distance_mat, int _k){
		// this is a class do simplest k_medoids algorithm to find k-cluster
		
		distance_mat = _distance_mat;
		k = _k;
		
		// initialze medoids
		medoids = arma::zeros(k);
		for(int i=0;i<k;i++){
			medoids(i) = i;
		}
		classifcation = arma::zeros(distance_mat.n_cols);
	};
	
	void cluster(){
		cost = calc_cost(medoids);
		
		for(int i = 0; i<k;i++){
			double new_cost = 1e9;
			for(int j = 0; j<distance_mat.n_cols;j++){
				
				if( check_repeat(j, medoids)){
				}
				else{
					arma::vec new_medoids = medoids;
					new_medoids(i) = j;
					new_cost = calc_cost(new_medoids);
					if(new_cost<cost){
						cost = new_cost;
						medoids = new_medoids;
					}
				}
				
			}
		}
		
		// compute final classification 
		calc_cost(medoids);
		//cout<<medoids<<endl;
		cout<<classifcation<<endl;
		
	}
	
	double calc_cost(arma::vec current_medoids){
		double newcost = 0;
		for(int i = 0; i<distance_mat.n_cols;i++){
			double dot_cost = 1e9;
			
			for(int j=0;j<k;j++){
				double currdot_cost = distance_mat(i, int(current_medoids(j)));
				
				if (currdot_cost<dot_cost){
					dot_cost = currdot_cost;
					classifcation(i) = j;
				}
				
			}
			
			newcost += dot_cost;
		}
		return newcost;
	}
	
	bool check_repeat(int indice, arma::vec current_medoids){
		bool indicator = false;
		
		for(int i = 0; i<k;i++){
			
			if(indice == current_medoids(i)){
				indicator = true;
				break;
			}
			
		}
		
		return indicator;
	}
	
};

#endif