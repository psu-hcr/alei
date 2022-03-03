#ifndef TRAJIK_HPP
#define TRAJIK_HPP
#include<armadillo>

class traj_IK{
	public:
	int xdim;
	arma::vec dtraj = arma::zeros(7);
	int total_step;				// Total number of step for one period
	double dt;					// Time step
	arma::mat djsmat;		// Matrix storing desire joint state
	double t_curr;				// current time
	arma::mat data;			// matrix storing trajectory
	
	// class initialization
	traj_IK(double _dt, arma::mat _data){
		dt = _dt;
		data = _data;
		total_step = data.n_rows;
		djsmat = arma::zeros(7,total_step);
	}
	
	// desire trajectory
	arma::vec desire_traj(int step){
		arma::rowvec dtraj = data.row(step);
		return dtraj.as_col();
	}
	
	// desire joint state
	arma::vec desire_jointstate(double t){
		arma::vec djs;
		if (int(t/dt)< total_step){
			int current_step = int(t/dt);
			djs = djsmat.col(current_step);
		}
		else{
			djs = arma::zeros(7);
		}
		return djs;
	}
		
};
#endif