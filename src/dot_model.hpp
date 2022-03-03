#ifndef dot_model_HPP
#define dot_model_HPP
#include<armadillo>
#include"rk4_int.hpp"



class dot_model {
    public:
        double dt;
        double tcurr=0.0;
        arma::vec Xcurr, Ucurr;
        arma::mat xdlist;
        dot_model (double _dt);//argument should be any required parameters
        arma::vec proj_func (const arma::vec& x);
        inline arma::vec f(const arma::vec& x, const arma::vec& u);
        inline arma::mat dfdx(const arma::vec& x, const arma::vec& u);
        inline arma::mat hx(const arma::vec& x);
        void step(void);
        inline arma::vec get_measurement(const arma::vec& x);
};

dot_model::dot_model (double _dt){//add any additional system parameters
    dt = _dt; 
}

arma::vec dot_model::proj_func (const arma::vec& x){//anglewrapping function if needed
    arma::vec xwrap=x;
    //do nothing if no angle wrapping is required
    return xwrap;
}
inline arma::vec dot_model::f(const arma::vec& x, const arma::vec& u){//control affine required
    arma::vec xdot = {
		x(1),
		u(0),
		x(3),
        u(1),
        x(5),
        u(2)
	};
    return xdot;
}; 
inline arma::vec dot_model::get_measurement(const arma::vec& x){
	return x;
};

inline arma::mat dot_model::dfdx(const arma::vec& x, const arma::vec& u){
    arma::mat A = {//add more rows as needed
        {0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 0, 0},
		{0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 1},
		{0, 0, 0, 0, 0, 0}
    };
    return A;
}; 

inline arma::mat dot_model::hx(const arma::vec& x){
    arma::mat H = {//add more rows as needed
        {0, 0, 0},
        {1, 0, 0},
		{0, 0, 0},
        {0, 1, 0},
		{0, 0, 0},
        {0, 0, 1}
    };
    return H;
}; 

void dot_model::step(){
    Xcurr = RK4_step(this,Xcurr,Ucurr,dt);
    tcurr = tcurr+dt;
};


#endif