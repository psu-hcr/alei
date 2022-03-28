#ifndef dkl_cost_cube_HPP
#define dkl_cost_cube_HPP
#include<armadillo>
#include<math.h>
#include<omp.h>
using namespace std;

template <class system>
class dkl_cost_cube {
  system* sys;
  double L1,L2,L3,T;
  int X1,X2, X3;//index of relavant dimensions in xvector
  arma::uvec X_DKL;
  int K;//number of samples in the search domain
  arma::mat sigma; 
  int MAXINT=600;
  public:
    arma::mat xpast;
    arma::mat domainsamps;
    arma::vec qs_i,ps_i;
    double Q;
    arma::mat R;
    int T_index,t_now=0;
    arma::cube phid;
    dkl_cost_cube(double _Q, arma::mat _R,int _K,arma::mat _sigma, int _X1,int _X2, int _X3,arma::cube _phid,double _L1,double _L2, 
	double _L3, double _T,system *_sys){
      Q=_Q; R=_R; sys=_sys; K = _K; phid = _phid; T=_T; // initialize with Q, R, sys, phid, and the domain
      X1 = _X1; X2=_X2; X3=_X3; L1 = _L1; L2 = _L2; L3 = _L3;X_DKL<<X1<<X2<<X3; sigma=_sigma;
      T_index = T/sys->dt;
      omp_set_dynamic(0); // get rid of dynamic stuff
      omp_set_num_threads(16); // set the number of threads
      xpast.set_size(sys->Xcurr.n_rows,300/sys->dt);//initialize xpast to hold up to fiveminutes of data
      domainsamps.set_size(3, K);
      qs_i.zeros(K);ps_i.set_size(K);
      resample(); //initialize the uniform samples over the domain and the probability of each sample
    };
    double l (const arma::vec& x,const arma::vec& u,double ti);
    arma::vec dldx (const arma::vec&x, const arma::vec& u, double ti);
    double calc_cost (const arma::mat& x,const arma::mat& u);
    void xmemory (const arma::vec&);
    void resample ();
    void qs_disc(const arma::mat& x);
};/////////end main class def

template<class system> double dkl_cost_cube<system>::l (const arma::vec& x,const arma::vec& u,double ti){
      arma::vec xproj = sys->proj_func(x);
      arma::mat Qtemp = arma::zeros<arma::mat>(xproj.n_rows,xproj.n_rows);
      Qtemp(X1,X1)= pow(xproj(X1)/(L1+(0.1*L1)),8);
      Qtemp(X2,X2) = pow(xproj(X2)/(L2+(0.1*L2)),8);
	  Qtemp(X3,X3) = pow(xproj(X3)/(L3+(0.1*L3)),8);
      return arma::as_scalar((xproj.t()*Qtemp*xproj+u.t()*R*u)/2);
}

template<class system> arma::vec dkl_cost_cube<system>::dldx (const arma::vec&x, const arma::vec& u, double ti){
  arma::vec xproj = sys->proj_func(x);
  arma::vec a; a.zeros(xproj.n_rows);
  arma::mat Qtemp = arma::zeros<arma::mat>(xproj.n_rows,xproj.n_rows);
  Qtemp(X1,X1) = pow(xproj(X1)/(L1+(0.1*L1)),8);
  Qtemp(X2,X2) = pow(xproj(X2)/(L2+(0.1*L2)),8);
  Qtemp(X3,X3) = pow(xproj(X3)/(L3+(0.1*L3)),8);
  a=a+5*Qtemp*xproj;
  for(int n=0;n<ps_i.n_rows;n++){
    arma::vec s_x = domainsamps.col(n)-xproj.elem(X_DKL);
    a.elem(X_DKL)-= arma::as_scalar(ps_i(n)/qs_i(n))*exp(-0.5*arma::as_scalar(s_x.t()*sigma.i()*s_x))*sigma.i()*s_x;
  };
  return a;}

template<class system> double dkl_cost_cube<system>::calc_cost (const arma::mat& x,const arma::mat& u){
  double J1 = 0.,Jtemp;
  arma::mat xjoined;
  if(t_now<=180){xjoined = arma::join_rows(xpast.cols(0,t_now),x);}
  else{xjoined = arma::join_rows(xpast.cols(t_now-120,t_now),x);};	//cout<<"test"<<endl;
  qs_disc(xjoined);	//cout<<"qs_disc"<<endl;
  J1 = -arma::as_scalar(arma::sum(ps_i%arma::log(qs_i)));	//cout<<"ps_i= "<<ps_i<<endl;
  J1 = Q*J1;
  for (int i = 0; i<x.n_cols; i++){
    arma::vec xproj = sys->proj_func(x.col(i));
    J1+=l(xproj,u.col(i),sys->tcurr+(double)i*sys->dt); 
  };
return J1;}

template<class system> void dkl_cost_cube<system>::qs_disc(const arma::mat& x){//double start_time = omp_get_wtime();
  #pragma omp parallel for
  for(int n=0;n<qs_i.n_rows;n++){
    qs_i(n) = 0.;
    if(x.n_cols<MAXINT){
      for(int j=0;j<x.n_cols;j++){
        arma::vec sj = domainsamps.col(n)-sys->proj_func(x.col(j)).elem(X_DKL);
        qs_i(n)+=sys->dt*exp(-0.5*arma::as_scalar(sj.t()*sigma.i()*sj));
      };
    }else{
      arma::vec ind = arma::randi<arma::vec>(MAXINT,arma::distr_param(0,MAXINT-1));
      for(int j=0;j<MAXINT;j++){
        //int index = arma::randi<int>(arma::distr_param(0,MAXINT-1));
        arma::vec sj = domainsamps.col(n)-sys->proj_func(x.col(ind(j))).elem(X_DKL);
        qs_i(n)+=sys->dt*exp(-0.5*arma::as_scalar(sj.t()*sigma.i()*sj));
      }; 
        
    };    
  };
  qs_i=arma::normalise(qs_i,1);//normalise the discrete pdf over the samples
  //if((omp_get_wtime() - start_time)>1./60.) cout <<"qi time: "<<(omp_get_wtime() - start_time)<<endl;
}
      
template<class system> void dkl_cost_cube<system>::resample(){
  //Choose K samples over the domain [[-L1,L1],[-L2,L2]]
  arma::vec domainsize = {2.*L1, 2.*L2, 2.*L3};//{2*L1,2*L2};
  domainsamps=arma::diagmat(domainsize)*arma::randu<arma::mat>(3,K);//generate uniform random samples from 0 to 2*L
  domainsamps.each_col() -= (0.5*domainsize);//shift samples to go from -L to L
  #pragma omp parallel for
  for(int n=0;n<ps_i.n_rows;n++){
	int posX = (domainsamps(0, n) + L1)/0.05;	//cout<<"posX "<<posX<<endl;
	int posY = (domainsamps(1, n) + L2)/0.05;	//cout<<"posY "<<posY<<endl;
	int posZ = (domainsamps(2, n) + L3)/0.05;	//cout<<"posZ "<<posZ<<endl;
    ps_i(n) = phid(posX, posY, posZ); 					//cout<<"phid(posX, posY, posZ)"<<phid(posX, posY, posZ)<<endl;
  };
  ps_i=arma::normalise(ps_i,1);//normalise the discrete pdf over the samples
}

template<class system> void dkl_cost_cube<system>::xmemory(const arma::vec& x){
  xpast.col(t_now)= x;
  t_now++;
  resample();
}
#endif