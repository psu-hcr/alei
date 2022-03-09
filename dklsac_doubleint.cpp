#include <iostream>
#include<string>
#include <fstream>
#include<math.h>
#include<armadillo>

using namespace std;

#include"SAC_MDA/doubleint.hpp"
#include"SAC_MDA/dkl_cost.hpp"
#include"SAC_MDA/SAC.hpp"
#include"SAC_MDA/rk4_int.hpp"


double xbound = 0.5,ybound = 0.5;

double phid(const arma::vec& x){
  arma::vec lbounds = {{-xbound},{-ybound}};
  //arma::vec x = {{x1},{x2}};
  arma::vec Mu= {{1-0.2},{0.1}}; //Mu=Mu-lbounds;
  arma::vec Mu2 = {{1+0.3},{-0.1}}; //Mu2 = Mu2-lbounds;
  arma::vec Mu3 = {{1+0.35},{0.3}}; //Mu3 = Mu3-lbounds;
  arma::mat Sig = {{0.001,0.},{0.,0.001}};
  arma::mat Sig2 = {{0.01,0.},{0.,0.01}};
  double dist1 = 0.3*arma::as_scalar(arma::expmat(-0.5*(x-Mu).t()*Sig.i()*(x-Mu))/pow(pow(2*PI,2)*arma::det(Sig),0.5));
  double dist2 = 0.3*arma::as_scalar(arma::expmat(-0.5*(x-Mu2).t()*Sig2.i()*(x-Mu2))/pow(pow(2*PI,2)*arma::det(Sig2),0.5));
  double dist3 = 0.4*arma::as_scalar(arma::expmat(-0.5*(x-Mu3).t()*Sig.i()*(x-Mu3))/pow(pow(2*PI,2)*arma::det(Sig),0.5));
return dist1+dist2+dist3;};

arma::vec unom(double t){
        return arma::zeros(2,1);};

int main()
{   ofstream myfile;
    myfile.open ("DIdkltest.csv");
    DoubleInt syst1 (1./100.);
    syst1.Ucurr = unom(0); 
    random_device rd; mt19937 eng(rd());
    uniform_real_distribution<> distr(-0.4,0.4);
    //syst1.Xcurr = {-0.2,0.0,0.1,0.0};
    syst1.Xcurr = {distr(eng),distr(eng),distr(eng),distr(eng)};//must be initialized before instantiating cost
    arma::mat R = 0.0001*arma::eye(2,2); double q=1.;
    arma::vec umax = {10.0,10.0};
    double T = 0.6;
    arma::mat SIGMA = 0.01*arma::eye(2,2);
    dklcost<DoubleInt> cost (q,R,100,SIGMA,0,2,phid,xbound,ybound,T,&syst1);
    sac<DoubleInt,dklcost<DoubleInt>> sacsys (&syst1,&cost,0.,T,umax,unom);
 
    arma::vec xwrap;
           
    myfile<<"time,x,xdot,y,ydot,ux,uy,dklcost\n";
 
    while (syst1.tcurr<30.0){
    //double start_time = omp_get_wtime();
    cost.xmemory(syst1.Xcurr);
    //cout <<"resamp time: "<< 1000 * (omp_get_wtime() - start_time)<<endl;
    if(fmod(syst1.tcurr,2)<syst1.dt)cout<<"Time: "<<syst1.tcurr<<"\n"<<syst1.Xcurr;
    myfile<<syst1.tcurr<<",";
    xwrap = syst1.proj_func(syst1.Xcurr); 
    myfile<<xwrap(0)<<","<<xwrap(1)<<",";
    myfile<<xwrap(2)<<","<<xwrap(3)<<",";
    myfile<<syst1.Ucurr(0)<<","<<syst1.Ucurr(1)<<",";
    myfile<<cost.calc_cost(syst1.Xcurr,syst1.Ucurr);
    myfile<<"\n";
    syst1.step();
    sacsys.SAC_calc();
    syst1.Ucurr = sacsys.ulist.col(0); 
    sacsys.unom_shift();  
     
    } 
      
    myfile.close();
 ofstream samples;
 samples.open("Domain_samples.csv");
 cost.domainsamps.save(samples,arma::csv_ascii);
 arma::mat temp = cost.ps_i.t();
 temp.save(samples,arma::csv_ascii);
 samples.close();
    
}

