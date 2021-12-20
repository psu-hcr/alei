#ifndef IIWABASIS2_HPP
#define IIWABASIS2_HPP
#include<armadillo>
#include<cmath>


class iiwaBasis2 {
	
	public:
		arma::vec proj_func (const arma::vec& x);
        arma::vec zxu(const arma::vec& x, const arma::vec& u);
		arma::vec zx(const arma::vec& x);
		arma::vec zu(const arma::vec& x, const arma::vec& u);
		arma::mat dvdu(const arma::vec& z);
		int zdim = 219;//set to length of z including v(x,u)
		int xdim = 212;//set to length of z(x(t)) only        
        
};

arma::vec iiwaBasis2::proj_func (const arma::vec& x){//angle wrapping function
    arma::vec xwrap=x;
    //do nothing if not angle wrapping is required
    return xwrap;
}

arma::vec iiwaBasis2::zx (const arma::vec& x){//note the order of x with a comment here
	arma::vec pos = {x(0),
					 x(1),
					 x(2),
					 x(3),
					 x(4),
					 x(5),
					 x(6)};
	
	arma::vec vel = {x(7),
					 x(8),
					 x(9),
					 x(10),
					 x(11),
					 x(12),
					 x(13)};
	arma::vec pose = {x(14),
					  x(15),
					  x(16),
					  x(17),
					  x(18),
					  x(19),
					  x(20)};
	
    arma::vec psix = {pose(0),
					  pose(1),
					  pose(2),
					  pose(3),
					  pose(4),
					  pose(5),
					  pose(6),
					  1,
					  
					  //pos
					  pos(0),
					  pos(1),
					  pos(2),
					  pos(3),
					  pos(4),
					  pos(5),
					  pos(6),
					  pos(0)*pos(1),
					  pos(1)*pos(2),
					  pos(2)*pos(3),
					  pos(3)*pos(4),
					  pos(4)*pos(5),
					  pos(5)*pos(6),
					  pow(pos(0), 2)*pow(pos(1), 2),
					  pow(pos(1), 2)*pow(pos(2), 2),
					  pow(pos(2), 2)*pow(pos(3), 2),
					  pow(pos(3), 2)*pow(pos(4), 2),
					  pow(pos(4), 2)*pow(pos(5), 2),
					  pow(pos(5), 2)*pow(pos(6), 2),
					  pow(pos(0), 3)*pow(pos(1), 3),
					  pow(pos(1), 3)*pow(pos(2), 3),
					  pow(pos(2), 3)*pow(pos(3), 3),
					  pow(pos(3), 3)*pow(pos(4), 3),
					  pow(pos(4), 3)*pow(pos(5), 3),
					  pow(pos(5), 3)*pow(pos(6), 3),
					  
					  //cos
					  cos(pos(0)),
					  cos(pos(1)),
					  cos(pos(2)),
					  cos(pos(3)),
					  cos(pos(4)),
					  cos(pos(5)),
					  cos(pos(6)),
					  
					  //sin
					  sin(pos(0)),
					  sin(pos(1)),
					  sin(pos(2)),
					  sin(pos(3)),
					  sin(pos(4)),
					  sin(pos(5)),
					  sin(pos(6)),
					  
					  //sin*sin
					  sin(pos(0))*sin(pos(1)),
					  sin(pos(0))*sin(pos(2)),
					  sin(pos(0))*sin(pos(3)),
					  sin(pos(0))*sin(pos(4)),
					  sin(pos(0))*sin(pos(5)),
					  sin(pos(0))*sin(pos(6)),
					  sin(pos(1))*sin(pos(2)),
					  sin(pos(1))*sin(pos(3)),
					  sin(pos(1))*sin(pos(4)),
					  sin(pos(1))*sin(pos(5)),
					  sin(pos(1))*sin(pos(6)),
					  sin(pos(2))*sin(pos(3)),
					  sin(pos(2))*sin(pos(4)),
					  sin(pos(2))*sin(pos(5)),
					  sin(pos(2))*sin(pos(6)),
					  sin(pos(3))*sin(pos(4)),
					  sin(pos(3))*sin(pos(5)),
					  sin(pos(3))*sin(pos(6)),
					  sin(pos(4))*sin(pos(5)),
					  sin(pos(4))*sin(pos(6)),
					  sin(pos(5))*sin(pos(6)),
					  
					  //sin*cos
					  sin(pos(0))*cos(pos(1)),
					  sin(pos(0))*cos(pos(2)),
					  sin(pos(0))*cos(pos(3)),
					  sin(pos(0))*cos(pos(4)),
					  sin(pos(0))*cos(pos(5)),
					  sin(pos(0))*cos(pos(6)),
					  sin(pos(1))*cos(pos(2)),
					  sin(pos(1))*cos(pos(3)),
					  sin(pos(1))*cos(pos(4)),
					  sin(pos(1))*cos(pos(5)),
					  sin(pos(1))*cos(pos(6)),
					  sin(pos(2))*cos(pos(3)),
					  sin(pos(2))*cos(pos(4)),
					  sin(pos(2))*cos(pos(5)),
					  sin(pos(2))*cos(pos(6)),
					  sin(pos(3))*cos(pos(4)),
					  sin(pos(3))*cos(pos(5)),
					  sin(pos(3))*cos(pos(6)),
					  sin(pos(4))*cos(pos(5)),
					  sin(pos(4))*cos(pos(6)),
					  sin(pos(5))*cos(pos(6)),
					  
					  //cos*cos
					  cos(pos(0))*cos(pos(1)),
					  cos(pos(0))*cos(pos(2)),
					  cos(pos(0))*cos(pos(3)),
					  cos(pos(0))*cos(pos(4)),
					  cos(pos(0))*cos(pos(5)),
					  cos(pos(0))*cos(pos(6)),
					  cos(pos(1))*cos(pos(2)),
					  cos(pos(1))*cos(pos(3)),
					  cos(pos(1))*cos(pos(4)),
					  cos(pos(1))*cos(pos(5)),
					  cos(pos(1))*cos(pos(6)),
					  cos(pos(2))*cos(pos(3)),
					  cos(pos(2))*cos(pos(4)),
					  cos(pos(2))*cos(pos(5)),
					  cos(pos(2))*cos(pos(6)),
					  cos(pos(3))*cos(pos(4)),
					  cos(pos(3))*cos(pos(5)),
					  cos(pos(3))*cos(pos(6)),
					  cos(pos(4))*cos(pos(5)),
					  cos(pos(4))*cos(pos(6)),
					  cos(pos(5))*cos(pos(6)),
					  
					  //vel
					  vel(0),
					  vel(1),
					  vel(2),
					  vel(3),
					  vel(4),
					  vel(5),
					  vel(6),
					  vel(0)*vel(1),
					  vel(1)*vel(2),
					  vel(2)*vel(3),
					  vel(3)*vel(4),
					  vel(4)*vel(5),
					  vel(5)*vel(6),
					  pow(vel(0), 2)*pow(vel(1), 2),
					  pow(vel(1), 2)*pow(vel(2), 2),
					  pow(vel(2), 2)*pow(vel(3), 2),
					  pow(vel(3), 2)*pow(vel(4), 2),
					  pow(vel(4), 2)*pow(vel(5), 2),
					  pow(vel(5), 2)*pow(vel(6), 2),
					  pow(vel(0), 3)*pow(vel(1), 3),
					  pow(vel(1), 3)*pow(vel(2), 3),
					  pow(vel(2), 3)*pow(vel(3), 3),
					  pow(vel(3), 3)*pow(vel(4), 3),
					  pow(vel(4), 3)*pow(vel(5), 3),
					  pow(vel(5), 3)*pow(vel(6), 3),
	
					  //cos*vel
					  cos(pos(0))*vel(0),
					  cos(pos(1))*vel(1),
					  cos(pos(2))*vel(2),
					  cos(pos(3))*vel(3),
					  cos(pos(4))*vel(4),
					  cos(pos(5))*vel(5),
					  cos(pos(6))*vel(6),
					  
					  //sin*vel
					  sin(pos(0))*vel(0),
					  sin(pos(1))*vel(1),
					  sin(pos(2))*vel(2),
					  sin(pos(3))*vel(3),
					  sin(pos(4))*vel(4),
					  sin(pos(5))*vel(5),
					  sin(pos(6))*vel(6),
					 
					  //sin*sin
					  sin(pos(0))*sin(pos(1))*vel(0),
					  sin(pos(0))*sin(pos(2))*vel(0),
					  sin(pos(0))*sin(pos(3))*vel(0),
					  sin(pos(0))*sin(pos(4))*vel(0),
					  sin(pos(0))*sin(pos(5))*vel(0),
					  sin(pos(0))*sin(pos(6))*vel(0),
					  sin(pos(1))*sin(pos(2))*vel(1),
					  sin(pos(1))*sin(pos(3))*vel(1),
					  sin(pos(1))*sin(pos(4))*vel(1),
					  sin(pos(1))*sin(pos(5))*vel(1),
					  sin(pos(1))*sin(pos(6))*vel(1),
					  sin(pos(2))*sin(pos(3))*vel(2),
					  sin(pos(2))*sin(pos(4))*vel(2),
					  sin(pos(2))*sin(pos(5))*vel(2),
					  sin(pos(2))*sin(pos(6))*vel(2),
					  sin(pos(3))*sin(pos(4))*vel(3),
					  sin(pos(3))*sin(pos(5))*vel(3),
					  sin(pos(3))*sin(pos(6))*vel(3),
					  sin(pos(4))*sin(pos(5))*vel(4),
					  sin(pos(4))*sin(pos(6))*vel(4),
					  sin(pos(5))*sin(pos(6))*vel(5),
					  
					  //sin*cos
					  sin(pos(0))*cos(pos(1))*vel(0),
					  sin(pos(0))*cos(pos(2))*vel(0),
					  sin(pos(0))*cos(pos(3))*vel(0),
					  sin(pos(0))*cos(pos(4))*vel(0),
					  sin(pos(0))*cos(pos(5))*vel(0),
					  sin(pos(0))*cos(pos(6))*vel(0),
					  sin(pos(1))*cos(pos(2))*vel(1),
					  sin(pos(1))*cos(pos(3))*vel(1),
					  sin(pos(1))*cos(pos(4))*vel(1),
					  sin(pos(1))*cos(pos(5))*vel(1),
					  sin(pos(1))*cos(pos(6))*vel(1),
					  sin(pos(2))*cos(pos(3))*vel(2),
					  sin(pos(2))*cos(pos(4))*vel(2),
					  sin(pos(2))*cos(pos(5))*vel(2),
					  sin(pos(2))*cos(pos(6))*vel(2),
					  sin(pos(3))*cos(pos(4))*vel(3),
					  sin(pos(3))*cos(pos(5))*vel(3),
					  sin(pos(3))*cos(pos(6))*vel(3),
					  sin(pos(4))*cos(pos(5))*vel(4),
					  sin(pos(4))*cos(pos(6))*vel(4),
					  sin(pos(5))*cos(pos(6))*vel(5),
					  
					  //cos*cos
					  cos(pos(0))*cos(pos(1))*vel(0),
					  cos(pos(0))*cos(pos(2))*vel(0),
					  cos(pos(0))*cos(pos(3))*vel(0),
					  cos(pos(0))*cos(pos(4))*vel(0),
					  cos(pos(0))*cos(pos(5))*vel(0),
					  cos(pos(0))*cos(pos(6))*vel(0),
					  cos(pos(1))*cos(pos(2))*vel(1),
					  cos(pos(1))*cos(pos(3))*vel(1),
					  cos(pos(1))*cos(pos(4))*vel(1),
					  cos(pos(1))*cos(pos(5))*vel(1),
					  cos(pos(1))*cos(pos(6))*vel(1),
					  cos(pos(2))*cos(pos(3))*vel(2),
					  cos(pos(2))*cos(pos(4))*vel(2),
					  cos(pos(2))*cos(pos(5))*vel(2),
					  cos(pos(2))*cos(pos(6))*vel(2),
					  cos(pos(3))*cos(pos(4))*vel(3),
					  cos(pos(3))*cos(pos(5))*vel(3),
					  cos(pos(3))*cos(pos(6))*vel(3),
					  cos(pos(4))*cos(pos(5))*vel(4),
					  cos(pos(4))*cos(pos(6))*vel(4),
					  cos(pos(5))*cos(pos(6))*vel(5)};
	return psix;
}
arma::vec iiwaBasis2::zu(const arma::vec& x, const arma::vec& u){//recommend psiu=u;
    arma::vec psiu = {u};
    return psiu;
}; 

arma::mat iiwaBasis2::dvdu(const arma::vec& z){//if psiu=u this is the identity
     arma::mat dvu= 1.*arma::eye(7,7);
    return dvu;
};


inline arma::vec iiwaBasis2::zxu(const arma::vec& x,const arma::vec& u){
    arma::vec psixu = arma::join_cols(zx(x),zu(x,u));
    return psixu;
}; 

#endif