#include "anderson.h"

#include <iostream>

namespace math {

  
  anderson::anderson(unsigned n, unsigned m) : n(n), m(m) {

	if( m ) {
	  G = mat::Zero(n, m);
	  F = mat::Zero(n, m);
	  K = mat::Zero(m, m);
	}
	
	index = 0;
  }

  
  void anderson::step(vec& out, const vec& x, const vec& dx) {
	if( !m ) return;

	tmp.noalias() = F.transpose() * F.col(index);
	
	G.col(index) = x;
	F.col(index) = dx;

	K.col(index) = tmp;
	K.row(index) = tmp.transpose();
	  
	inv.compute( K ); 
	tmp = inv.solve( vec::Ones(m) );

	const real c = tmp.sum();

	if( c ) {
	  tmp /= c;
	  out.noalias() = G * tmp;
	} else {
	  std::cout << "DERP" << std::endl;
	}

	std::cout << "alpha: " << tmp.transpose() << std::endl;

	index = (index + 1) % m;
  }



}
