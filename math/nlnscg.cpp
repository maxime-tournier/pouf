#include "nlnscg.h"

namespace math {


  nlnscg::nlnscg(unsigned n) : p(vec::Zero(n)), grad_norm2(1) { }

  real nlnscg::step(vec& x, const vec& dx, const vec& diag) {

	// note: grad = -dx;
	
	// const real tmp = dx.squaredNorm();
	const real tmp = dx.dot( diag.cwiseProduct(dx) );

	real beta = 0;
	if( grad_norm2 ) {
	  
	  beta = tmp / grad_norm2;
	  if( beta <= 1) {
		
		// before ? after ? the paper says x first
		x += beta * p;
		p = beta * p + dx;

		// this is actually equivalent to:
		// x_new = x_old + p_new
		// x_new = x_curr - dx + p_new
		// x_new = x_curr + beta * p_old
		
	  } else {
		p.setZero();
	  }		  
	}
	
	grad_norm2 = tmp;

	return beta;
  }


}
