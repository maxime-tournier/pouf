#include "nlnscg.h"

namespace math {


  void nlnscg::step(vec& x, const vec& dx) {

	// note: grad = -dx;
	
	const real tmp = dx.squaredNorm();
		
	if( grad_norm2 && p.size() ) {
	  
	  const real beta = tmp / grad_norm2;
	  if( beta <= 1) {
		
		// before ? after ? who knows ?
		x += beta * p;
		
		p = beta * p + dx;
		
	  } else {
		p.setZero();
	  }		  
	} else {
	  p = dx;
	}

	grad_norm2 = tmp;
  }


}
