#ifndef POUF_MATH_NLNSCG_H
#define POUF_MATH_NLNSCG_H

#include <math/types.h>

namespace math {

  struct nlnscg {

	vec p;
	real grad_norm2;

	nlnscg(unsigned n);
	
	// dx = x - x_old
	real step(vec& x, const vec& dx, const vec& diag);
	
  };


}



#endif
