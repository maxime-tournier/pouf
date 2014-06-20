#ifndef POUF_MATH_LANCZOS_H
#define POUF_MATH_LANCZOS_H

#include <math/types.h>

namespace math {

  // lanczos iteration
  struct lanczos {

	vec v, v_prev, p;
	
	real beta;
	real alpha;

	unsigned k;
	
	const real sigma;

	lanczos(real sigma = 0.0)
	  : sigma(sigma),
		k(-1) { }
	
	void init(const vec& b) {
	  beta = b.norm();

	  // TODO throw
	  if(!beta) return;
	  
	  v = b.normalized();
	  v_prev = vec::Zero(b.size());

	  k = 0;
	}

	// lanczos step for M + sigma * I

	template<class Matrix>
	void step(const Matrix& M) {
	  p.noalias() = M(v);
	  if( sigma ) p += sigma * v;

	  alpha = p.dot(v);

	  p -= alpha * v + beta * v_prev;

	  // paranoid orthgonalization against previous v
	  p -= p.dot(v) * v;
	  
	  beta = p.norm();

	  // save v before overwriting
	  v_prev.swap(v);

	  // normalization
	  v = p / beta;
	}

  };

  
 
}



#endif
