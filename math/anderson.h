#ifndef POUF_MATH_ANDERSON_H
#define POUF_MATH_ANDERSON_H

#include <math/types.h>

#include <Eigen/Cholesky>

namespace math {

  struct anderson {

	unsigned n, m;

	typedef Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic> mat;
	
	mat G, F, K;
	vec tmp;
  
	unsigned index;
	Eigen::LDLT<Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic> > inv;
	
	anderson(unsigned n, unsigned m = 4);
	void step(vec& out, const vec& x, const vec& dx);
  
  };


  struct anderson_new : anderson {

	mat P;

	mutable vec next_dual, next_primal, next_f;

	anderson_new(unsigned n, unsigned m = 4);
	void step(vec& dual, vec& primal, const vec& diag, const vec& unilateral);
	void step_opt(vec& x, vec& primal, const vec& diag, const vec& unilateral);

	mutable vec f;
	
	real df2;
  };





}


#endif
