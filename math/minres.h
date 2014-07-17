#ifndef POUF_MATH_MINRES_H
#define POUF_MATH_MINRES_H

#include <math/lanczos.h>

namespace math {

  struct minres : lanczos {

	vec d_next, d, d_prev;

	vec r;

	real phi, tau;

	real delta_1;

	real c, s;

	real eps;


	minres(real sigma = 0.0) : lanczos(sigma) { }


	void init(const vec& residual) {
	  lanczos::init(residual);

	  r = residual;

	  const unsigned n = residual.size();

	  d = vec::Zero(n);
	  d_prev = vec::Zero(n);
	  
	  phi = beta;
	  tau = beta;

	  delta_1 = 0;

	  c = -1;
	  s = 0;

	  eps = 0;
	  k = 0;
	}
	

	static inline real sign(const real& x) { return !x ? 0 : 
		x < 0 ? -1 : 1; }

	static inline real abs(const real& x) { return sign(x) * x; }


	// givens rotation annihilating b. r is the new value for a.
	static void givens(const real& a, const real& b,
					   real& c, real&s, real& r) {

	  // identity or half-turn depending on sign since r must be
	  // positive
	  if( !b ) {
		s = 0;
		r = abs( a );
		c = a ? sign(a) : 1.0;
      } else if( !a ) {
		c = 0;
		s = sign(b);
		r = abs(b);
      } else {

		// regular case
		const real aabs = abs(a);
		const real babs = abs(b);
      
		if( babs > aabs ) {
		  const real tau = a / b;
		  
		  // TODO wikipedia says -s and -c 
		  s = sign( b ) / std::sqrt( 1 + tau * tau );
		  c = s * tau;
		  r = b / s;
		} else {
		  const real tau = b / a;

		  // TODO wikipedia says -s and -c 
		  c = sign( a ) / std::sqrt( 1 + tau * tau );
		  s = c * tau;
		  r = a / c;
		}
	
      }

	}

	// iteration infos
	struct {
	  real alpha_v, alpha_d, alpha_dprev;
	  real tau;
	  
	  // use this to step K x from values of Kv (that is computed
	  // during main call to A(v) when stepping minres. call this
	  // after a minres step. Kv is erased.
	  void step(vec& Kx, vec& Kd, vec& Kdprev, vec&& Kv) const {
		vec&& tmp = std::move(Kv);
		
		tmp = alpha_v * Kv  +  alpha_d * Kd  +  alpha_dprev * Kdprev;
		Kx += tau * tmp;

		Kd.swap(tmp);
		Kdprev.swap(tmp);
	  }
	  
	} info;

	

	template<class Matrix>
	void step(vec& x, const Matrix& M) {
	  // tool::log("minres step, phi", phi);
	  if( !phi ) return;

	  // note: current v becomes v_prev

	  // tool::log("before lanczos", "v", v.transpose(), "v_prev", v_prev.transpose());
	  lanczos::step(M);
	  // tool::log("after lanczos", "v", v.transpose(), "v_prev", v_prev.transpose());
	  
	  const real delta_2 = c * delta_1  +  s * alpha;
	  const real gamma_1 = s * delta_1  -  c * alpha;

	  const real eps_next = s * beta;
	  const real delta_1_next = -c * beta;

	  // givens rotation
	  real gamma_2;
	  givens(gamma_1, beta, c, s, gamma_2);
	  
	  tau = c * phi;
	  phi = s * phi;

	  // tool::log("minres", "tau", tau, "phi", phi);

	  if(!gamma_2) {
		throw std::logic_error("gamma_2 is zero");
	  }
	  
	  if( gamma_2 ) {
		d_next = (v_prev  -  delta_2 * d  -  eps * d_prev ) / gamma_2;

		// tool::log("minres", "d_next", d_next.transpose(), "v_prev", v_prev.transpose());
		
		// step solution
		x += tau * d_next;
		
		// save stuff
		info.alpha_v = 1 / gamma_2;
		info.alpha_d = -delta_2 / gamma_2;
		info.alpha_dprev = -eps / gamma_2;
		info.tau = tau;

		// moving forward
		d_prev = std::move(d);
		d = std::move( d_next );
		
		eps = eps_next;
		delta_1 = delta_1_next;
		
		// optional stuff
		r = (s * s) * r - (phi * c) * v;
	  }
	  

	  // TODO norm, cond

	}
	
	// TODO iteration infos

	// TODO complete solve
	
  };


}


#endif
