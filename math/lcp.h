#ifndef POUF_MATH_LCP_H
#define POUF_MATH_LCP_H

#include <math/minres.h>
#include <math/iter.h>
#include <tool/log.h>

namespace math {

  struct lcp {

	// first index to become negative
	static real track(const vec& start, const vec& delta, unsigned* index) {
	  real alpha = 1;
	  
	  for(unsigned i = 0, n = start.size(); i < n; ++i) {
		
		if( (delta(i) < 0) ) {
		  const real current = -start(i) / delta(i);
		  
		  if( current > 0 && current < alpha ) {
			alpha = current;
			*index = i;
		  }
		}
		
	  }
	  
	  return alpha;
	}

	// first index to become negative
	static real track_notin(const vec& start, const vec& delta, unsigned* index, const vec& mask) {
	  real alpha = 1;
	  
	  for(unsigned i = 0, n = start.size(); i < n; ++i) {
		
		if( !mask(i) && (delta(i) < 0) ) {
		  const real current = -start(i) / delta(i);
		  
		  if( current > 0 && current < alpha ) {
			alpha = current;
			*index = i;
		  }
		}
		
	  }
	  
	  return alpha;
	}


	template<class Matrix>
	static void solve(vec& x, const Matrix& M, const vec& q, const iter& it) {
	  const unsigned n = q.size();

	  vec Mx = M(x);

	  // TODO make it with w instead of z
	  vec w;
	  
	  vec z = Mx + q - x;
	  vec mask = (z.array() < 0).cast<real>();
	  
	  vec storage, My;
	  vec Md, Mdprev;
	  
	  auto A = [&](const vec& y) -> const vec& {
		storage = mask.cwiseProduct(y);
		My.noalias() = M(storage);
		storage = mask.cwiseProduct( My );
		return storage;
	  };
	  
	  vec rhs = -mask.cwiseProduct( q );
	  
	  minres solver;

	  auto restart = [&] {
		Md = vec::Zero(n);
		Mdprev = vec::Zero(n);
	  };

	
	  vec residual = rhs - mask.cwiseProduct(Mx);
	  solver.init( residual );
	  restart();
	  
	  vec old_z, dz;
	  vec old_x, dx;

	  vec best = x;
	  real min = 1e42;

	  unsigned k;
	  for(k = 0; k < it.max; ++k) {
		old_z = z;
		old_x = x;

		const real phi_pre = residual.norm();
		solver.step(x, A);
		solver.info.step(Mx, Md, Mdprev, std::move(My));
		Mx.noalias() = M(x);

		residual = mask.cwiseProduct(Mx + q);
		const real phi_post = residual.norm();
		
		// this should not happen in exact arithmetic
		if( phi_post > phi_pre ) {
		  // tool::log(k, "residual norm increase on minres step!",
		  // 			"pre", phi_pre, "post", phi_post);
		  
		  const real delta = (x - old_x).norm();
		  // tool::log(k, "delta norm", delta);
		  
		  // we stagnate completely, abort
		  if( delta < it.precision ) {
			// core::log(k, "premature exit");
			// break;
		  }

		  // restart
		  residual = mask.cwiseProduct(Mx + q);
		  solver.init(residual);
		  restart();
		}
		
		// update z
		z = Mx + q - x;
		
		// deltas
		dz = z - old_z;
		dx = x - old_x;
		
		unsigned index_x, index_z;
		
		// track first index to become negative
		const real alpha_z = lcp::track_notin(old_z, dz, &index_z, mask); // TODO only for inactive
		const real alpha_x = lcp::track(old_x, dx, &index_x);
		
		// whichever comes first
		const real alpha = std::min(alpha_x, alpha_z);
		
		// sign change before step end ?
		if( alpha < 1 ) {

		  // tool::log(k, "alpha:", alpha);
		  
		  const real active = (alpha == alpha_z) ? 1 : 0;
		  const unsigned index = active ? index_z : index_x;
		  
		  // advance to active-set change
		  x = old_x + alpha * dx;
		  z = old_z + alpha * dz;
		  
		  // update mask
		  // mask = (z.array() < 0).cast<real>();
		  mask = ((x.array() > 0) || ((x.array() == 0) && (z.array() < 0) )).cast<real>();

		  // force active-set change on x even though z is (numerically)
		  // negative even though x went negative first
		  // if( !active && !x(index) ) mask(index) = active;
		  // mask(index) = active;
		  
		  // update residual
		  residual = -mask.cwiseProduct(z + x);
		  
		  // this should not happen, except on numerical issues
		  const real phi_proj = residual.norm();
		  if( phi_proj > phi_pre ) {
			// core::log(k, "residual norm increase on projection!",
			// 		  "pre", phi_pre, "post", phi_post, "proj", phi_proj,
			// 		  "x", x(index), "z", z(index));
		  }

		  // just in case, again for numerical reasons
		  x = mask.cwiseProduct( x );
		  rhs = -mask.cwiseProduct( q );
		  
		  // restart minres
		  solver.init( residual );
		  restart();
		  Mx = z + x - q;
		  Mx.noalias() = M(x);
		}

		w = Mx + q;
		const real error = w.cwiseMin(x).norm();
		
		if( it.cb ) it.cb(k, x);

		if( error < min ) {
		  best = x;
		  min = error;
		}

		// tool::log(k, "error", error, "residual", solver.phi, "precision", it.precision);
		
		if( error <= it.precision ) {
		  break; 
		}

	  }

	  tool::log("iterations:", k);
	  if( k == it.max ) tool::log("no convergence !");
	  
	  x = best;
	};

  };

  

}


#endif
