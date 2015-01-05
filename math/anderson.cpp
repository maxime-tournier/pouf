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

	const unsigned prev = (index + m - 1) % m;

	// TODO optimize previously computed norm
	if( dx.squaredNorm() > F.col(prev).squaredNorm() ) {
	  // reset yo
	  
	  G = mat::Zero(n, m);
	  F = mat::Zero(n, m);
	  K = mat::Zero(m, m);
	}

	G.col(index) = x;
	F.col(index) = dx;

	tmp.noalias() = F.transpose() * F.col(index);

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

	index = (index + 1) % m;
  }



  anderson_new::anderson_new(unsigned n, unsigned m):
	anderson(n, m),
	df2(0) {

	if( m ) {
	  P = mat::Zero(n, m);
	}
  }


  void anderson_new::step(vec& dual, vec& primal, const vec& diag, const vec& unilateral) {
	if( !m ) return;

	if( false && m == 2 ) {
	  step_opt(dual, primal, diag, unilateral);
	  return;
	}

	auto feature = [&](vec& out, const vec& d, const vec& p) {
	  out = unilateral.array() * (d.array()).min( p.array() )
	  + (1 - unilateral.array()) * p.array();
	  
	  // out = unilateral.array() * d.array().min( p.array() )
	  // + (1 - unilateral.array()) * p.array();
	};


	G.col(index) = dual;
	P.col(index) = primal;
	
	feature(f, dual, primal);
	F.col(index) = f;
	
	// tmp.noalias() = F.transpose() * (F.col(index).array() / diag.array()).matrix() ;
	tmp.noalias() = F.transpose() * F.col(index);
	
	K.col(index) = tmp;
	K.row(index) = tmp.transpose();
	 
	inv.compute( K ); 
	tmp = inv.solve( vec::Ones(m) );

	const real c = tmp.sum();
	
	if( c ) {
	  tmp /= c;

	  next_dual.noalias() = G * tmp;
	  next_primal.noalias() = P * tmp;

	  feature(next_f, next_dual, next_primal);
	  
	  if( next_f.squaredNorm() <= F.col(index).squaredNorm() ) {
		dual.swap( next_dual );
		primal.swap( next_primal );
	  } else {
 		// K = mat::Zero(m, m);
		// F = mat::Zero(n, m);
	  }

	} else {
	  std::cout << "DERP" << std::endl;
	}


	index = (index + 1) % m;
  }


  void anderson_new::step_opt(vec& dual, vec& primal, const vec& diag, const vec& unilateral) {
	if( !m ) return;
	
	G.col(index) = dual;
	P.col(index) = primal;

	auto feature = [&](vec& out, const vec& d, const vec& p) {
	  out = unilateral.array() * (diag.array() * d.array()).min( p.array() )
	  + (1 - unilateral.array()) * p.array();
	  
	  // out = unilateral.array() * d.array().min( p.array() )
	  // + (1 - unilateral.array()) * p.array();
	};
	
	feature(f, dual, primal);
	F.col(index) = f;

	const unsigned prev = (index + m - 1) % m;	
	next_f = F.col(index) - F.col(prev);

	real old_df2 = df2;
	df2 = next_f.squaredNorm();
	
	real alpha = 0.5;

	if( df2 ) {
	  alpha = -F.col(prev).dot(next_f) / df2;
	}

	next_dual.noalias() = G.col(prev) + alpha * ( G.col(index) - G.col(prev));
	next_primal.noalias() = P.col(prev) + alpha * ( P.col(index) - P.col(prev));

	feature(next_f, next_dual, next_primal );
	
	if( next_f.squaredNorm() < old_df2 ) {
	  dual.swap( next_dual );
	  primal.swap( next_primal );
	}

	index = (index + 1) % m;
  }


}
