#include "pgs.h"

#include <sofa/core/ObjectFactory.h>
#include <Compliant/utils/scoped.h>
#include <Compliant/utils/edit.h>
#include <Compliant/utils/schur.h>
#include <Compliant/utils/nan.h>

#include <Compliant/constraint/CoulombConstraint.h>

#include <tool/lcp.h>

#include <math/anderson.h>
#include <math/nlnscg.h>


// SOFA_DECL_CLASS(SequentialSolver);
int pgsClass = sofa::core::RegisterObject("pgs")
  .add< pgs >()
  .addAlias("pouf.pgs");




typedef sofa::component::linearsolver::Response response_type;
static void response_solve(thread::pool& pool,
						   const response_type& response,
						   sofa::component::linearsolver::AssembledSystem::cmat& res,
						   const sofa::component::linearsolver::AssembledSystem::rmat& rhs) {
  scoped::timer timer("response computation");
  
  using namespace sofa::component::linearsolver;
  std::vector<AssembledSystem::cmat> chunk(pool.size());

  auto task = [&](const thread::pool::chunk& c) {
	chunk[c.id].resize(rhs.rows(), c.end - c.start);
	
	AssembledSystem::cmat tmp(res.rows(), 1);
	
	for(unsigned i = c.start; i < c.end; ++i) {
	  tmp.setZero();
	  response.solve(tmp, rhs.middleRows(i, 1).transpose());
	  chunk[c.id].middleCols(i - c.start, 1) = tmp;
	}	
	
  };

  pool.parallel_for(0, rhs.rows(), task);

  unsigned off = 0;
  for(unsigned i = 0, n = pool.size(); i < n; ++i) {
	unsigned dim = chunk[i].cols();

	if(dim) {
	  res.middleCols(off, dim) = chunk[i];
	}

	off += dim;

  }
}


void pgs::factor(const system_type& system) { 
  scoped::timer timer("system factorization");

  if( log.getValue() ) {
    edit(convergence)->clear();
  }

  typedef sofa::component::linearsolver::Benchmark benchmark_type;

  benchmark_type::scoped_timer bench_timer(this->bench, &benchmark_type::factor);

  assert( response );
  response->factor( system.H );
	
  // find blocks
  fetch_blocks(system);

  // compute block responses
  const unsigned n = blocks.size();
	
  if( !n ) return;

  // compute offsets
  offsets.resize(n);
  unsigned off = 0;
  for(unsigned i = 0; i < n; ++i ){
	offsets[i] = off;
	off += blocks[i].size * blocks[i].size;
  }

  // hack: we still resize blocks_inv to get index during solve_block
  blocks_inv.resize( n );
	
  // inverse storage resize
  if( off > inverse_storage.size() ) {
	inverse_storage.resize( off );
  }


  // mapping responses
  mapping_response.resize( system.J.cols(), system.J.rows() );

  JP = system.J * system.P;
  cmat tmp; tmp.resize( mapping_response.rows(),
						mapping_response.cols());

  response_solve(pool, *response, tmp, JP);
  mapping_response = system.P * tmp;

  // diagonal
  diagonal.resize(system.J.rows());

  // build blocks and factorize
  auto task = [&](const thread::pool::chunk& c) {
	for(unsigned i = c.start; i < c.end; ++i) {
	  const block& b = blocks[i];

	  // virew on inverse chunk data
	  view_type view(inverse_storage.data() + offsets[i], b.size, b.size);

	  // temporary sparse mat, difficult to remove :-/
	  const cmat tmp = (JP.middleRows(b.offset, b.size) * 
						mapping_response.middleCols(b.offset, b.size)).triangularView<Eigen::Upper>();
	
	  // fill constraint block
	  view = tmp.triangularView<Eigen::Upper>();

	  // add diagonal C block
	  for( unsigned r = 0; r < b.size; ++r) {
		for(system_type::mat::InnerIterator it(system.C, b.offset + r); it; ++it) {
				
		  // paranoia, i has it
		  assert( it.col() >= int(b.offset) );
		  assert( it.col() < int(b.offset + b.size) );
		
		  view(r, it.col() - int(b.offset)) += it.value();
		}

	  }


	  if( homogenize.getValue() and
		  b.projector and 
		  typeid(*b.projector) == typeid(sofa::component::linearsolver::CoulombConstraint) ) {
		math::vec3 tmp;

		// can't do it directly, we need a temporary. this is a bug in
		// eigen.
		tmp = view.diagonal();
		view = tmp.asDiagonal();
		
		const real value = view.diagonal().segment<2>(1).maxCoeff();
		view.diagonal().segment<2>(1).setConstant( value );
	  }

	  diagonal.segment(b.offset, b.size) = view.diagonal();
	  
	  inverse_type inv( view.selfadjointView<Eigen::Upper>() );
	  view = inv.solve( dense_matrix::Identity( b.size, b.size ) );
	}	
  };
  
  {
	scoped::timer timer("building blocks");
	// task(0, n);
	pool.parallel_for(0, n, task);
  }

}



static pgs::real track(const pgs::vec& prev,
					   const pgs::vec& delta, unsigned* index = 0,
					   pgs::real epsilon = 0) {
  
  pgs::real alpha = 1;

  for(unsigned i = 0, n = delta.size(); i < n; ++i) {
	  
	  if( delta(i) ) {
		const pgs::real value = - prev(i) / delta(i);

		if( value >= epsilon and value < alpha ) {
		  alpha = value;
		  if( index ) *index = i;
		}
	  }

	};

  return alpha;
}


void pgs::solve_block(chunk_type result, const inverse_type& inv, chunk_type rhs) const {

  // i smell hack
  const unsigned i = &inv - &blocks_inv.front();

  result.noalias() = const_view_type(inverse_storage.data() + offsets[i],
									 rhs.size(), rhs.size()).selfadjointView<Eigen::Upper>() * rhs;

}

void pgs::reset() {
  SequentialSolver::reset();
  const int n = threads.getValue();
  pool = thread::pool( std::max<int>(0, n - 1) );

}


void pgs::solve_impl(vec& res,
					 const system_type& sys,
					 const vec& rhs,
					 bool correct,
					 real damping) const {
  

	assert( response );

	// reset bench if needed
	if( this->bench ) {
		bench->clear();
		bench->restart();
	}

	// free velocity
	vec tmp( sys.m );
	
	response->solve(tmp, sys.P.selfadjointView<Eigen::Upper>() * rhs.head( sys.m ) );
	res.head(sys.m).noalias() = sys.P.selfadjointView<Eigen::Upper>() * tmp;
	
	// we're done lol
	if( !sys.n ) return;

	
	// lagrange multipliers TODO reuse res.tail( sys.n ) ?
	vec lambda = res.tail(sys.n); 

	// net constraint velocity correction
	vec net = mapping_response * lambda;
	
	// lambda change work vector
	vec delta = vec::Zero( sys.n );
	
	// lcp rhs 
	vec constant = rhs.tail(sys.n) - JP * res.head( sys.m );

	// lcp error (work vector)
	vec error = vec::Zero( sys.n );
	
	const real epsilon = relative.getValue() ? 
		constant.norm() * precision.getValue() : precision.getValue();

	if( this->bench ) this->bench->lcp(sys, constant, *response, lambda);


	// outer loop
	unsigned k = 0, max = iterations.getValue();

	// BECAUSE I SAY SO
	// if( correct ) max /= 2;
	
	vec primal;

	vec lambda_prev;

	vec f, net_prev, Ng, Np = vec::Zero(sys.m);

	// math::anderson_new accel_anderson(sys.n, anderson.getValue());
	math::nlnscg accel_nlnscg(sys.n);

	unsigned s1 = convergence.getValue().size();
 
	for(k = 0; k < max; ++k) {
	  
	  // acceleration
	  // if( anderson.getValue() ) {
	  // 	// accel_anderson.step(lambda, primal, diagonal, unilateral_mask);
	  // }
	  
	  lambda_prev = lambda;
	  net_prev = net;

	  // net = mapping_response * lambda;
	  real estimate2 = step( lambda, net, sys, constant, error, delta, correct );

	  if( nlnscg.getValue() ) {
		f = ( lambda - lambda_prev ).array();
		
		const real beta = accel_nlnscg.step(lambda, f, diagonal);

		if( beta > 1 ) {
		  Np = vec::Zero(sys.m);
		} else {
		  Ng = net - net_prev;
		  net += beta * Np;
		  Np = beta * Np + Ng;
		}

	  }


	  // if( nlnscg.getValue() ) {

	  // 	const real grad_norm2_prev = grad_norm2;
		
	  // 	grad = -lambda + lambda_prev;
	  // 	Ng = -net + net_prev;
		
	  // 	Grad_norm2 = grad.squaredNorm();

	  // 	// conjugation
	  // 	if( k > 0 and grad_norm2_prev) {
			
	  // 	  assert( grad_norm2_prev > std::numeric_limits<real>::epsilon() );
	  // 	  const real beta = grad_norm2 / grad_norm2_prev;
		  
	  // 	  if( beta > 1 ) {
	  // 		// restart
	  // 		p.setZero();
	  // 		Np.setZero();
			
	  // 	  } else {
	  // 		// conjugation
	  // 		lambda += beta * p;
	  // 		net += beta * Np;

	  // 		// net = mapping_response * lambda;

	  // 		p = beta * p - grad;
	  // 		Np = beta * Np - Ng;
	  // 	  }
	  // 	} else {
	  // 	  // first iteration
	  // 	  p = -grad;
	  // 	  Np = -Ng;
	  // 	}
		
	  // 	grad_prev.swap(grad);
	  // }

	  if( log.getValue() ) {
		primal = JP * net - constant;
		real error = lambda.cwiseMin(primal).norm();
		edit(convergence)->push_back(error);
		if( error <= epsilon ) break;
	  } else { 
		// stop if we only gain one significant digit after precision
		if( std::sqrt(estimate2) / sys.n <= epsilon ) break;
	  }
	}
	
	// std::cerr << "sanity check: " << (net - mapping_response * lambda).norm() << std::endl;

	res.head( sys.m ) += net;
	res.tail( sys.n ) = lambda;
	
	if(log.getValue()){
	  edit(convergence)->push_back(convergence.getValue().size() - s1);
	}
}

pgs::pgs()
  : nlnscg(initData(&nlnscg, false, "nlnscg", "non-smooth non-linear cg")),
	accel(initData(&accel, unsigned(0), "anderson", "anderson acceleration")),
	log(initData(&log, false, "log", "log convergence history")),
	threads(initData(&threads, unsigned(1), "threads", "number of additional concurrent threads")),
	homogenize(initData(&homogenize, true, "homogenize", "homogenize tangent directions")),
	convergence(initData(&convergence, "convergence", "convergence history (read-only)")),
	filename(initData(&filename, "filename", "dump lcp data to filename"))
{

}

void pgs::solve(vec& res,
				const system_type& sys,
				const vec& rhs) const {
  solve_impl(res, sys, rhs, false );
}


void pgs::correct(vec& res,
				  const system_type& sys,
				  const vec& rhs,
				  real damping ) const {
  solve_impl(res, sys, rhs, true, damping );
}




void pgs::fetch_blocks(const system_type& system) {
  using namespace sofa;
  
	// TODO don't free memory ?
	blocks.clear();
	
	unsigned off = 0;

	// TODO parallelize
	for(unsigned i = 0, n = system.compliant.size(); i < n; ++i) {
		system_type::dofs_type* const dofs = system.compliant[i];

		const unsigned dim = dofs->getDerivDimension();
		
		for(unsigned k = 0, max = dofs->getSize(); k < max; ++k) {
			
			block b;

			b.offset = off;
			b.size = dim;
            b.projector = dofs->getContext()->get<component::linearsolver::Constraint>( core::objectmodel::BaseContext::Local );
			
            assert( !b.projector || b.projector->mask.empty() || b.projector->mask.size() == max );
            b.activated = true;
			

			blocks.push_back( b );

			off += dim;
		}
	}

}
