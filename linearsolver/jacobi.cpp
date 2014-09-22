#include "jacobi.h"

#include <sofa/core/ObjectFactory.h>
#include <Compliant/utils/scoped.h>
#include <Compliant/utils/edit.h>
#include <Compliant/utils/schur.h>
#include <Compliant/utils/nan.h>

#include <tool/lcp.h>

#include <thread/pool.h>

#include <Compliant/constraint/CoulombConstraint.h>

#include <math/anderson.h>
#include <math/nlnscg.h>

// SOFA_DECL_CLASS(SequentialSolver);
int jacobiClass = sofa::core::RegisterObject("jacobi")
  .add< jacobi >()
  .addAlias("pouf.jacobi");


static void response_solve(thread::pool& pool,
						   const sofa::component::linearsolver::Response& response,
						   sofa::component::linearsolver::AssembledSystem::cmat& res,
						   const sofa::component::linearsolver::AssembledSystem::rmat& rhs) {
  scoped::timer timer("response computation");
  
  using namespace sofa::component::linearsolver;
  std::vector<AssembledSystem::cmat> chunk(pool.size());

  std::mutex mutex;
  
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




static void parallel_prod(const thread::pool& pool,
						  jacobi::vec& res,
						  const jacobi::cmat& mat,
						  const jacobi::vec& rhs,
						  jacobi::dense_matrix& buffer ) {
  const unsigned m = mat.rows();
  const unsigned n = mat.cols();
  const unsigned k = pool.size();

  buffer.resize(mat.rows(), pool.size());
  res.resize(mat.rows());

  auto task = [&]( const thread::pool::chunk& c ) {
	const unsigned dim = c.end - c.start;
	buffer.col(c.id).noalias() = mat.middleCols(c.start, dim) * rhs.segment(c.start, dim);
  };
  
  pool.parallel_for(0, n, task);
  
  res.noalias() = buffer * jacobi::vec::Ones(k);
}


void jacobi::reset() {

  base::reset();

  pool = thread::pool( threads.getValue() );
}


void jacobi::factor(const system_type& sys) {
  scoped::timer timer("system factorization");

  if( log.getValue() ) {
    edit(convergence)->clear();
  }

  assert( response );
  response->factor( sys.H );
  
  // find blocks
  fetch_blocks(sys);

  // compute block responses
  const unsigned n = blocks.size();
  
  if( !n ) return;

  // mapping responses
  mapping_response.resize( sys.m, sys.n );
  
  JP = sys.J * sys.P;
  cmat tmp; tmp.resize( sys.m, sys.n );
  
  response_solve(pool, *response, tmp, JP);
  mapping_response = sys.P * tmp;
  
  // mass-splitting
  vec prec = vec::Zero( sys.m );

  // count non-zero entries per row of mapping_response
  for(int k = 0; k < mapping_response.outerSize(); ++k) {
	for(cmat::InnerIterator it(mapping_response, k); it; ++it) {
	  prec( it.row() ) += 1;
	}
  }

  diagonal = vec::Zero(sys.n);
  
  // schur complement
  // TODO parallel
  for(unsigned i = 0; i < sys.n; ++i) {
	diagonal(i) = JP.row(i).dot(prec.asDiagonal() * mapping_response.col(i) );
  }

  // compliance
  // TODO figure out precise convergence conditions apart from diagonal C
  for(int k = 0; k < sys.C.outerSize(); ++k) {
  	for(mat::InnerIterator it(sys.C, k); it; ++it) {

  	  if( it.row() == it.col() ) {
  		diagonal(k) += it.value();
  		break;
  	  }
  	}
  };


  // maximal over-relaxation yo !
  diagonal /= 2;

  using namespace sofa::component::linearsolver;

  // homogenize tangent directions to get anisotropic friction
  if( homogenize.getValue() ) {

	for(unsigned i = 0, n = blocks.size(); i < n; ++i) {
	  const block& b = blocks[i];

	  if( dynamic_cast< CoulombConstraint* >(b.projector) ) {
		const real value = diagonal.segment<2>(b.offset + 1).maxCoeff();
		diagonal.segment<2>(b.offset + 1).setConstant( value );
	  }

	}

  }  
}




void jacobi::solve_impl(vec& res,
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

  // lagrange multipliers
  vec lambda = res.tail(sys.n); 

  vec net = mapping_response * lambda;
	
  // lcp rhs
  vec constant = rhs.tail(sys.n) - JP * res.head( sys.m );

  // constraint error
  vec primal = JP * net - constant + sys.C * lambda;
  if( damping ) primal += damping * lambda;

  const real epsilon = relative.getValue() ? 
	constant.norm() * precision.getValue() : precision.getValue();
	
  if( this->bench ) this->bench->lcp(sys, constant, *response, lambda);

  // main loop
  unsigned k = 0, max = iterations.getValue();
	
  // BECAUSE I SAY SO
  if( correct ) max /= 2;

  dense_matrix buffer;
  
  math::anderson accel_anderson(sys.n, anderson.getValue());
  math::nlnscg accel_nlnscg;

  vec lambda_prev, f = vec::Zero(sys.n);
  
  const unsigned cv_size = convergence.getValue().size();
  for(k = 0; k < max; ++k) {

	lambda_prev = lambda;
	
	// vanilla jacobi
	lambda.array() -= primal.array() / (diagonal.array() + damping);
	  
	// projection
	auto project = [&](const thread::pool::chunk& c) {

	  for(unsigned i = c.start; i < c.end; ++i) {
		const block& b = blocks[i];
		chunk_type lambda_chunk(&lambda(b.offset), b.size);
		
		if( b.projector ) {
		  b.projector->project( lambda_chunk.data(), lambda_chunk.size(), correct );
		  assert( !has_nan(lambda_chunk.eval()) );
		}
	  }
	};
	pool.parallel_for(0, blocks.size(), project);
	
	// acceleration
	if( anderson.getValue() ) {
	  f = diagonal.array().sqrt() * ( lambda - lambda_prev ).array();
	  accel_anderson.step(lambda, lambda, f);
	}

	if( nlnscg.getValue() ) {
	  f = ( lambda - lambda_prev ).array();
	  accel_nlnscg.step(lambda, f);
	}


	// update stuff
	// TODO parallel
	// net.noalias() = mapping_response * lambda;
	parallel_prod(pool, net, mapping_response, lambda, buffer);
	
	primal.noalias() = JP * net - constant + sys.C * lambda;
	if( damping ) primal += damping * lambda;
	
	const real error = lambda.cwiseMin(primal).norm();

	// TODO good stop criterion
	if( log.getValue() ) {
	  edit(convergence)->push_back(error);
	}

	if( error <= epsilon ) break;
	  
  }
	
  res.head( sys.m ) += net;
  res.tail( sys.n ) = lambda;
	
  if(log.getValue()){
	edit(convergence)->push_back(convergence.getValue().size() - cv_size);
  }


}

jacobi::jacobi()
  : nlnscg(initData(&nlnscg, false, "nlnscg", "non-smooth non-linear cg")),
	anderson(initData(&anderson, unsigned(0), "anderson", "anderson acceleration")),
	log(initData(&log, false, "log", "log convergence history")),
	convergence(initData(&convergence, "convergence", "convergence history (read-only)")),
	filename(initData(&filename, "filename", "dump lcp data to filename")),
	homogenize(initData(&homogenize, true, "homogenize", "homogenize tangent directions")),
	threads(initData(&threads, unsigned(1), "threads", "number of concurrent threads"))
{
  
}

void jacobi::solve(vec& res,
				   const system_type& sys,
				   const vec& rhs) const {
  solve_impl(res, sys, rhs, false );
}


void jacobi::correct(vec& res,
					 const system_type& sys,
					 const vec& rhs,
					 real damping ) const {
  solve_impl(res, sys, rhs, true, damping );
}












void jacobi::fetch_blocks(const system_type& system) {
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
