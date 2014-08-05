#include "pgs.h"

#include <sofa/core/ObjectFactory.h>
#include <Compliant/utils/scoped.h>
#include <Compliant/utils/edit.h>


// SOFA_DECL_CLASS(SequentialSolver);
int pgsClass = sofa::core::RegisterObject("pgs")
  .add< pgs >()
  .addAlias("pouf.pgs");

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
	
  // TODO: temporary :-/
  response->solve(tmp, JP.transpose());
  mapping_response = system.P * tmp;
	
  // build blocks and factorize
  for(unsigned i = 0; i < n; ++i) {
	const block& b = blocks[i];

	// virew on inverse chunk data
	view_type view(inverse_storage.data() + offsets[i], b.size, b.size);

	// temporary sparse mat, difficult to remove :-/
	const cmat tmp = JP.middleRows(b.offset, b.size) * 
	  mapping_response.middleCols(b.offset, b.size);
		
	// fill constraint block
	view = tmp;
		
	// add diagonal C block
	for( unsigned r = 0; r < b.size; ++r) {
	  for(system_type::mat::InnerIterator it(system.C, b.offset + r); it; ++it) {
				
		// paranoia, i has it
		assert( it.col() >= int(b.offset) );
		assert( it.col() < int(b.offset + b.size) );
				
		view(r, it.col() - int(b.offset)) += it.value();
	  }
	}

	inverse_type inv( view );
	view = inv.solve( dense_matrix::Identity( b.size, b.size ) );
	
  }

}



static pgs::real track(const pgs::vec& prev,
					   const pgs::vec& delta, unsigned* index = 0,
					   pgs::real epsilon = 0) {
  
  pgs::real alpha = 1;

  for(unsigned i = 0, n = delta.size(); i < n; ++i) {
	  
	  if( delta(i) ) {
		const pgs::real value = - prev(i) / delta(i);

		if( value >= epsilon && value < alpha ) {
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

  result.noalias() = const_view_type( inverse_storage.data() + offsets[i],
									  rhs.size(), rhs.size() ) * rhs;

}



void pgs::solve_impl(vec& res,
					 const system_type& sys,
					 const vec& rhs,
					 bool correct) const {
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
	
	// lcp error
	vec error = vec::Zero( sys.n );
	
	const real epsilon = relative.getValue() ? 
		constant.norm() * precision.getValue() : precision.getValue();

	if( this->bench ) this->bench->lcp(sys, constant, *response, lambda);


	// outer loop
	unsigned k = 0, max = iterations.getValue();
	vec primal;

	// nlnscg
	vec lambda_prev, grad_prev, p, grad, next, diff;

	// accel
	dense_matrix G, F, K;
	vec delta2;
	
	const unsigned acc = accel.getValue();
	const unsigned acc_alt = accel_alt.getValue();
	
	if( acc || acc_alt) {
	  unsigned dim = std::max(acc, acc_alt);
	  G = dense_matrix::Zero(sys.n, dim);
	  F = dense_matrix::Zero(sys.n, dim);
	  K = dense_matrix::Zero(dim, dim);
	}

	inverse_type inv;

	unsigned s1 = convergence.getValue().size();
 
	for(k = 0; k < max; ++k) {

	  lambda_prev = lambda;
	  
	  real estimate2 = step( lambda, net, sys, constant, error, delta, correct );

	  if( nlnscg.getValue() ) {
		grad = -lambda + lambda_prev;

		// conjugation
		if( k > 0 ) {
			
		  assert( grad_prev.norm() > std::numeric_limits<real>::epsilon() );
		  real beta = grad.squaredNorm() / grad_prev.squaredNorm();
			
		  if( beta > 1 ) {
			// restart
			p.setZero();
		  } else {
			// conjugation
			lambda += beta * p;
			p = beta * p - grad;
		  }
		} else {
		  // first iteration
		  p = -grad;
		}
		
		grad_prev = grad;
	  }


	  if( acc && lambda != lambda_prev ) {
		const unsigned index = k % acc;

		G.col(index) = lambda;
		F.col(index) = lambda - lambda_prev;

		tmp.noalias() = F.transpose() * F.col(index);
		
		K.col(index) = tmp;
		K.row(index) = tmp.transpose();

		inv.compute( K ); 
		tmp = inv.solve( vec::Ones(acc) );

		const real lambda_inv = tmp.sum();
		tmp /= lambda_inv;

		next.noalias() = G * tmp;
		diff = next - lambda;
		
		const real alpha = track(lambda, diff);
		lambda += alpha * diff;
	  }


	  if( acc_alt && lambda != lambda_prev ) {
		const unsigned index = k % acc_alt;

		G.col(index) = lambda;
		F.col(index) = lambda - lambda_prev;

		// const unsigned index_prev = (k + acc_alt - 1) % acc_alt;
		// const unsigned index_next = (k + 1) % acc_alt;
		
		// vec p = F.col(index_prev);
		// vec r = F.col(index);

		// vec Ap = JP * (mapping_response * p);

		// real pAp = p.dot(Ap);
		// if( pAp ) { 
		//   real mu = -r.dot(Ap) / pAp;
		//   p = r + mu * p;

		//   F.col(index_next) = p;
		//   G.col(index_next) = lambda + p;
		// }

		tmp.noalias() = F.transpose() * F.col(index);
		K.col(index) = tmp;
		K.row(index) = tmp.transpose();

		// K = F.transpose() * F;

		// std::cout << K << std::endl;

		inv.compute( K ); 
		tmp = inv.solve( vec::Ones(acc_alt) );

		const real lambda_inv = tmp.sum();
		tmp /= lambda_inv;

		next.noalias() = G * tmp;
		lambda = next.cwiseMax( vec::Zero(sys.n) );

		// diff = next - lambda;
		
		// const real alpha = track(lambda, diff);
		// lambda += alpha * diff;
	  }

	  
		
	  if( this->bench ) this->bench->lcp(sys, constant, *response, lambda);


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
	accel(initData(&accel, unsigned(0), "accel", "anderson acceleration")),
	accel_alt(initData(&accel_alt, unsigned(0), "accel_alt", "custom anderson acceleration")),
	log(initData(&log, false, "log", "log convergence history")),
	convergence(initData(&convergence, "convergence", "convergence history (read-only)"))
{

}




void pgs::fetch_blocks(const system_type& system) {
  using namespace sofa;
  
	// TODO don't free memory ?
	blocks.clear();
	
	unsigned off = 0;

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
