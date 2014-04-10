#include "pgs.h"

#include <sofa/core/ObjectFactory.h>
#include <utils/scoped.h>

// SOFA_DECL_CLASS(SequentialSolver);
int pgsClass = sofa::core::RegisterObject("pgs")
  .add< pgs >()
  .addAlias("pouf.pgs");

void pgs::factor(const system_type& system) { 
  scoped::timer timer("system factorization");

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
	
  // avoid allocating matrices for each block
  vec storage;

  inverse_type inv;
  // build blocks and factorize
  for(unsigned i = 0; i < n; ++i) {
	const block& b = blocks[i];
		
	// resize storage if needed TODO alloc max size only once
	if( b.size * b.size > storage.size() ) storage.resize(b.size * b.size);
		
	// view on storage
	schur_type schur(storage.data(), b.size, b.size);
		
	// temporary sparse mat, difficult to remove :-/
	const cmat tmp = JP.middleRows(b.offset, b.size) * 
	  mapping_response.middleCols(b.offset, b.size);
		
	// fill constraint block
	schur = tmp;
		
	// add diagonal C block
	for( unsigned r = 0; r < b.size; ++r) {
	  for(system_type::mat::InnerIterator it(system.C, b.offset + r); it; ++it) {
				
		// paranoia, i has it
		assert( it.col() >= int(b.offset) );
		assert( it.col() < int(b.offset + b.size) );
				
		schur(r, it.col() - int(b.offset)) += it.value();
	  }
	}

	inv.compute( schur );

	// view on inverse chunk
	view_type(inverse_storage.data() + offsets[i], b.size, b.size)
	  = inv.solve( dense_matrix::Identity( b.size, b.size ) );
		
  }

}

void pgs::solve_block(chunk_type result, const inverse_type& inv, chunk_type rhs) const {

  // i smell hack
  const unsigned i = &inv - &blocks_inv.front();

  result.noalias() = const_view_type( inverse_storage.data() + offsets[i],
									  rhs.size(), rhs.size() ) * rhs;

}
