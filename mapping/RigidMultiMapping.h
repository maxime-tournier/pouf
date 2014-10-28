#ifndef RIGIDMULTIMAPPING_H
#define RIGIDMULTIMAPPING_H

#include <Compliant/mapping/AssembledMultiMapping.h>
#include <Compliant/utils/se3.h>
#include <Compliant/utils/map.h>

#include "../init.h"

namespace sofa {
  namespace component {
	namespace mapping {

	  // RigidMapping from multiple source dofs
	  template <class TIn, class TOut >
	  class SOFA_pouf_API RigidMultiMapping : public AssembledMultiMapping<TIn, TOut> {
		public:
		SOFA_CLASS(SOFA_TEMPLATE2(RigidMultiMapping,TIn,TOut), 
				   SOFA_TEMPLATE2(AssembledMultiMapping,TIn,TOut));
		
		typedef defaulttype::Vec<2, unsigned> index_pair;

		// note: indices should be sorted TODO actually, no ?
		typedef vector< index_pair > pairs_type;
		Data< pairs_type > pairs; // parent (mstate, index) for each child.

		typedef defaulttype::Vec<3, SReal> vec3;
		typedef vector< vec3 > local_type;
		Data<local_type> local;

		
		typedef typename TIn::Real in_real;
		typedef typename TOut::Real out_real;
		
		typedef SE3< in_real > se3;
		typedef typename se3::coord_type coord_type;
		typedef RigidMultiMapping self;

		
		RigidMultiMapping()
		  : pairs(initData(&pairs, "pairs", "parent (mstate, index) for each child, ***lexicographically sorted***")),
			local(initData(&local, "local", "local coordinates for each child, repeated if needed (zero if none)"))
		{
		  
		}


		void apply(typename self::out_pos_type& out,
				   const vector< typename self::in_pos_type >& in ) {

		  const pairs_type& p = pairs.getValue();
		  const local_type& loc = local.getValue();
			
		  assert(out.size() == p.size());

		  vec3 curr(0, 0, 0);
		  
		  for( unsigned i = 0, n = p.size(); i < n; ++i) {
			
			const coord_type src = in[ p[i][0] ][ p[i][1] ];
			
			const int index = std::min(int(i), int(loc.size()) - 1);

			if( index >= 0 ) {
			  curr = loc[index];
			}

			out[i] = src.projectPoint(curr);
		  }
		  

		}


		void assemble(const vector< typename self::in_pos_type >& in ) {
		  assert(this->getFrom()[0] != this->getFrom()[1]);
		  
		  const pairs_type& p = pairs.getValue();
		  
		  typedef typename se3::mat66 mat66;
		  typedef typename se3::mat33 mat33;
		  typedef typename se3::mat36 mat36;

		  typedef typename self::jacobian_type::CompressedMatrix matrix_type;
		  
		  // resize/clean jacobians
		  for(unsigned j = 0, m = in.size(); j < m; ++j) {
			matrix_type& J = this->jacobian(j).compressedMatrix;
			J.resize( 3 * p.size(), 
			          6 * in[j].size() );
			J.setZero();
		  }
		

		  const local_type& loc = local.getValue();

		  vec3 curr(0, 0, 0);
		  mat36 block;

		  block.template leftCols<3>().setIdentity();
		  
		  for(unsigned i = 0, n = p.size(); i < n; ++i) {

			const unsigned j = p[i][0];
			matrix_type& J = this->jacobian(j).compressedMatrix;

			// index in parent
			const unsigned k = p[i][1];

			const coord_type src = in[ p[i][0] ][ p[i][1] ];
			const int index = std::min(int(i), int(loc.size()) - 1);
			
			if( index >= 0 ) {
			  curr = loc[index];
			}

			// TODO optimize if curr is zero
			block.template rightCols<3>() = - se3::hat( se3::rotation(src) * utils::map(curr).cast<in_real>() );

			for( unsigned a = 0; a < 3; ++a) {
			  const unsigned row = 3 * i + a;
			  J.startVec( row );
			  
			  for( unsigned b = 0; b < 6; ++b) {
				const unsigned col = 6 * k + b;

				if( block(a, b) ) {
				  J.insertBack(row, col) = block(a, b);
				}
				
			  }
			}


		  }

		  for(unsigned j = 0, m = in.size(); j < m; ++j) {
			matrix_type& J = this->jacobian(j).compressedMatrix;
			J.finalize();
		  }

		}
		  
	  };
	  
	}
  }
}

#endif
