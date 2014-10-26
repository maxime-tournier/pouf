#ifndef POUF_TOOL_SPARSE_H
#define POUF_TOOL_SPARSE_H

#include <sofa/component/linearsolver/EigenBaseSparseMatrix.h>
#include <stdexcept>

namespace tool {

// applies Op on the good cast to an eigen matrix
template<class U, class BaseMatrix, class Op>
void eigen_cast(const BaseMatrix* m,
				const Op& op) {

	typedef sofa::component::linearsolver::EigenBaseSparseMatrix<double> matrixd;

    const matrixd* smd = dynamic_cast<const matrixd*> (m);
    if ( smd ) {
		op( smd->compressedMatrix.template cast<U>() );
		return;		
	}

    typedef sofa::component::linearsolver::EigenBaseSparseMatrix<float> matrixf;

    const matrixf* smf = dynamic_cast<const matrixf*>(m);
    if( smf ) {
		op( smf->compressedMatrix.template cast<U>() );
		return;
	}

	throw std::logic_error("not an eigen matrix");
}

namespace impl {
template<class EigenMatrix>
struct op_prod {
  EigenMatrix& result;
  const EigenMatrix& rhs;

  op_prod(EigenMatrix& result,
		  const EigenMatrix& rhs)
	: result(result),
	  rhs(rhs) {

  }
  
  template<class M>
  void operator()(const M& m) const {
	result += m * rhs;
  }

};


template<class EigenMatrix>
struct op_scal {
  
  EigenMatrix& result;
  typedef typename EigenMatrix::Scalar lambda_type;
  const lambda_type lambda;

  op_scal(EigenMatrix& result,
		  const lambda_type& lambda)
	: result(result),
	  lambda(lambda) {

  }
  
  template<class M>
  void operator()(const M& m) const {
	result += lambda * m;
  }

};


}

template<class EigenMatrix,
		 class BaseMatrix>
void peq_mult(EigenMatrix& result,
			  const BaseMatrix* lhs,
			  const EigenMatrix& rhs) {
	typedef typename EigenMatrix::Scalar real;
	eigen_cast<real>(lhs, impl::op_prod<EigenMatrix>(result, rhs) );
}



template<class EigenMatrix,
		 class BaseMatrix>
void peq_mult(EigenMatrix& result,
			  typename EigenMatrix::Scalar lambda,
			  const BaseMatrix* rhs) {

	typedef typename EigenMatrix::Scalar real;
	eigen_cast<real>(rhs, impl::op_scal<EigenMatrix>(result, lambda) );
}



}


#endif
