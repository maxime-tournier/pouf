#include "assemble.h"

#include "mapping_graph.h"
#include "graph_vector.h"

#include <Eigen/Sparse>

#include <sofa/simulation/common/Node.h>
#include <sofa/component/linearsolver/SingleMatrixAccessor.h>

#include <assembly/AssemblyHelper.h>
#include <utils/scoped.h>

#include "sparse.h"

namespace tool {

typedef sofa::component::linearsolver::AssembledSystem::real real;

typedef Eigen::SparseMatrix<real, Eigen::RowMajor> rmat;
typedef Eigen::SparseMatrix<real, Eigen::ColMajor> cmat;


struct result_type {

	std::vector<rmat> J, H;	// all dofs
	std::vector<rmat> P;	// master dofs
	std::vector<rmat> C;	// compliant dofs

	void reserve(unsigned n) {
		J.reserve(n);
		H.reserve(n);

		// TODO tighter for these
		C.reserve(n);
		P.reserve(n);

	}

};


// gather H/C and concatenate J
struct assembly_visitor {
	
  result_type& result;
  const graph_vector& state;


	// mutable because of sofa not having heard of const-correctness
	mutable sofa::core::MechanicalParams mparams_compliance, mparams_stiffness;

	mutable unsigned off_master;
	
	assembly_visitor(result_type& result,
					 const graph_vector& state,
					 const sofa::core::MechanicalParams mparams) 
		: result(result),
		  state(state),
		  mparams_compliance( mparams ),
		  mparams_stiffness( mparams ) {
		
		off_master = 0;
		
		// mparams crap
		mparams_compliance.setKFactor( 0 );
	}



	void push_C(unsigned vertex,
				sofa::simulation::Node* node, 
				const mapping_graph& graph) const {
		using namespace sofa;

		const unsigned old = result.C.size();
		for(unsigned j = 0, m = node->forceField.size(); j < m; ++j ) {
			core::behavior::BaseForceField* ffield = node->forceField[j];

			// TODO optimize conversion for symmetric matrices ?
			if( ffield->isCompliance.getValue() ) {
			  // TODO it's not cool to store C as a triangularView
			  // cause it makes gauss-seidel more difficult
			  result.C.push_back(convert<rmat>(ffield->getComplianceMatrix(&mparams_compliance)));
			}
		}

		if( result.C.size() - old > 1 ) {
			throw std::runtime_error("more than one compliance under a node");
		}
	}


	
	void push_P(unsigned vertex,
				sofa::simulation::Node* node, 
				const mapping_graph& graph) const {
		using namespace sofa;
		
		mapping_graph::out_edge_range out_edges = boost::out_edges(vertex, graph);
		
		// only for independent dofs
		if( out_edges.first != out_edges.second ) return;
		
		const unsigned dim = graph[vertex].mstate->getMatrixSize();

		component::linearsolver::EigenBaseSparseMatrix<real> tmp;
		tmp.compressedMatrix = sofa::shift_right<rmat>(0, dim, dim);

		for(unsigned i = 0, n = node->projectiveConstraintSet.size(); i < n; ++i){
			node->projectiveConstraintSet[i]->projectMatrix(&tmp, 0);
		}
		
		result.P.push_back( tmp.compressedMatrix );
	}


	void push_H(unsigned vertex,
				sofa::simulation::Node* node, 
				const mapping_graph& graph) const {
		
		const unsigned dim = graph[vertex].mstate->getMatrixSize();
		using namespace sofa;
		component::linearsolver::EigenBaseSparseMatrix<real> sqmat( dim, dim );
		component::linearsolver::SingleMatrixAccessor accessor( &sqmat );

		// TODO deal with interaction forcefields ?
		
		// note that mass are included in forcefield
		for(unsigned j = 0, m = node->forceField.size(); j < m; ++j ) {
			core::behavior::BaseForceField* ffield = node->forceField[j];
			
			component::linearsolver::SingleMatrixAccessor accessor( &sqmat );

			// for compliance you need to add M, B but not K

			// TODO fill only upper part !
			ffield->addMBKToMatrix( ffield->isCompliance.getValue() ?
									&mparams_compliance : &mparams_stiffness,
									&accessor );
		}
		
		sqmat.compress();
		result.H.push_back( sqmat.compressedMatrix.triangularView<Eigen::Upper>() );
	}

	

	// push concatenated mapping and geometric stiffness onto
	// result. H must be up-to-date !
	void push_J(unsigned vertex,
				sofa::simulation::Node* node,
				const mapping_graph& graph) const {
		unsigned dim = graph[vertex].mstate->getMatrixSize();
		
		using namespace sofa;
		
		// mapping block
		result.J.push_back( rmat(dim, state.master.dim) );
		rmat& J = result.J.back();

		mapping_graph::out_edge_range out_edges = boost::out_edges(vertex, graph);

		{
		  // scoped::timer step("concat");

		  // independent dofs
		  if( out_edges.first == out_edges.second ) {
			J = shift_right<rmat>(off_master, dim, state.master.dim);
			off_master += dim;
		  } else {

			for(mapping_graph::out_edge_iterator e = out_edges.first;
				e != out_edges.second; ++e) {
				
			  // mapping concatenation
			  {
				const rmat& parent = result.J[ boost::target(*e, graph) ];
					
				// J += block * parent
				peq_mult(J, graph[*e].j_block(), parent);
			  }

			  // geometric stiffness
			  if( graph[*e].ks ) {
				rmat& parent = result.H[ boost::target(*e, graph) ];
					
				// TODO optimize
				peq_mult(parent, mparams_stiffness.kFactor(), graph[*e].k_block() );
			  }
			}

		  }
		}
		
	}


	
	void operator()(unsigned vertex, const mapping_graph& graph) const {
		using namespace sofa;

		core::behavior::BaseMechanicalState* dofs = graph[vertex].mstate;
		simulation::Node* node = static_cast<simulation::Node*>(dofs->getContext());

		// TODO these 3 can be parallel
		push_H(vertex, node, graph);
		push_P(vertex, node, graph);
		push_C(vertex, node, graph);
		
		push_J(vertex, node, graph);
		
	}

};




#include <iostream>

struct debug {
  const result_type& res;
  const graph_vector& state;
  
  debug(const result_type& res,
		const graph_vector& state) : res(res), state(state) {
	
		std::cout << "master: ";
		for(unsigned i = 0, n = state.master.vertex.size(); i < n; ++i) {
			std::cout << state.master.vertex[i] << '\t';
		}
		std::cout << std::endl;


		std::cout << "compliant: ";
		for(unsigned i = 0, n = state.compliant.vertex.size(); i < n; ++i) {
			std::cout << state.compliant.vertex[i] << '\t';
		}
		std::cout << std::endl;
		
	}
	
	void operator()(unsigned i, const mapping_graph& graph) const {

		std::cout << i << ": " << graph[i].mstate->getContext()->getName()
				  << "/" << graph[i].mstate->getName() << std::endl;
		
	}
};



sofa::component::linearsolver::AssembledSystem assemble(const mapping_graph& graph,
														const graph_vector& state,
														const sofa::core::MechanicalParams& params) {
  scoped::timer step("assemble");
  
  // fetch data/concatenate mappings
	result_type result;
	result.reserve( boost::num_vertices(graph) );

	{
	  scoped::timer step("fetch/concat");
	  graph.top_down( assembly_visitor(result, state, params) );
	}
	
	using namespace sofa::component::linearsolver;
	AssembledSystem sys(state.master.dim,
						state.compliant.dim);
	
	sys.dt = params.dt();
	
	// everyone
	// TODO parallel ?
	// TODO use triplets instead ?
	{
	  scoped::timer step("response pull");
	  
	  typedef Eigen::Triplet<AssembledSystem::real> T;
	  std::vector<T> triplets;

	  AssembledSystem::cmat tmp;
	  
	  for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {

        // response block
		if( result.H[i].nonZeros() ) {
		  tmp = (result.J[i].transpose() * result.H[i].selfadjointView<Eigen::Upper>() * result.J[i])
			.triangularView<Eigen::Upper>();
		  
		  for (int k = 0; k < tmp.outerSize(); ++k) {
			for (AssembledSystem::cmat::InnerIterator it(tmp, k); it; ++it) {
			  triplets.push_back(T(it.row(), it.col(), it.value()));
			}
		  }
		  
		  // sys.H += (result.J[i].transpose() * result.H[i] * result.J[i])
		  // 	.selfadjointView<Eigen::Upper>();
		}
		
	  }
	  
	  sys.H.setFromTriplets(triplets.begin(), triplets.end());
	}
	
	// master dofs
	sys.master.reserve( state.master.vertex.size() ); 
	for(unsigned i = 0, n = state.master.vertex.size(), offset = 0; i < n; ++i) {
		const unsigned vertex = state.master.vertex[i];

		sys.master.push_back( graph[ vertex ].mstate );
	
		const unsigned dim = graph[ vertex ].mstate->getMatrixSize();
		
		// projection block
		sys.P.middleRows(offset, dim) = sofa::shifted_matrix(result.P[i], offset, dim);
		offset += dim;
	}
	
	// compliant dofs
	sys.compliant.reserve( state.compliant.vertex.size() );
	for(unsigned i = 0, n = state.compliant.vertex.size(), offset = 0; i < n; ++i) {
		const unsigned vertex = state.compliant.vertex[i];
		
		sys.compliant.push_back( graph[ vertex ].mstate );
		
		const unsigned dim = graph[ vertex ].mstate->getMatrixSize();

		// mapping block
		sys.J.middleRows(offset, dim) = result.J[ vertex ];

		// compliance block
		sys.C.middleRows(offset, dim) = sofa::shifted_matrix(result.C[i],
															 offset,
															 dim,
															 -1.0 / params.kFactor() );
		offset += dim;
	}
	
	
	return sys;
}

}
