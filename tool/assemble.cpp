#include "assemble.h"

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

	// TODO could be merged with graph_vector
	struct chunk_type {
		std::vector<unsigned> vertex, offset;
		unsigned dim;
	} master, compliant;

	std::vector<rmat> J, H;	// all dofs
	std::vector<rmat> P;	// master dofs
	std::vector<rmat> C;	// compliant dofs

	void reserve(unsigned n) {
		J.reserve(n);
		H.reserve(n);

		// TODO tighter for these
		C.reserve(n);
		P.reserve(n);

		master.vertex.reserve(n);
		master.offset.reserve(n);

		compliant.vertex.reserve(n);
		compliant.offset.reserve(n);
	}

};


struct offset_visitor {
	result_type& result;

	offset_visitor(result_type& result)
		: result(result) {
		
		result.master.dim = 0;
		result.compliant.dim = 0;
	}



	void push_offsets(result_type::chunk_type& out,
					  unsigned vertex,
					  const mapping_graph& graph)  const {

		if( out.offset.empty() ) {
			// first dofs 
			out.offset.push_back( 0 );
		} else {
			// previous dofs
			unsigned prev_dim = graph[ out.vertex.back() ]->getMatrixSize();
			out.offset.push_back(out.offset.back() + prev_dim);
		}

		out.vertex.push_back( vertex );
		out.dim += graph[vertex]->getMatrixSize();
		
	}
	
	// push master/compliant offsets for current vertex
	void push_offsets(unsigned vertex,
					  sofa::simulation::Node* node,
					  const mapping_graph& graph) const {

		mapping_graph::out_edge_range out_edges = boost::out_edges(vertex, graph);

		// independent dofs ?
		if( out_edges.first == out_edges.second ) {
			push_offsets(result.master, vertex, graph);
		} else {
			// mapped dofs
			
			for(unsigned j = 0, m = node->forceField.size(); j < m; ++j ) {
				sofa::core::behavior::BaseForceField* ffield = node->forceField[j];
				// compliant dofs ?
				if(ffield->isCompliance.getValue() ) {
					push_offsets(result.compliant, vertex, graph);
				}
			}
		}
	}



	void operator()(unsigned vertex, const mapping_graph& graph) const {
		using namespace sofa;
		
		core::behavior::BaseMechanicalState* dofs = graph[vertex];
		simulation::Node* node = static_cast<simulation::Node*>(dofs->getContext());
		
		push_offsets(vertex, node, graph);
	}
	
};


// gather H/C and concatenate J
struct assembly_visitor {
	
	result_type& result;

	// mutable because of sofa not having heard of const-correctness
	mutable sofa::core::MechanicalParams mparams_compliance, mparams_stiffness;

	mutable unsigned off_master;
	
	assembly_visitor(result_type& result,
					 const sofa::core::MechanicalParams mparams) 
		: result(result),
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

			if( ffield->isCompliance.getValue() ) {
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
		
		const unsigned dim = graph[vertex]->getMatrixSize();

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
		
		const unsigned dim = graph[vertex]->getMatrixSize();
		using namespace sofa;
		component::linearsolver::EigenBaseSparseMatrix<real> sqmat( dim, dim );
		component::linearsolver::SingleMatrixAccessor accessor( &sqmat );

		// TODO deal with interaction forcefields ?
		
		// note that mass are included in forcefield
		for(unsigned j = 0, m = node->forceField.size(); j < m; ++j ) {
			core::behavior::BaseForceField* ffield = node->forceField[j];
			
			component::linearsolver::SingleMatrixAccessor accessor( &sqmat );

			// for compliance you need to add M, B but not K
			ffield->addMBKToMatrix( ffield->isCompliance.getValue() ?
									&mparams_compliance : &mparams_stiffness,
									&accessor );
		}
		
		sqmat.compress();
		result.H.push_back( sqmat.compressedMatrix.selfadjointView<Eigen::Upper>() );
	}

	

	// push concatenated mapping and geometric stiffness onto
	// result. offsets/H must be up-to-date !
	void push_J(unsigned vertex,
				sofa::simulation::Node* node,
				const mapping_graph& graph) const {
		unsigned dim = graph[vertex]->getMatrixSize();
		
		using namespace sofa;
		
		// mapping block
		result.J.push_back( rmat(dim, result.master.dim) );
		rmat& J = result.J.back();

		mapping_graph::out_edge_range out_edges = boost::out_edges(vertex, graph);

		// independent dofs
		if( out_edges.first == out_edges.second ) {
			J = shift_right<rmat>(off_master, dim, result.master.dim);
			off_master += dim;
		} else {

			// mapping concatenation
			const std::vector<defaulttype::BaseMatrix*>* js =
				node->mechanicalMapping->getJs();
			
			for(mapping_graph::out_edge_iterator e = out_edges.first;
				e != out_edges.second; ++e) {

				const rmat& parent = result.J[ boost::target(*e, graph) ];

				// J += block * parent
				peq_mult(J, (*js)[ graph[*e] ], parent);
			}


			// geometric stiffness
			const std::vector<defaulttype::BaseMatrix*>* ks =
				node->mechanicalMapping->getKs();

			if( !ks ) return;
			
			for(mapping_graph::out_edge_iterator e = out_edges.first;
				e != out_edges.second; ++e) {
				
				rmat& parent = result.H[ boost::target(*e, graph) ];
				
				// TODO optimize
				peq_mult(parent, mparams_stiffness.kFactor(), (*ks)[ graph[*e] ] );
			}
			

		}
		
	}
	
	void operator()(unsigned vertex, const mapping_graph& graph) const {
		using namespace sofa;

		core::behavior::BaseMechanicalState* dofs = graph[vertex];
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

	debug(const result_type& res) : res(res) {

		std::cout << "master: ";
		for(unsigned i = 0, n = res.master.vertex.size(); i < n; ++i) {
			std::cout << res.master.vertex[i] << '\t';
		}
		std::cout << std::endl;


		std::cout << "compliant: ";
		for(unsigned i = 0, n = res.compliant.vertex.size(); i < n; ++i) {
			std::cout << res.compliant.vertex[i] << '\t';
		}
		std::cout << std::endl;
		
	}
	
	void operator()(unsigned i, const mapping_graph& graph) const {

		std::cout << i << ": " << graph[i]->getContext()->getName()
				  << "/" << graph[i]->getName() << std::endl;
		
	}
};



sofa::component::linearsolver::AssembledSystem assemble(const mapping_graph& graph,
														const sofa::core::MechanicalParams& params) {
	// fetch data/concatenate mappings
	result_type result;
	result.reserve( boost::num_vertices(graph) );
	
	graph.top_down( offset_visitor(result) );
	graph.top_down( assembly_visitor(result, params) );
	
	using namespace sofa;
	component::linearsolver::AssembledSystem sys(result.master.dim,
												 result.compliant.dim);

	sys.dt = params.dt();
	
	// TODO parallel

	// everyone
	for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {

        // response block
		if( result.H[i].nonZeros() ) {
			sys.H += (result.J[i].transpose() * result.H[i] * result.J[i]).selfadjointView<Eigen::Upper>();
		}
		
	}

	// master dofs
	sys.master.reserve( result.master.vertex.size() );
	for(unsigned i = 0, n = result.master.vertex.size(); i < n; ++i) {
		const unsigned vertex = result.master.vertex[i];
		const unsigned offset = result.master.offset[i];

		sys.master.push_back( graph[ vertex ] );
	
		const unsigned dim = graph[ vertex ]->getMatrixSize();
		
		// projection block
		sys.P.middleRows(offset, dim) = shifted_matrix(result.P[i], offset, dim);
	}
	
	// compliant dofs
	sys.compliant.reserve( result.compliant.vertex.size() );
	for(unsigned i = 0, n = result.compliant.vertex.size(); i < n; ++i) {
		const unsigned vertex = result.compliant.vertex[i];
		const unsigned offset = result.compliant.offset[i];
		
		sys.compliant.push_back( graph[ vertex ] );
		
		const unsigned dim = graph[ vertex ]->getMatrixSize();

		// mapping block
		sys.J.middleRows(offset, dim) = result.J[ vertex ];

		// compliance block
		sys.C.middleRows(offset, dim) = shifted_matrix(result.C[i],
													   offset,
													   dim,
													   -1.0 / params.kFactor() );
	}
	
	
	return sys;
}

}
