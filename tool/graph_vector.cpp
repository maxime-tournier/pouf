#include "graph_vector.h"

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/component/linearsolver/EigenBaseSparseMatrix.h>
#include <sofa/simulation/common/Node.h>

#include <tool/log.h>

namespace tool {

struct info_recorder {
	typedef std::vector< graph_vector::info_type > res_type;

	res_type& res;
	graph_vector::special_type& master;
	graph_vector::special_type& compliant;

	info_recorder( res_type& res,
				   graph_vector::special_type& master,
				   graph_vector::special_type& compliant)
		: res(res),
		  master(master),
		  compliant(compliant) {

		master.dim = 0;
		compliant.dim = 0;
		
	}

	void operator()(unsigned vertex, const mapping_graph& graph) const {

		graph_vector::info_type info;

		info.dim = graph[vertex].mstate->getMatrixSize();
		info.off = res.empty() ? 0 : res.back().off + res.back().dim;

		res.push_back(info);

		// master ?
		mapping_graph::out_edge_range out_edges = boost::out_edges(vertex, graph);
		if( out_edges.first == out_edges.second ) {
			master.vertex.push_back( vertex );
			master.dim += info.dim;
		} else {
			using namespace sofa;
			simulation::Node* node = static_cast<simulation::Node*>(graph[vertex].mstate->getContext());
			
			// compliant ?
			for(unsigned j = 0, m = node->forceField.size(); j < m; ++j ) {
				sofa::core::behavior::BaseForceField* ffield = node->forceField[j];

				if(ffield->isCompliance.getValue() ) {
					compliant.vertex.push_back(vertex);
					compliant.dim += info.dim;
				}

				// TODO assert we only process one compliance at most
			}
		}
		
	}
};

graph_vector::graph_vector(const mapping_graph& graph)
	: master(*this),
	  compliant(*this)
{
	const unsigned n = boost::num_vertices(graph);

	master.vertex.reserve( n );
	graph.top_down( info_recorder( info, master, compliant ) );
	
	dim = info.back().off + info.back().dim;
}

graph_vector::special_type::special_type(const graph_vector& parent) : parent(parent) { }



static void push_impl(graph_vector::chunk out,
					  sofa::defaulttype::BaseMatrix* J,
					  graph_vector::chunk in) {
	
	using namespace sofa::component::linearsolver;
	typedef EigenBaseSparseMatrix<double> matrixd;

    const matrixd* smd = dynamic_cast<const matrixd*> (J);

    if ( smd ) {
		out.noalias() = out + smd->compressedMatrix.cast<SReal>() * in;
		return;
	}

    typedef EigenBaseSparseMatrix<float> matrixf;
    const matrixf* smf = dynamic_cast<const matrixf*>(J);
	
    if( smf ) {
		out.noalias() = out + smf->compressedMatrix.cast<SReal>() * in;
		return;
	}

	throw std::logic_error("only eigen matrices are supported");
}


static void pull_impl(graph_vector::chunk out,
					  sofa::defaulttype::BaseMatrix* J,
					  graph_vector::chunk in) {
	using namespace sofa::component::linearsolver;
	typedef EigenBaseSparseMatrix<double> matrixd;

    const matrixd* smd = dynamic_cast<const matrixd*> (J);

    if ( smd ) {
		out.noalias() = out + smd->compressedMatrix.transpose().cast<SReal>() * in;
		return;
	}

    typedef EigenBaseSparseMatrix<float> matrixf;
    const matrixf* smf = dynamic_cast<const matrixf*>(J);
	
    if( smf ) {
		out.noalias() = out + smf->compressedMatrix.transpose().cast<SReal>() * in;
		return;
	}
	
	throw std::logic_error("only eigen matrices are supported");
	
}


void graph_vector::push(vec& storage, const mapping_graph& graph) const {

	
	for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {

		const chunk src(storage.data() + info[i].off,
					   info[i].dim);
		
		mapping_graph::in_edge_range in_edges = boost::in_edges(i, graph);

		for(mapping_graph::in_edge_iterator e = in_edges.first;
			e != in_edges.second; ++e) {

			const unsigned j = boost::source(*e, graph);
			
			chunk dst(storage.data() + info[j].off,
						   info[j].dim);

			push_impl(dst, graph[*e].j_block(), src);
		}
		
	}	
	
}



void graph_vector::pull(vec& storage, const mapping_graph& graph) const {

	
	for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {

		const unsigned j = n - 1 - i;
		
		const chunk src(storage.data() + info[j].off,
					   info[j].dim);
		
		mapping_graph::out_edge_range out_edges = boost::out_edges(j, graph);
		
		for(mapping_graph::out_edge_iterator e = out_edges.first;
			e != out_edges.second; ++e) {
			
			const unsigned k = boost::target(*e, graph);
			
			chunk dst(storage.data() + info[k].off,
						   info[k].dim);

			pull_impl(dst, graph[*e].j_block(), src);
		}
		
	}	
	
}



void graph_vector::special_type::get(vec& storage, const vec& data) const {
	assert( storage.size() == dim );
	assert( data.size() == parent.dim);
	
	unsigned off = 0;
	for(unsigned i = 0, n = vertex.size(); i < n; ++i) {
		const unsigned v = vertex[i];

		const unsigned d = parent.info[v].dim;
		storage.segment(off, d) = data.segment(parent.info[v].off, d);

		off += d;
	}
}


void graph_vector::special_type::set(vec& storage, const vec& data) const {
	assert( storage.size() == parent.dim );
	assert( data.size() == dim );
	
	unsigned off = 0;
	for(unsigned i = 0, n = vertex.size(); i < n; ++i) {
		const unsigned v = vertex[i];

		const unsigned d = parent.info[v].dim;

		storage.segment(parent.info[v].off, d) = data.segment(off, d);
		
		off += d;
	}
}

}
