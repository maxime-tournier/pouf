#include "mapping_graph.h"

#include "find_insert.h"

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/BaseMapping.h>

#include <sofa/simulation/common/Node.h>
#include <sofa/simulation/common/MechanicalVisitor.h>

#include <stdexcept>
#include <utils/scoped.h>


namespace tool {

namespace impl {
struct printer {
	
	printer() : i(0) { }
	mutable unsigned i;
	
	void operator()(unsigned vertex, const mapping_graph& g) const {
		assert( vertex == i );
		++i;
	}
	
};

}


void mapping_graph::set(const std::vector<impl::vertex::mstate_type>& data) {
  scoped::timer step("building graph");
  
	// reset
	base tmp(data.size());
	
	// index map
	std::map<mstate_type, unsigned> index;

	// mechanical flags
	std::vector<bool> mechanical(data.size(), false);

	// build initial graph
	// complexity: n * ( log(n) + m * log(n) ) where n == data.size()
	// and m is max connectivity (in degree)
	for(unsigned i = 0, n = data.size(); i < n; ++i) {

	    // log(n)
		const unsigned vertex = find_insert(index, data[i], index.size());
		tmp[vertex].mstate = data[i];
		
		sofa::simulation::Node* node = static_cast<sofa::simulation::Node*>(data[i]->getContext());
		assert( dynamic_cast<sofa::simulation::Node*>( data[i]->getContext() ) );

		// mechanical: has mass/stiffness/compliance
		mechanical[i] = node->forceField.size() || node->interactionForceField.size();
		
		// graph edges
		if(!node->mechanicalMapping) continue;

		if( node->mechanicalMapping->getTo().size() != 1 ) {
			throw std::runtime_error("only n -> 1 mappings are supported");
		}
		
		const std::vector<sofa::core::BaseState*> from = node->mechanicalMapping->getFrom();

		const impl::edge::matrix_type* js = node->mechanicalMapping->getJs();
		impl::edge::const_block_type k = node->mechanicalMapping->getK();
        assert( (from.size() == 1) or not k );

		for( unsigned j = 0, m = from.size(); j < m; ++j ) {

			// parent dofs
			const mstate_type p = dynamic_cast<mstate_type>(from[j]);

			if( !p ) throw std::logic_error("non mechanical parent :-/");

			// log(n)
			const unsigned parent_vertex = find_insert(index, p, index.size());
			tmp[parent_vertex].mstate = p;

			// graph edge from child to parent
			const edge_descriptor e = boost::add_edge(vertex, parent_vertex, tmp).first;

			tmp[e].js = js;
			tmp[e].k = k;
			tmp[e].index = j;
		}

	}

	
	// record traversal order
	std::vector<unsigned> top_down;
	top_down.reserve(data.size());
	
	utils::dfs(tmp, recorder(top_down) );

	// mechanical flags propagation: bottom-up
	unsigned n_mechanical = 0;
	for(unsigned i = 0, n = top_down.size(); i < n; ++i) {
		const unsigned j = n - 1 - i;
		const unsigned vertex = top_down[j];
		
		if( !mechanical[ vertex ] ) continue;
		++n_mechanical;

		const out_edge_range out_edges = boost::out_edges(vertex, tmp);

		// propagation
		for(out_edge_iterator e = out_edges.first; e != out_edges.second; ++e) {
			const unsigned dst = boost::target(*e, tmp);
			mechanical[ dst ] = true;
		}
	}

	// now we know who is mechanical
	// for(unsigned i = 0, n = mechanical.size(); i < n; ++i) {
	// 	std::cout << i << '\t' << tmp[i]->getContext()->getName() << '/' << tmp[i]->getName()
	// 			  << '\t' << mechanical[i] << std::endl;
	// }

	// final graph
	base::operator=( base(n_mechanical) );

	// unpruned graph index -> pruned graph index, -1 if none
	std::vector<unsigned> index_map(data.size(), -1);

	// pruning: associate only mechanical vertices. note that we
	// iterate in top-down order so that the final graph is naturally
	// ordered like this
	unsigned off = 0;
	
	for(unsigned i = 0, n = top_down.size(); i < n; ++i) {
		const unsigned vertex = top_down[i];
		if( mechanical[vertex] ) {

			(*this)[off] = tmp[vertex];
			index_map[vertex] = off;
			++off;
		}
	}
	assert( off == n_mechanical );

	// adding edges to pruned graph
	for(unsigned i = 0, n = top_down.size(); i < n; ++i) {
	  const unsigned vertex = top_down[i];
	  const unsigned src = index_map[vertex];
	  
	  if( src != -1 ) {
		const out_edge_range out_edges = boost::out_edges(vertex, tmp);
		
		for(out_edge_iterator e = out_edges.first; e != out_edges.second; ++e) {
		  const unsigned dst = index_map[ boost::target(*e, tmp) ];
		  assert( dst != -1 );
		  
		  const edge_descriptor f = boost::add_edge(src, dst, *this).first;
		  (*this)[f] = tmp[*e];
		}

	  }
	}
	
	// check graph is really ordered top-down
	assert( (utils::dfs(*this, impl::printer()), true ) );
	
}

// simply record mstates as the graph is visited
struct dofs_recorder : sofa::simulation::MechanicalVisitor {

	std::vector<mapping_graph::mstate_type>& out;

	sofa::core::MechanicalParams mparams;
	dofs_recorder(std::vector<mapping_graph::mstate_type>& out)
		: sofa::simulation::MechanicalVisitor(&mparams),
		  out(out) { }
	
	Result processNodeTopDown(sofa::simulation::Node* node) {

		if( node->mechanicalState ) {
		  out.push_back( node->mechanicalState );
		}
		
		return RESULT_CONTINUE;
	}


};


void mapping_graph::set(sofa::core::objectmodel::BaseContext* context) {
  scoped::timer step("mapping graph");

	std::vector<mstate_type> dofs;
	dofs_recorder vis(dofs);

	{
	  scoped::timer step("visitor");
	  context->executeVisitor( &vis );
	}

	set( dofs );
}


  namespace impl {
	edge::edge() : js(0), k(0), index(-1) { }

	edge::block_type edge::j_block() const {
	  return (*js)[index];
	}

  };
}
