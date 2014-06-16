#include "manager.h"


namespace adaptive {

  using namespace sofa;

  void manager_base::init() {

	// find dofs
	dofs = this->getContext()->get< dofs_type >( core::objectmodel::BaseContext::Local );

	// create graph
	unsigned n = dofs->getSize();
	graph = graph_type(n);

	// edges creation
	for(unsigned i = 0, end = edges.getValue().size(); i < end; ++i) {
	  const pair_type& e = edges.getValue()[i];
	  
	  boost::add_edge(e[0], e[1], graph);
	}

	// find roots
	front.clear();
	
	for(unsigned i = 0; i < n; ++i) {

	  graph_type::in_edge_range in = boost::in_edges(i, graph);

	  if( in.first == in.second ) {
		front.push_back(i);
	  }
	}

	update();
  }


  void manager_base::save() {
	backup = front;

  }


  void manager_base::restore() {
	front = backup;
  }

  
  manager_base::front_type manager_base::candidates(operation_type op) const {

	front_type res;

	for(unsigned i = 0, n = front.size(); i < n; ++i) {
	  const unsigned v = front[i];
	  switch(op) {
	  case COARSEN: {
		for(graph_type::in_edge_range in = boost::in_edges(v, graph);
			in.first != in.second; ++in.first) {
		  res.push_back(boost::source(*in.first, graph));
		}

	  } break;
	  case REFINE: {
		for(graph_type::out_edge_range out = boost::out_edges(v, graph);
			out.first != out.second; ++out.first) {
		  res.push_back(boost::target(*out.first, graph));
		}
	  }
		
	  }
	  
	}

	return res;
  }
  

  void manager_base::change(operation_type op) {
	front = candidates(op);
	update();
  }



}
