#include "manager.h"
#include "dependency.h"

namespace adaptive {

  using namespace sofa;


  manager_base::manager_base()
	: edges(initData(&edges, "edges", "multi-resolution hierarchy topology as a list of edges")),
	  weights(initData(&weights, "weights", "interpolation weight for each edge")) {

  }
  
  void manager_base::init() {

	if( edges.getValue().size() != weights.getValue().size() ) {
	  throw std::runtime_error("edges must be the same lengths as weights");
	}
	
	// find dofs
	all = this->getContext()->get< dofs_type >( core::objectmodel::BaseContext::Local );

	// create graph
	unsigned n = all->getSize();
	graph = make_graph(n, edges.getValue(), weights.getValue());

	// find roots
	front.clear();
	
	for(unsigned i = 0; i < n; ++i) {

	  graph_type::in_edge_range in = boost::in_edges(i, graph);

	  if( in.first == in.second ) {
		front.push_back(i);
	  }
	}

	post_init();
	update();
  }


  void manager_base::set_front( const front_type& f ) {
	front = f;
	update();
  }

  const manager_base::front_type& manager_base::get_front() const {
	return front;
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


  manager_base::dofs_type::SPtr manager_base::dofs() const { return all; }









  // test yo
  void manager<sofa::defaulttype::Vec3dTypes>::post_init() {


  }


  // test yo
  void manager<sofa::defaulttype::Vec3dTypes>::update() {

	graph_type contracted = contract(graph, front);

	// update mapping defs
	
  }


  
}
