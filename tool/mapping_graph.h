#ifndef MAPPING_GRAPH_H
#define MAPPING_GRAPH_H

#include <utils/graph.h>
#include <vector>

namespace sofa {
namespace core {
class BaseMapping;

namespace objectmodel{
class BaseContext;
}

namespace behavior {
class BaseMechanicalState;
}
}
namespace defaulttype {
class BaseMatrix;
}
}


/** 
	mapping *dependency* graph: vertices represent mstates, and edge
	(x -> y) exists iff y is mapped from x (thus y depends from x).

	vertices contain mstate pointers, edges contains an index for the
	corresponding mapping block (source dof context mechanical mapping)

	an important property is that kinemactic_graphs are naturally
	ordered according to a top_down traversal (parents first).

	@author Maxime Tournier
*/

namespace tool {

class mapping_graph : public utils::graph<sofa::core::behavior::BaseMechanicalState*,
											unsigned,
											boost::bidirectionalS> {
public:
	
	typedef utils::graph< vertex_type, edge_type, direction_type> base;

	void set(const std::vector<vertex_type>& dofs);
	void set(sofa::core::objectmodel::BaseContext* context);

	template<class Visitor>
	void top_down(const Visitor& vis) const {
		for(unsigned i = 0, n = boost::num_vertices(*this); i < n; ++i) {
			vis(i, *this);
		}
	}

	template<class Visitor>
	void bottom_up(const Visitor& vis) const {
		for(unsigned i = 0, n = boost::num_vertices(*this); i < n; ++i) {
			const unsigned j = n - 1 - i;
			vis(j, *this);
		}
	}

	struct recorder {
		
		std::vector<unsigned>& res;
		recorder(std::vector<unsigned>& res) : res( res ) { }
		
		template<class G>
		void operator()(unsigned i, const G& ) const {
			res.push_back(i);
		}
		
	};


};

}


#endif
