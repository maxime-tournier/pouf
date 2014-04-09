#ifndef POUF_GRAPH_VECTOR_H
#define POUF_GRAPH_VECTOR_H

#include "mapping_graph.h"

#include <Eigen/Core>

namespace tool {
struct graph_vector {

	graph_vector(const mapping_graph& graph);

	unsigned dim;

	struct info_type {
		unsigned off;
		unsigned dim;
	};
	
	std::vector< info_type > info;

	typedef Eigen::VectorXd vec;
	typedef Eigen::Map< vec > chunk;

	struct special_type {
		special_type(const graph_vector&);
		const graph_vector& parent;
		std::vector< unsigned > vertex;
		unsigned dim;
		
		// get/set data for special dofs in a big storage vector
		void set(vec& full, const vec& sub) const;
		void get(vec& sub, const vec& full) const;
		
	} master, compliant;

	
	// top-down
	void push(vec& storage, const mapping_graph& graph) const;

	// bottom-up
	void pull(vec& storage, const mapping_graph& graph) const;

	
};

}
#endif
