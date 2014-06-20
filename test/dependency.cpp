
#include <adaptive/dependency.h>

#include <boost/graph/graphviz.hpp>


extern "C" {
  
  void test_dependency(const char* filename  ) {
	typedef utils::graph<unsigned, double, boost::bidirectionalS> graph_type;
  
	graph_type g(7);

	using namespace boost;
	add_edge(0, 3, g.ep(0.5), g );
	add_edge(1, 4, g);
	add_edge(2, 5, g);
  
	add_edge(4, 3, g);
	add_edge(4, 5, g);

	add_edge(3, 6, g);
	add_edge(5, 6, g);

	std::vector<unsigned> front;

	front.push_back(3);
	front.push_back(4);
    front.push_back(2);

	graph_type deps = adaptive::contract(g, front);

	std::ofstream out(filename);
	write_graphviz(out, deps);
  
  }
  
}
