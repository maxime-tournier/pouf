#ifndef ADAPTIVE_DEPENDENCY_H
#define ADAPTIVE_DEPENDENCY_H

#include <utils/graph.h>
#include <deque>

#include <iostream>

namespace adaptive {


  namespace impl {

	struct postfix_recorder {
	  std::vector<unsigned>& res;

	  postfix_recorder(std::vector<unsigned>& res) : res(res) {
		res.clear();
	  }

	  template<class G>
	  void operator()(unsigned v, const G& g) const {
		res.push_back(v);
	  }
	  
	};


	// this must be called top-down (prefix)
	template<class G>
	struct dependency_visitor {
	  G& res;

	  typedef G graph_type;
	  
	  dependency_visitor(graph_type& res) : res(res) { }

	  void operator()(unsigned v, const graph_type& g) const {
		using namespace boost;

		std::cout << "visiting " << v << std::endl;
	  
		// for each parent of v in dependency graph
		for(typename graph_type::in_edge_range in = in_edges(v, g);
			in.first != in.second; ++in.first) {

		  const unsigned s = source(*in.first, g);
		  const double w = g[*in.first];
		
		  // is parent independent ?
		  typename graph_type::in_edge_range s_in = in_edges(s, res);

		  if( s_in.first == s_in.second ) {
			// new direct dependency from s to v

			typename graph_type::edge_descriptor e = add_edge(s, v, res).first;
			res[e] = w;

		  } else {
			// concatenate dependencies
			for(; s_in.first != s_in.second; ++s_in.first) {

			  const unsigned ss = source(*s_in.first, res);
			  const double ws = res[ *s_in.first ];

			  typename graph_type::edge_descriptor e = add_edge(ss, v, res).first;

			  res[e] = w * ws;
			}
		  
		  }
		
		}
	  }
	
	};

  }


  // TODO move to adaptive/graph.h
  typedef utils::graph<unsigned, double, boost::bidirectionalS> graph_type;
  
  template<class Edges, class Weights>
  graph_type make_graph(unsigned n, const Edges& edges, const Weights& weights) {
	graph_type res(n);

	// edges creation
	for(unsigned i = 0, end = edges.size(); i < end; ++i) {
	  const typename Edges::value_type& ee = edges[i];
	  
	  boost::add_edge(ee[0], ee[1], res.ep( weights[i] ), res);
	}

	return res;
  }
  

  template<class G, class V>
  static G contract(const G& graph, const V& front) {
	
	using namespace boost;
	typedef G graph_type;
	const unsigned n = num_vertices(graph);
		
	graph_type tmp = graph;

	// typename property_map<graph_type, vertex_color_t>::type color = 
	auto color = get(vertex_color, tmp);

	// paint it black
	for(unsigned i = 0; i < n; ++i) {
	  color[i] = black_color;
	}
	

	// remove any edge "above" front
	std::deque<unsigned> queue(front.begin(), front.end());

	for(unsigned i = 0, k = front.size(); i < k; ++i) {
	  color[front[i]] = white_color;
	}
	
	// invariant: every edge between f and queue has been removed from
	// tmp
	while(!queue.empty()) {
	  const unsigned v = queue.front();
	  queue.pop_front();

	  std::cout << v << std::endl;
	  
	  for(typename graph_type::in_edge_range in = boost::in_edges(v, graph);
		  in.first != in.second; ++in.first) {
		
		const unsigned s = source(*in.first, graph);
		const unsigned t = target(*in.first, graph);

		std::cout << "(" << s <<  " " <<  t <<  ")" << std::endl;
		
		remove_edge(s, t, tmp);

		if( color[s] == black_color ) {
		  queue.push_back(s);
		  std::cout << "pushing " << s << std::endl;
		}
		
		color[v] = white_color;
	  }


	}

	std::cout << "upper graph pruned" << std::endl;

	// record postfix traversal
	std::vector<unsigned> postfix;
	postfix.reserve(n);
	utils::dfs(tmp, impl::postfix_recorder(postfix));
	
	// now we can process tmp top-down to concatenate dependencies
	graph_type res(n);
	impl::dependency_visitor<G> vis(res);

	for(unsigned i = 0; i < n; ++i) {
	  vis( postfix[n - 1 - i], tmp);
	}
	
	return res;
  }

}


#endif
