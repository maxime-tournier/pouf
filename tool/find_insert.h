#ifndef POUF_FIND_INSERT_H
#define POUF_FIND_INSERT_H

#include <map>

namespace tool {


template<class Map>
typename Map::mapped_type& find_insert(Map& map,
									  const typename Map::key_type& k,
									  const typename Map::mapped_type& v) {
	
	typename Map::iterator lb = map.lower_bound(k);
	
	if(lb != map.end() && !(map.key_comp()(k, lb->first))) {
		// key already exists
		return lb->second;
	} else {
		// the key does not exist in the map add it to the map.
		// use lb as a hint to insert,so it can avoid another lookup
		typename Map::iterator res = map.insert(lb, typename Map::value_type(k, v));
		return res->second;
	}
}


template<class Set>
typename Set::value_type& find_insert(Set& set,
									  const typename Set::value_type& v) {
  
  typename Set::iterator lb = set.lower_bound(v);
	
  if(lb != set.end() && !(set.comp()(v, lb->first))) {
	return *lb;
  } else {
	typename Set::iterator res = set.insert(lb, v);
	return *res;
  }
  
}

  
}

#endif

