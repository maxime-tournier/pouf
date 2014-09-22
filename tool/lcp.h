#ifndef POUF_TOOL_LCP_H
#define POUF_TOOL_LCP_H

#include <math/types.h>
#include <fstream>

// TODO proper header/source

namespace tool {

  static void write_vec(std::ofstream& out, const math::vec& v) {
	for(unsigned j = 0, n = v.size(); j < n; ++j) {
	  out << v(j);
	  if( j < n - 1 ) out << "\t";			
	}
	out << std::endl;
  }
  

  template<class Matrix>
  static void write_lcp(const std::string& filename,
						const Matrix& M,
						const math::vec& q) {
	using namespace math;
	
	std::ofstream out(filename);
	unsigned n = q.size();

	out << n << std::endl;
	
	math::vec unit = vec::Zero(n);

	math::vec tmp;
	
	for(unsigned i = 0; i < n; ++i) {
	  unit(i) = 1;

	  write_vec(out, M(unit) );		

	  unit(i) = 0;
	}

	write_vec( out, q );
  }

}  


#endif
