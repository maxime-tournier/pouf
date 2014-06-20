#ifndef MATH_POUF_ITER_H
#define MATH_POUF_ITER_H

#include <math/types.h>
#include <functional>

namespace math {

  struct iter {
	unsigned max;
	real precision;

	typedef std::function< void (unsigned, const vec& ) > cb_type;
	cb_type cb;
	
	iter() : max(0), precision(0) { }
	
  };



}



#endif
