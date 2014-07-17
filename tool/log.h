#ifndef POUF_TOOL_LOG_H
#define POUF_TOOL_LOG_H

#include <iostream>

namespace tool {

  template<class H>
  void log(const H& h) {
	std::cout << h << std::endl;
  }

  template<class H, class ... T>
  void log(const H& h, const T&...t) {
	std::cout << h << '\t';
	log(t...);
  }
  
}


#endif
