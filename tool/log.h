#ifndef POUF_TOOL_LOG_H
#define POUF_TOOL_LOG_H

#include <iostream>

namespace tool {

  template<class H>
  void log(const H& h) {
	std::cout << h << std::endl;
  }

  // stupid VS2012...
#ifdef SOFA_HAVE_VARIADIC_TEMPLATES
  template<class H, class ... T>
  void log(const H& h, const T&...t) {
	std::cout << h << '\t';
	log(t...);
  }
#else
  template<class H, class T>
  void log(const H& h, const T& t) {
	std::cout << h << '\t';
	log(t);
  }
#endif
  
}


#endif
