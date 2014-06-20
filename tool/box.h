#ifndef BOX_H
#define BOX_H

namespace tool {

  // boxed types
  template<class T>
  class box {
	T data;
  public:
	
	box(const T& data) : data(data) { }
	box() { }

	box& operator=(const T& x) {
	  data = x;
	}

	operator const T& () const {
	  return data;
	}

	operator T& () {
	  return data;
	}
	
  };

  // boxed pointer types
  template<class T>
  class box<T*> {
	T* data;
  public:
	box(T* data) : data(data) { }
	box() { }

	box& operator=(const T& x) {
	  data = x;
	}

	operator const T& () const {
	  return data;
	}
	
	operator T& () {
	  return data;
	}

	T* operator->() const {
	  return data;
	}

	T& operator*() const {
	  return *data;
	}
	
  };


  
}

#endif
