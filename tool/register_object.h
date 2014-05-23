#ifndef REGISTER_OBJECT_H
#define REGISTER_OBJECT_H

#include <sofa/core/objectmodel/BaseClass.h>
#include <sofa/core/ObjectFactory.h>

namespace tool {
  namespace impl {


	template<class T>
	static std::string class_name() {
	  return sofa::core::objectmodel::BaseClass::decodeClassName( typeid(T) );
	}

  
	template<class T>
	struct parse {

	  static std::string template_name() {
		return "";
	  }
	  
	};


	template< template<class> class T, class A>
	struct parse< T<A> > {

	  static std::string template_name() {
		return class_name<A>();
	  }
	
	};

	template< template<class, class> class T, class A, class B>
	struct parse< T<A, B> > {

	  static std::string template_name() {
		return class_name<A>() + ',' + class_name<B>();
	  }
	
	};

	// etc...

	
  }


  template<class T>
  static int register_object(const std::string& desc = "") {

	using namespace sofa::core;

	int foo = RegisterObject(desc)
	  .addCreator(impl::class_name<T>(),
				  impl::parse<T>::template_name(),
				  ObjectFactory::Creator::SPtr(new ObjectCreator< T >()) );
	return 0;
  }

}


#endif
