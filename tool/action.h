#ifndef POUF_ACTION_H
#define POUF_ACTION_H

#include "../init.h"

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/simulation/common/Node.h>

// a single action to perform on the graph
template<class Impl>
class SOFA_pouf_API action : public sofa::core::objectmodel::BaseObject {
public:

  SOFA_CLASS(SOFA_TEMPLATE( action, Impl ), sofa::core::objectmodel::BaseObject);

  typedef sofa::simulation::Node node_type;

  void parse( sofa::core::objectmodel::BaseObjectDescription* arg) {
	BaseObject::parse( arg );
	Impl::exec( dynamic_cast<node_type*>( this->getContext()) );

	this->getContext()->removeObject( this );

	// TODO not sure if safe, but it's never destroyed 
	delete this;
  }

  
};



struct hello {

  static void exec(sofa::simulation::Node* node) {
	std::cout << "hello !" << std::endl;
  }

};



#endif
