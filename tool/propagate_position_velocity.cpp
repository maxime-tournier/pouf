#include "action.h"

#include "register_object.h"
#include <sofa/simulation/common/MechanicalVisitor.h>

struct propagate {

  static void exec(sofa::simulation::Node* node) {

	sofa::core::MechanicalParams mparams;
	sofa::simulation::MechanicalPropagatePositionAndVelocityVisitor vis(&mparams);
	node->executeVisitor( &vis );

  }

};

static int _ = tool::register_object< action<propagate> >();

template class SOFA_pouf_API action<propagate>;

