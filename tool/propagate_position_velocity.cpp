#include <sofa/simulation/common/MechanicalVisitor.h>

extern "C" {

  void propagate_position_velocity(sofa::simulation::Node* node) {
	sofa::core::MechanicalParams mparams;
	sofa::simulation::MechanicalPropagatePositionAndVelocityVisitor vis(&mparams);
	node->executeVisitor( &vis );
  }

}
