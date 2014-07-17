

#include <sofa/component/collision/ComponentMouseInteraction.h>


namespace sofa {
  namespace component {
	namespace collision {


	  static struct disabler {

		disabler() {
		  typedef ComponentMouseInteraction::ComponentMouseInteractionFactory factory_type;
		  factory_type::ResetEntry("MouseSpringRigid3d");
		  factory_type::ResetEntry("MouseSpringRigid3f");
		}


	  } instance;

	}
  }
}
