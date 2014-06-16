#ifndef ADAPTIVE_MANAGER_H
#define ADAPTIVE_MANAGER_H

#include "../init.h"
#include <utils/graph.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/behavior/BaseMechanicalState.h>

#include <sofa/helper/system/config.h>

namespace adaptive {

  class SOFA_pouf_API manager_base : public sofa::core::objectmodel::BaseObject {

  protected:
	typedef utils::graph<unsigned, unsigned, boost::bidirectionalS> graph_type;

	graph_type graph;

	typedef std::vector<unsigned> front_type;
	front_type front, backup;
	
	typedef sofa::core::behavior::BaseMechanicalState dofs_type;
	dofs_type::SPtr dofs;

  public:

	SOFA_CLASS(manager_base, sofa::core::objectmodel::BaseObject);

	manager_base();

	typedef SReal real_type;
	typedef sofa::defaulttype::Vec<2, real_type> pair_type;
	typedef sofa::vector< pair_type > edges_type;
	sofa::Data<edges_type> edges;

	void init();
	
	void save();
	void restore();

	typedef enum {
	  COARSEN,
	  REFINE
	} operation_type;
	
	virtual void change(operation_type);
	
  protected:
	// callbacks
	virtual void post_init() = 0;
	virtual void update() = 0;

	front_type candidates(operation_type) const;
	
  };


  template<class DataType>
  class SOFA_pouf_API manager;

  
  template<>
  class SOFA_pouf_API manager<sofa::defaulttype::Rigid3dTypes>: public manager_base {
  public:
	SOFA_CLASS( manager, manager_base);
  protected:
	
	void post_init();
	void update();
  };



}



#endif
