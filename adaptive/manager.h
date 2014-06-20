#ifndef ADAPTIVE_MANAGER_H
#define ADAPTIVE_MANAGER_H

#include "../init.h"
#include <utils/graph.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/BaseMechanicalState.h>

#include <sofa/helper/system/config.h>

namespace adaptive {

  class SOFA_pouf_API manager_base : public sofa::core::objectmodel::BaseObject {

  protected:

	typedef utils::graph<unsigned, double, boost::bidirectionalS> graph_type;
	graph_type graph;

	typedef std::vector<unsigned> front_type;
	front_type front, backup;

	// master dofs
	typedef sofa::core::behavior::BaseMechanicalState dofs_type;
	dofs_type::SPtr master, all;

	struct dependency_visitor;
	graph_type dependency(const front_type& f) const;
	
  public:

	SOFA_CLASS(manager_base, sofa::core::objectmodel::BaseObject);

	manager_base();

	typedef SReal real_type;
	typedef sofa::defaulttype::Vec<2, real_type> pair_type;

	typedef sofa::vector< pair_type > edges_type;
	typedef sofa::vector< real_type > weights_type;

	// full dependency graph
	sofa::Data<edges_type> edges;
	sofa::Data<weights_type> weights;

	void init();

	typedef enum {
	  COARSEN,
	  REFINE
	} operation_type;

	//  list of dof candidate for coarsening or refining
	front_type candidates(operation_type) const;

	// change front
	void set_front(const front_type& );
	const front_type& get_front() const;

	// pointer to all dofs
	dofs_type::SPtr dofs() const;

  protected:
	// callbacks
	virtual void post_init() = 0;

	virtual void update() = 0;
	
  };


  template<class DataType>
  class SOFA_pouf_API manager;

  template<>
  class SOFA_pouf_API manager<sofa::defaulttype::Vec3dTypes>: public manager_base {
  public:
	SOFA_CLASS( manager, manager_base);
  protected:
	
	void post_init();
	void update();
  };



}



#endif
