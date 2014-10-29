#ifndef POUF_IMPLICIT_EULER_H
#define POUF_IMPLICIT_EULER_H

#include "init.h"

#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/MultiVec.h>
#include <sofa/simulation/common/MechanicalVisitor.h>

// TODO forward instead ?
#include "numericalsolver/KKTSolver.h"

namespace tool {

struct graph_vector;
struct mapping_graph;

}


namespace sofa {

namespace simulation {
struct AssemblyVisitor;

namespace common {
class MechanicalOperations;
class VectorOperations;
}

}

namespace component {
 
namespace linearsolver {
struct AssembledSystem;
}




namespace odesolver {
			


/**
   
   a nice and clean solver. 

   @author maxime.tournier@brain.riken.jp

*/

class SOFA_pouf_API pouf_solver : public sofa::core::behavior::OdeSolver {
  public:
				
	SOFA_CLASS(pouf_solver, sofa::core::behavior::OdeSolver);


    typedef linearsolver::AssembledSystem system_type;
				
    virtual void init();

    // OdeSolver API
    virtual void solve(const core::ExecParams* params,
                       double dt,
                       core::MultiVecCoordId posId,
                       core::MultiVecDerivId velId);

	pouf_solver();
    virtual ~pouf_solver();
	
    // mechanical params
    void buildMparams( core::MechanicalParams& mparams,
                       const core::ExecParams& params,
                       double dt) const;

    Data<bool> warm_start, aggregate_lambdas, stabilization, integration;
	Data<unsigned> debug;
	Data<SReal> stabilization_damping;
	
  protected:
	// send a visitor 
    void send(simulation::Visitor& vis);
			  
	// integrate positions
    void integrate( const core::MechanicalParams* params, 
					core::MultiVecCoordId posId, 
					core::MultiVecDerivId velId );

	// propagate positions/velocities
    void propagate(const core::MechanicalParams* params);

	// aggregate lambdas
    void aggregate(const core::MechanicalParams* params); 
	
	// linear solver: TODO hide in pimpl ?
	typedef linearsolver::KKTSolver kkt_type;
	kkt_type::SPtr kkt;

public:
	typedef system_type::vec vec;


	// linear rhs for dynamics/correction steps
    virtual void rhs_dynamics(vec& res,
							  const system_type& sys,
							  const vec& ck) const;
	
	virtual void rhs_correction(vec& res, const system_type& sys) const;
	
	// current v, lambda
	virtual void get_state(vec& res, const system_type& sys) const;

	virtual void set_vel(const system_type& sys, const vec& data) const;


	void update_scene(const tool::graph_vector& state,
					  const tool::mapping_graph& graph,
					  const vec& x,
					  SReal dt);
	
	// this is for warm start and returning constraint forces
	core::behavior::MultiVecDeriv lagrange;

protected:

	
};

}
}
}



#endif
