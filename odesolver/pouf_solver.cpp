#include "pouf_solver.h"

#include <tool/here.h>

// this is for propagate_visitor
// #include <odesolver/AssembledSolver.h>

#include <SofaEigen2Solver/EigenSparseMatrix.h>
#include <SofaEigen2Solver/EigenVector.h>
#include <sofa/core/ObjectFactory.h>


#include <sofa/simulation/common/MechanicalOperations.h>
#include <sofa/simulation/common/VectorOperations.h>

#include <assembly/AssemblyVisitor.h>
#include <constraint/ConstraintValue.h>

#include <utils/minres.h>
#include <utils/scoped.h>

#include <utils/nan.h>

#include <tool/mapping_graph.h>
#include <tool/graph_vector.h>
#include <tool/assemble.h>

using std::cerr;
using std::endl;

namespace sofa {
  namespace component {
	namespace odesolver {

	  SOFA_DECL_CLASS(pouf_solver);

	  static const bool nan_hunt = false;

	  template<class X>
	  static inline void nan_check(const X& x, const char* msg) {
		if( nan_hunt and has_nan(x) ) {
		  std::cout << msg << std::endl;
		  throw std::logic_error( msg );
		}
	  }


	  
    static int pouf_solverClass = core::RegisterObject("Example compliance solver using assembly")
		.add< pouf_solver >()
		.addAlias("pouf.solver");

	  using namespace sofa::defaulttype;
	  using namespace sofa::helper;
	  using namespace core::behavior;


	  pouf_solver::pouf_solver()
		: warm_start(initData(&warm_start,
							  false,
							  "warm_start",
							  "warm start iterative solvers: avoids biasing solution towards zero (and speeds-up resolution)")),
		  aggregate_lambdas(initData(&aggregate_lambdas,
									 true,
									 "aggregate_lambdas",
									 "aggregate Lagrange multipliers in force vector at the end of time step")),
		  stabilization(initData(&stabilization,
								 false,
								 "stabilization",
								 "apply a stabilization pass on kinematic constraints requesting it")),
		  debug(initData(&debug,
						 unsigned(0),
						 "debug",
						 "print debug stuff")),
		  
		  stabilization_damping(initData(&stabilization_damping,
										 SReal(1e-7),
										 "stabilization_damping",
										 "stabilization damping hint to relax infeasible problems")),
		  integration(initData(&integration,
							   true,
							   "integration",
							   "integrate positions"))
	  {
		
	  }




	  void pouf_solver::send(simulation::Visitor& vis) {
		// scoped::timer step("visitor execution");
		this->getContext()->executeVisitor( &vis );
	  }


	  void pouf_solver::integrate( const core::MechanicalParams* params,
								   core::MultiVecCoordId posId,
								   core::MultiVecDerivId velId ) {
		scoped::timer step("position integration");
		SReal dt = params->dt();

		// integrate positions
		sofa::simulation::common::VectorOperations vop( params, this->getContext() );

		typedef core::behavior::BaseMechanicalState::VMultiOp VMultiOp;
		VMultiOp multi;

		multi.resize(1);

		multi[0].first = posId;
		multi[0].second.push_back( std::make_pair(posId, 1.0) );
		multi[0].second.push_back( std::make_pair(velId, dt) );

		vop.v_multiop( multi );
	  }




	  pouf_solver::~pouf_solver() {
	
	  }




	  // this is c_k computation (see compliant-reference.pdf, section 3)
	  static void compute_momentum(linearsolver::AssembledSystem::vec& res,
								   const tool::graph_vector& state,
								   const tool::mapping_graph& graph) {
		scoped::timer step("momentum computation");

		// sneaky trick
		core::MechanicalParams params;
		params.setMFactor(1.0);
		params.setDx( core::ConstVecDerivId::velocity() );

		typedef linearsolver::AssembledSystem::vec vec;
		vec storage = vec::Zero( state.dim );

		// fill forces vector
		for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {

		  simulation::Node* node = static_cast<simulation::Node*>(graph[i].mstate->getContext());

		  // reset/accumulate/add forces
		  graph[i].mstate->resetForce(&params, core::VecDerivId::force() );
		
		  // ffields
		  for(unsigned j = 0, m = node->forceField.size(); j < m; ++j) {
			core::behavior::BaseForceField* ff = node->forceField[j];
			ff->addMBKdx(&params, core::VecDerivId::force());
		  }

		  // write it to storage 
		  graph[i].mstate->copyToBuffer(storage.data() + state.info[i].off,
										core::VecDerivId::force(),
										state.info[i].dim);

		
		}

		state.pull( storage, graph );
		state.master.get(res, storage);
	  }

	  // this is c_k computation (see compliant-reference.pdf, section 3)
	  static void compute_force(linearsolver::AssembledSystem::vec& res,
								const tool::graph_vector& state,
								const tool::mapping_graph& graph) {
		scoped::timer step("force computation");

		core::MechanicalParams params;

		typedef linearsolver::AssembledSystem::vec vec;
		vec storage = vec::Zero( state.dim );

		// fill forces vector
		for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {

		  simulation::Node* node = static_cast<simulation::Node*>(graph[i].mstate->getContext());

		  // reset/accumulate/add forces
		  graph[i].mstate->resetForce(&params, core::VecDerivId::force() );
		  graph[i].mstate->accumulateForce(&params, core::VecDerivId::force() );

		  // ffields
		  for(unsigned j = 0, m = node->forceField.size(); j < m; ++j) {
			core::behavior::BaseForceField* ff = node->forceField[j];
			if(!ff->isCompliance.getValue() ) {
			  ff->addForce(&params, core::VecDerivId::force());
			}
			
		  }

		  // interaction ffields. still don't get why they even exist.
		  for(unsigned j = 0, m = node->interactionForceField.size(); j < m; ++j) {
			core::behavior::BaseInteractionForceField* ff = node->interactionForceField[j];
			if(!ff->isCompliance.getValue() ) {
			  ff->addForce(&params, core::VecDerivId::force());
			}
		  }

		  // write it to storage 
		  graph[i].mstate->copyToBuffer(storage.data() + state.info[i].off,
										core::VecDerivId::force(),
										state.info[i].dim);

		
		}

		// std::cout << "full ck: " << storage.transpose() << std::endl;
		state.pull( storage, graph );
		state.master.get(res, storage);
	  }


	  // this is c_k computation (see compliant-reference.pdf, section
	  // 3) if non-zero, 'forces' is filled with aggregated forces,
	  // useful for geometric stiffness computation
	  static void compute_ck(linearsolver::AssembledSystem::vec& res,
							 const tool::graph_vector& state,
							 const tool::mapping_graph& graph,
							 SReal dt,
							 linearsolver::AssembledSystem::vec* forces = 0) {
		scoped::timer step("ck computation");
		
		// sneaky trick
		core::MechanicalParams params;
		params.setMFactor(1.0 / dt);
		params.setDx( core::ConstVecDerivId::velocity() );

		// ck storage
		typedef linearsolver::AssembledSystem::vec vec;
		vec storage = vec::Zero( state.dim );

		// forces storage
		if( forces ) *forces = vec::Zero( state.dim );
		
		// fill forces vector
		for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {

		  simulation::Node* node = static_cast<simulation::Node*>(graph[i].mstate->getContext());

		  // reset/accumulate/add forces
		  graph[i].mstate->resetForce(&params, core::VecDerivId::force() );
		  graph[i].mstate->accumulateForce(&params, core::VecDerivId::force() );

		  // ffields
		  for(unsigned j = 0, m = node->forceField.size(); j < m; ++j) {
			core::behavior::BaseForceField* ff = node->forceField[j];
			if(!ff->isCompliance.getValue() ) {
			  ff->addForce(&params, core::VecDerivId::force());
			}
		  }

		  // interaction ffields. these should die once and for all.
		  for(unsigned j = 0, m = node->interactionForceField.size(); j < m; ++j) {
			core::behavior::BaseInteractionForceField* ff = node->interactionForceField[j];
			if(!ff->isCompliance.getValue() ) {
			  ff->addForce(&params, core::VecDerivId::force());
			}
		  }

		  // write forces if asked for
		  if( forces ) {
			assert( state.info[i].off < forces->size() );
			assert( state.info[i].off + state.info[i].dim <= forces->size() );
			
			graph[i].mstate->copyToBuffer(forces->data() + state.info[i].off,
										  core::ConstVecDerivId::force(),
										  state.info[i].dim);
		  }
		  
		  // momentum / dt
		  for(unsigned j = 0, m = node->forceField.size(); j < m; ++j) {
			core::behavior::BaseForceField* ff = node->forceField[j];
			ff->addMBKdx(&params, core::VecDerivId::force());
		  }
		  
		  // write ck / dt
		  assert( state.info[i].off < storage.size() );
		  assert( state.info[i].off + state.info[i].dim <= storage.size() );

		  graph[i].mstate->copyToBuffer(storage.data() + state.info[i].off,
										core::ConstVecDerivId::force(),
										state.info[i].dim);

		
		}

		// std::cout << "full ck: " << storage.transpose() << std::endl;
		state.pull( storage, graph );
		state.master.get(res, storage);
		res *= dt;

		if( forces ) state.pull(*forces, graph);
	  }


	  typedef linearsolver::AssembledSystem::vec vec;


	  // copy net forces in force vecid, then trigger geometric
	  // stiffness computation
	  static void prepare_geometric_stiffness(const tool::graph_vector& state,
											  const tool::mapping_graph& graph,
											  const vec& forces) {
		scoped::timer step("geometric stiffness");
		
		// fill force vector
		for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {
		  graph[i].mstate->copyFromBuffer(core::VecDerivId::force(),
										  forces.data() + state.info[i].off,
										  state.info[i].dim);
		}

	  }





	  void pouf_solver::rhs_dynamics(vec& res,
									 const system_type& sys,
									 const vec& ck) const {
		
		assert( res.size() == sys.size() );


		// TODO in compute_forces instead ?
		res.head( sys.m ) = sys.P * ck;

		unsigned off = sys.m;

		if ( sys.n ) res.tail(sys.n).setConstant( -14 );
		
		// compliant dofs
		for(unsigned i = 0, end = sys.compliant.size(); i < end; ++i) {
		  system_type::dofs_type* dofs = sys.compliant[i];

		  const unsigned dim = dofs->getMatrixSize();
		  
		  // fetch constraint value if any
		  BaseConstraintValue::SPtr value =
			dofs->getContext()->get<BaseConstraintValue>( core::objectmodel::BaseContext::Local );

		  // fallback TODO optimize ?
		  if( !value ) {
			value = new ConstraintValue( dofs );
			dofs->getContext()->addObject( value );
			value->init();

		  }

		  const unsigned size = dofs->getSize();
		  value->dynamics(&res(off), size, dim / size, stabilization.getValue());
		  off += dim;
		}
		assert( off == sys.size() );

		nan_check(res, HERE);
	  }

	  void pouf_solver::rhs_correction(vec& res, const system_type& sys) const {
		assert( res.size() == sys.size() );

		// master dofs
		res.head( sys.m ).setZero();
		unsigned off = sys.m;

		// compliant dofs

		for(unsigned i = 0, end = sys.compliant.size(); i < end; ++i) {
		  system_type::dofs_type* dofs = sys.compliant[i];

		  unsigned dim = dofs->getMatrixSize();

		  // fetch constraint value if any
		  BaseConstraintValue::SPtr value =
			dofs->getContext()->get<BaseConstraintValue>( core::objectmodel::BaseContext::Local );

		  // fallback TODO optimize ?
		  if(!value ) {
			value = new ConstraintValue( dofs );
			dofs->getContext()->addObject( value );
			value->init();
		  }

		  const unsigned size = dofs->getSize();
		  value->correction(&res(off), dofs->getSize(), dim / size);

		  off += dim;
		}

		nan_check( res, HERE );
	  }


	  void pouf_solver::buildMparams(core::MechanicalParams& mparams,
									 const core::ExecParams& params,
									 double dt) const
	  {

		SReal mfactor = 1.0;
		SReal bfactor = -dt;
		SReal kfactor = -dt * dt;

		mparams.setExecParams( &params );
		mparams.setMFactor( mfactor );
		mparams.setBFactor( bfactor );
		mparams.setKFactor( kfactor );
		mparams.setDt( dt );

		mparams.setImplicitVelocity( 1.0 );
		mparams.setImplicitPosition( 1.0 );
	  }



	  void pouf_solver::get_state(vec& res, const system_type& sys) const {
		// scoped::timer step("get_state");
		assert( res.size() == sys.size() );

		unsigned off = 0;

		for(unsigned i = 0, end = sys.master.size(); i < end; ++i) {
		  system_type::dofs_type* dofs = sys.master[i];

		  unsigned dim = dofs->getMatrixSize();

		  dofs->copyToBuffer(&res(off), core::VecDerivId::velocity(), dim);
		  off += dim;
		}

		//
		assert( sys.dt );
		vec buffer;
		for(unsigned i = 0, end = sys.compliant.size(); i < end; ++i) {
		  system_type::dofs_type* dofs = sys.compliant[i];

		  const unsigned dim = dofs->getMatrixSize();

		  // resize as needed
		  buffer.resize( std::max<unsigned>(buffer.size(), dim));

		  dofs->copyToBuffer(buffer.data(), lagrange.id().getId( dofs ), dim);

		  // impulse from force
		  res.segment(off, dim) = buffer.head(dim) * sys.dt;

		  off += dim;
		}

		assert( off == sys.size() );

	  }


	  void pouf_solver::set_vel(const system_type& sys, const vec& data) const {

		assert( data.size() == sys.m );

		unsigned off = 0;

		// TODO project v ?
		for(unsigned i = 0, end = sys.master.size(); i < end; ++i) {
		  system_type::dofs_type* dofs = sys.master[i];

		  unsigned dim = dofs->getMatrixSize();

		  dofs->copyFromBuffer(core::VecDerivId::velocity(), &data(off), dim);
		  off += dim;
		}

 

	  }


	  void pouf_solver::update_scene(const tool::graph_vector& state,
									 const tool::mapping_graph& graph,
									 const vec& x,
									 SReal dt) {

		vec storage; 
	
		// propagate velocities
		{
		  storage = vec::Zero(state.dim);
		  state.master.set(storage, x.head( state.master.dim ));
		  state.push(storage, graph);

		  for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {
			graph[i].mstate->copyFromBuffer(core::VecDerivId::velocity(),
											storage.data() + state.info[i].off,
											state.info[i].dim);
		  }
		
		}

		if(!state.compliant.dim) return;

		const vec forces = x.tail(state.compliant.dim) / dt;
	
		// lambdas
		{
		
		  unsigned off = 0;
		  for(unsigned i = 0, n = state.compliant.vertex.size(); i < n; ++i) {

			const unsigned vertex = state.compliant.vertex[i];
			const unsigned dim = state.info[vertex].dim;
		
			graph[vertex].mstate->copyFromBuffer(lagrange.id().getId( graph[vertex].mstate ),
												 forces.data() + off,
												 dim);
			off += dim;
		  }
		
		}

		// aggregate lambdas
		if( aggregate_lambdas.getValue() ) {
		  storage = vec::Zero(state.dim);
		
		  state.compliant.set(storage, forces);
		  state.pull(storage, graph);
		
		  for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {
			graph[i].mstate->copyFromBuffer(core::VecDerivId::force(),
											storage.data() + state.info[i].off,
											state.info[i].dim);
		  }
		
		}
	
	  }



	  void pouf_solver::solve(const core::ExecParams* params,
							  double dt,
							  core::MultiVecCoordId posId,
							  core::MultiVecDerivId velId) {
		assert(kkt);
		scoped::timer step("odesolver solve");
		
		// mechanical parameters
		core::MechanicalParams mparams;
		this->buildMparams( mparams, *params, dt );

		// graph
		tool::mapping_graph graph;
		graph.set(this->getContext());

			// lambdas
		{
		  scoped::timer step("realloc");
		  sofa::simulation::common::VectorOperations vop( params, this->getContext() );

		  if( lagrange.id().isNull() ) {
			lagrange.realloc( &vop, false, true );
		  }

		  for(unsigned i = 0, n = boost::num_vertices(graph); i < n; ++i) {
			graph[i].mstate->vRealloc(&mparams, lagrange.id().getId(graph[i].mstate));
		  }

		}

		const tool::graph_vector state(graph);

		vec ck = vec::Zero(state.master.dim);
		vec forces;
		
		compute_ck(ck, state, graph, dt, &forces);
		prepare_geometric_stiffness(state, graph, forces);
		
		const system_type sys = assemble(graph, state, mparams);
		
		if( debug.getValue() > 1) sys.debug();

		// system factor
		{
		  scoped::timer step("system factor");
		  kkt->factor( sys );
		}

		// backup current state
		vec current = vec::Zero(sys.size());
		get_state(current, sys);

		// system solution / rhs
		vec x = vec::Zero(sys.size());
		vec rhs = vec::Zero(sys.size());

		vec corr = vec::Zero(sys.size());
		
		// ready to solve yo
		{
		  scoped::timer step("system solve");

		  // constraint stabilization
		  if( sys.n && stabilization.getValue() ) {
			scoped::timer step("correction");

			x = vec::Zero( sys.size() );
			rhs_correction(rhs, sys);
				
			kkt->correct(x, sys, rhs, stabilization_damping.getValue() );
			nan_check(x, HERE);
			
			if( debug.getValue() ) {
			  std::cerr << "correction rhs:" << std::endl
						<< rhs.transpose() << std::endl
						<< "solution:" << std::endl
						<< x.transpose() << std::endl;
			}

			// set_vel(sys, x.head(sys.m));
			// integrate( &mparams, posId, velId );
			corr = x.head(sys.m);
		  }

		  // actual dynamics
		  {
			scoped::timer step("dynamics");
			x = vec::Zero( sys.size() );

			if( warm_start.getValue() ) x = current;
			rhs_dynamics(rhs, sys, ck);

			kkt->solve(x, sys, rhs);
			nan_check(x, HERE);
			
			if( debug.getValue() ) {
			  std::cerr << "dynamics rhs:" << std::endl
						<< rhs.transpose() << std::endl
						<< "solution:" << std::endl
						<< x.transpose() << std::endl;
			}

			
			if( integration.getValue() ) {
			  corr += x.head(sys.m);
			  set_vel(sys, corr);
			  integrate( &mparams, posId, velId );
			}

			update_scene(state, graph, x, dt);
			
		  }

		}

	  }



	  void pouf_solver::init() {

		// do want KKTSolver !
		kkt = getContext()->get<kkt_type>(core::objectmodel::BaseContext::Local);

		// TODO slightly less dramatic error, maybe ?
		if( !kkt ) throw std::logic_error("AssembledSolver needs a KKTSolver");

		// are we directly under root ?

		core::objectmodel::BaseNode* node = dynamic_cast<core::objectmodel::BaseNode*>(getContext());
		assert( node );
	
	  }



	}
  }
}
