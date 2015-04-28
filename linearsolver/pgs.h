#ifndef POUF_PGS_H
#define POUF_PGS_H

#include <numericalsolver/SequentialSolver.h>
#include <numericalsolver/Response.h>
#include <thread/pool.h>

#include "../init.h"

// a more cache-friendly sequential solver
class SOFA_pouf_API pgs : public sofa::component::linearsolver::SequentialSolver {

public:

  SOFA_CLASS(pgs, sofa::component::linearsolver::SequentialSolver);
  
  virtual void factor(const system_type& system);

  pgs();


  void solve(vec& res,
			 const system_type& sys,
			 const vec& rhs) const;
  
  void correct(vec& res,
			   const system_type& sys,
			   const vec& rhs,
			   real damping ) const;

  void reset();

protected:

  thread::pool pool;
  
  virtual void solve_block(chunk_type result,
						   const inverse_type& inv,
						   chunk_type rhs) const;

  typedef system_type::dmat dense_matrix;
  typedef Eigen::Map< dense_matrix > view_type;
  typedef Eigen::Map< const dense_matrix > const_view_type;


  sofa::Data<bool> nlnscg;
  sofa::Data<unsigned> accel;

  virtual void fetch_blocks(const system_type& system);
  
  void solve_impl(vec& x,
				  const system_type& system,
				  const vec& rhs,
				  bool correct,
				  real damping = 0) const;


  real step(vec& lambda,
			vec& net, 
			const system_type& sys,
			const vec& rhs,
			vec& error, vec& delta,
			bool correct ) const;


  
  // contiguous data for inverses
  vec inverse_storage;
  std::vector<unsigned> offsets;

  sofa::Data<unsigned> threads;
  sofa::Data<bool> homogenize;
  
  vec diagonal;

  sofa::Data<bool> log;
  mutable sofa::Data<sofa::vector<real> > convergence;

  sofa::Data<std::string> filename;

};




#endif
