#ifndef POUF_JACOBI_H
#define POUF_JACOBI_H

#include <numericalsolver/SequentialSolver.h>
#include <numericalsolver/Response.h>

#include <thread/pool.h>

#include "../init.h"

// a more cache-friendly sequential solver
class jacobi : public sofa::component::linearsolver::SequentialSolver {

public:

  virtual void factor(const system_type& system);

  jacobi();

  void solve(vec& res,
			 const system_type& sys,
			 const vec& rhs) const;
  
  void correct(vec& res,
			   const system_type& sys,
			   const vec& rhs,
			   real damping ) const;

  typedef sofa::component::linearsolver::SequentialSolver base;
  typedef base::cmat cmat;
  typedef base::dense_matrix dense_matrix;

  void reset();

protected:

  real step(vec& lambda,
			vec& net, 
			const system_type& sys,
			const vec& rhs,
			vec& error, vec& delta,
			bool correct ) const;


  typedef Eigen::Map< dense_matrix > view_type;
  typedef Eigen::Map< const dense_matrix > const_view_type;

  sofa::Data<bool> nlnscg;
  sofa::Data<unsigned> anderson;

  virtual void fetch_blocks(const system_type& system);
  
  void solve_impl(vec& x,
				  const system_type& system,
				  const vec& rhs,
				  bool correct,
				  real damping = 0) const;

  // diagonal preconditioner
  vec diagonal;

  sofa::Data<bool> log;
  mutable sofa::Data<sofa::vector<real> > convergence;

  sofa::Data<std::string> filename;

  sofa::Data<bool> homogenize;
  sofa::Data<unsigned> threads;

  thread::pool pool;

};




#endif