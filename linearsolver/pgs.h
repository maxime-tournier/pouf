#ifndef POUF_PGS_H
#define POUF_PGS_H

#include <numericalsolver/SequentialSolver.h>
#include <numericalsolver/Response.h>

#include "../init.h"

// a more cache-friendly sequential solver
class pgs : public sofa::component::linearsolver::SequentialSolver {

public:

  virtual void factor(const system_type& system);

  pgs();
  
protected:

  virtual void solve_block(chunk_type result,
						   const inverse_type& inv,
						   chunk_type rhs) const;

  typedef Eigen::Map< dense_matrix > view_type;
  typedef Eigen::Map< const dense_matrix > const_view_type;


  sofa::Data<bool> nlnscg;
  sofa::Data<unsigned> accel, accel_alt;

  virtual void fetch_blocks(const system_type& system);
  
  void solve_impl(vec& x,
				  const system_type& system,
				  const vec& rhs,
				  bool correct) const;
  
  // contiguous data for inverses
  vec inverse_storage;
  std::vector<unsigned> offsets;

  sofa::Data<bool> log;
  mutable sofa::Data<sofa::vector<real> > convergence;
  

};




#endif
