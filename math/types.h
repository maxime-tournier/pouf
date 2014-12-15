#ifndef POUF_MATH_TYPES_H
#define POUF_MATH_TYPES_H

#include <Eigen/Core>

namespace math {

  typedef double real;
  
  typedef Eigen::Matrix<real, Eigen::Dynamic, 1> vec;
  typedef Eigen::Matrix<real, 3, 1> vec3;

}


#endif
