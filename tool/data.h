#ifndef DATA_H
#define DATA_H

#include "../init.h"
#include <sofa/core/objectmodel/BaseObject.h>

// a placeholder for a data
class SOFA_pouf_API Data : public sofa::core::objectmodel::BaseObject {

  sofa::Data< sofa::vector<double> > value;
  
 public:
  Data();

  SOFA_CLASS(Data, sofa::core::objectmodel::BaseObject);
  
};

#endif
