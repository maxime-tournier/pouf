#include "data.h"
#include <sofa/core/ObjectFactory.h>

SOFA_DECL_CLASS(Data);
static int BenchmarkClass = sofa::core::RegisterObject("A data placeholder.").add< Data >();


Data::Data() :
  value(initData(&value, "value", "data value")) {

}
