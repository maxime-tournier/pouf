#include "RigidMultiMapping.h"


#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/MultiMapping.inl>

#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace mapping
{

SOFA_DECL_CLASS(RigidMultiMapping)


  using namespace defaulttype;

// Register in the Factory
static int RigidMultiMappingClass = core::RegisterObject("rigid mapping for multiple source dofs")

#ifndef SOFA_FLOAT
.add< RigidMultiMapping< Rigid3dTypes, Vec3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
.add< RigidMultiMapping< Rigid3fTypes, Vec3fTypes > >()
#endif
;

#ifndef SOFA_FLOAT
template class SOFA_Compliant_API RigidMultiMapping<  Rigid3dTypes, Vec3dTypes >;
#endif

#ifndef SOFA_DOUBLE
template class SOFA_Compliant_API RigidMultiMapping< Rigid3fTypes, Vec3fTypes >;

#endif



} // namespace mapping

} // namespace component

} // namespace sofa

