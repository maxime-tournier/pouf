#include "LocalFrameMapping.h"

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/Mapping.inl>

namespace sofa
{

namespace component
{

namespace mapping
{

SOFA_DECL_CLASS(LocalFrameMapping)

using namespace defaulttype;

// Register in the Factory
static int LocalFrameMappingClass = core::RegisterObject("Local frame rigid mapping")

#ifndef SOFA_FLOAT
.add< LocalFrameMapping< Rigid3dTypes, Rigid3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
.add< LocalFrameMapping< Rigid3fTypes, Rigid3fTypes > >()
#endif
;

#ifndef SOFA_FLOAT
template class SOFA_pouf_API LocalFrameMapping<  Rigid3dTypes, Rigid3dTypes >;
#endif

#ifndef SOFA_DOUBLE
template class SOFA_pouf_API LocalFrameMapping< Rigid3fTypes, Rigid3fTypes >;

#endif



} // namespace mapping

} // namespace component

} // namespace sofa

