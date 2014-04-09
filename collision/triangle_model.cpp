#include "triangle_model.h"

#include <sofa/core/ObjectFactory.h>
#include "../init.h"

int triangle_modelClass = sofa::core::RegisterObject("hack !")
#ifndef SOFA_FLOAT
	.add< triangle_model<sofa::defaulttype::Vec3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
	.add< triangle_model<sofa::defaulttype::Vec3fTypes> >()
#endif
	.addAlias("pouf.TriangleModel");


	
#ifndef SOFA_FLOAT
template class SOFA_pouf_API triangle_model<sofa::defaulttype::Vec3dTypes>;
#endif

#ifndef SOFA_DOUBLE
template class SOFA_pouf_API triangle_model<sofa::defaulttype::Vec3fTypes>;
#endif
