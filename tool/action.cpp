#include "action.h"

#include <sofa/core/ObjectFactory.h>



#include <iostream>

#include "register_object.h"


// SOFA_DECL_CLASS( action );

static int _ = tool::register_object< action<hello> >("execute a single action on component init");

template class SOFA_pouf_API action<hello>;


