#ifndef POUF_ASSEMBLE_H
#define POUF_ASSEMBLE_H

#include "mapping_graph.h"
#include <assembly/AssembledSystem.h>

namespace tool {
sofa::component::linearsolver::AssembledSystem assemble(const mapping_graph& graph,
														const sofa::core::MechanicalParams& mparams);

}

#endif
