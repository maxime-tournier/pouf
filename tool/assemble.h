#ifndef POUF_ASSEMBLE_H
#define POUF_ASSEMBLE_H


#include <assembly/AssembledSystem.h>

namespace tool {

  struct mapping_graph;
  struct graph_vector;
  
  sofa::component::linearsolver::AssembledSystem assemble(const mapping_graph& graph,
														  const graph_vector& state,
														  const sofa::core::MechanicalParams& mparams);

}

#endif
