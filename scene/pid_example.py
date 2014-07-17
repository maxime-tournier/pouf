import Sofa

import pouf

from pouf import rigid
from pouf import joint
from pouf import robot

import numpy as np

path = pouf.path()

def createScene(node):
    scene = pouf.scene( node )
    
    num = node.createObject('SequentialSolver',
                            iterations = 100,
                            precision = 0)

    ground = rigid.Body('ground')

    ground.visual = path + '/share/mesh/ground.obj'
    ground.collision = ground.visual
    ground.dofs.translation = [0, -1, 0]
    
    ground.insert( scene )
    ground.node.createObject('FixedConstraint', 
                             indices = '0' )
    
    
    
    return 0


