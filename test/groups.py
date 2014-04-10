import Sofa
import pouf

from pouf import robot
from pouf import rigid
from pouf import joint
from pouf import control
from pouf import script

path = pouf.path()

def createScene(node):
    scene = pouf.tool.scene( node )

    num = node.createObject('SequentialSolver',
                            iterations = 25,
                            precision = 0,
                            omega = 1.2)

    resp = node.createObject('LDLTResponse',
                             regularize = 0)
    
    ode = node.getObject('ode')
    
    ground = rigid.Body('ground')

    ground.collision = path + '/share/mesh/ground.obj'
    ground.dofs.translation = [0, -1.1, 0]
    ground.group = 0
    ground.insert( scene )
    ground.node.createObject('FixedConstraint', 
                             indices = '0' )

    box1 = rigid.Body('box1')
    box1.collision = path + '/share/mesh/cube.obj'
    box1.group = 1
    box1.dofs.translation = [-2, 2, 0]
    
    box1.insert( scene )

    box2 = rigid.Body('box2')
    box2.collision = path + '/share/mesh/cube.obj'
    box2.group = 1
    box2.dofs.translation = [2, 2, 0]

    box2.insert( scene )


    return 0
