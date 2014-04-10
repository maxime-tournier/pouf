import Sofa
import pouf

from pouf import robot
from pouf import rigid
from pouf import joint

path = pouf.path()



def createScene(node):
    scene = pouf.tool.scene( node )

    num = node.createObject('SequentialSolver',
                            iterations = 30,
                            precision = 0)

    ode = node.getObject('ode')
    
    ground = rigid.Body('ground')
    ground.collision = ground.visual
    ground.dofs.translation = [0, -1.1, 0]
    
    ground.insert( scene )
    ground.node.createObject('FixedConstraint', 
                             indices = '0' )
    
    robot = pouf.robot.Humanoid('robot')
    robot.insert( scene )

    
    return 0
