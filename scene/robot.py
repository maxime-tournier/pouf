import Sofa
import pouf

from pouf import robot
from pouf import rigid
from pouf import joint
from pouf import tool

path = pouf.path()



def createScene(node):
    scene = pouf.tool.scene( node )

    num = node.createObject('pgs',
                            iterations = 30,
                            precision = 0)

    ode = node.getObject('ode')
    
    ground = tool.ground(scene, [0, -0.5, 0] )
    
    robot = pouf.robot.Humanoid('robot')
    robot.insert( scene )

    return 0
