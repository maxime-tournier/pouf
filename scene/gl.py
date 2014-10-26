import Sofa

import OpenGL

# this might be useless for you
OpenGL.ERROR_CHECKING = False

from OpenGL.GL import *


# just a placeholder for sharing data
class Shared:
    pass

global shared
shared = Shared()

# script controller (callbacks on given entry points)
class Controller(Sofa.PythonScriptController):

    def onBeginAnimationStep(self, dt):
        return 0

    def onEndAnimationStep(self, dt):
        return 0

    def reset(self):
        return 0

    def draw(self):

        pos = shared.dofs.position[0]

        glColor([1, 0, 0])
        glDisable(GL_LIGHTING)
        glBegin(GL_LINES)
        glVertex([0, 0, 0])
        glVertex( pos )
        glEnd()
        glEnable(GL_LIGHTING)
        
        return 0

    
def createScene(node):

    # make sure we load the compliant plugin
    node.createObject('RequiredPlugin',
                      pluginName = 'Compliant')

    # ode/num solvers
    ode = node.createObject('CompliantImplicitSolver')
    
    num = node.createObject('SequentialSolver',
                            iterations = 10)

    # main node
    scene = node.createChild('scene')

    # some object node
    obj = scene.createChild('object')

    # sofa components: dofs and mass
    dofs = obj.createObject('MechanicalObject',
                            template = 'Vec3d')

    dofs.position = '0 1 0'
    

    mass = obj.createObject('UniformMass',
                            template = 'Vec3d',
                            mass = 1)

    # load the script controller defined above
    script = node.createObject('PythonScriptController',
                               filename = __file__,
                               classname = 'Controller')

    # shared data with script instance
    shared.dofs = dofs

    # timestep
    node.dt = 1e-2

    print dir(node)
    print node.getChildren()

    print node.getObjects()
    
    # all good, yar !
    return node

