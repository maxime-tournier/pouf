import Sofa
import pouf

from pouf import robot
from pouf import rigid
from pouf import joint
from pouf import control
from pouf import script
from pouf import pose
from pouf import contact

import Compliant
from Compliant import Tools

import math

path = pouf.path()

class Script:

    def __init__(self):
        self.polygon = None

    
    def onBeginAnimationStep(self, dt):
        self.servo.pre_step(dt)

        # obtain contact infos
        self.active = pouf.contact.active( self.ground.node )
        self.polygon = pouf.contact.polygon( self.active )
        self.com = self.servo.robot.com()
        
        return 0

    def onEndAnimationStep(self, dt):
        self.fsm.step()
        self.servo.post_step(dt)
        
        return 0

    def reset(self):
        self.fsm.start()
        self.servo.reset()
        return 0
    

    def draw(self):

        if self.polygon != None and len( self.polygon ) > 0:
            pouf.contact.draw(self.active,
                              self.polygon,
                              self.com)
            
        return 0



def Constraint:

    # note: dofs must all have the same dofs
    def __init__(self, parent, name, dofs, dim):

        self.node = parent.createChild(name)

        self.compliance = 0
        self.damping = 0
        
        input = []
        dofs_dim = 0
        
        for n in dofs:
            input.append( Tools.node_path_rel(self.node, n) )
            dofs_dim += len( n.velocity ) * len(n.velocity[0] )

        self.matrix = np.zeros( (dim, dofs_dim) )
        self.value = np.zeros( dim )


        
        self.dofs = self.node.createObject('MechanicalObject',
                                           name = 'dofs',
                                           template = 'Vec1d',
                                           position = concat( [0] * dofs_dim ) )

        template = dofs[0].template
        self.map = self.node.createObject('AffineMultiMapping',
                                          name = 'map',
                                          template = '{0}, Vec1d'.format( template ),
                                          input = concat( input ),
                                          output = '@dofs',
                                          matrix = concat( self.matrix.reshape( self.matrix.size ).tolist() ),
                                          value = concat( -self.value ) )

        self.ff = self.node.createObject('UniformCompliance',
                                         name = 'ff',
                                         template = 'Vec1d',
                                         compliance = self.compliance,
                                         damping = self.damping )

    def update(self):
        self.map.matrix = concat( self.matrix.reshape(self.matrix.size).tolist() )
        self.map.value = concat( -self.value )
        self.map.init()

        self.ff.compliance = self.compliance
        self.ff.damping = self.damping
        self.ff.init()
        

def createScene(node):
    scene = pouf.tool.scene( node )
    
    num = node.createObject('SequentialSolver',
                            iterations = 30,
                            precision = 0)

    ode = node.getObject('ode')

    # ode.stabilization = True
    
    # ground
    ground = pouf.tool.ground(scene)

    # robot
    robot = pouf.robot.Humanoid('robot')
    robot.insert( scene )

    # servo
    servo = pouf.control.PID(robot)

    fsm = pouf.control.FSM( StateGraph(servo, node) )
    
    # script
    script = Script()
    pouf.script.insert( node, script )

    script.robot = robot
    script.servo = servo
    script.fsm = fsm
    script.ground = ground
    
    return 0
