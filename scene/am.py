import Sofa
import pouf

from pouf import robot
from pouf import rigid
from pouf import joint
from pouf import control
from pouf import script
from pouf import pose
from pouf import contact
from pouf import tool

import Compliant
from Compliant import Tools

from pouf.tool import concat

import math
import numpy as np

path = pouf.path()

class Script:

    def __init__(self):
        pass

    
    def onBeginAnimationStep(self, dt):
        self.servo.pre_step(dt)

        # obtain contact infos
        self.active = pouf.contact.active( self.ground.node )
        self.polygon = pouf.contact.polygon( self.active )

        return 0

    def onEndAnimationStep(self, dt):
        self.servo.post_step(dt)
        
        return 0

    def reset(self):
        self.servo.reset()
        self.polygon = None
        
        return 0
    

    def draw(self):

        if self.polygon is not None and len( self.polygon ) > 0:
            pouf.contact.draw(self.active,
                              self.polygon,
                              self.com)
            
        return 0



        

def createScene(node):
    scene = pouf.tool.scene( node )
    
    num = node.createObject('SequentialSolver',
                            iterations = 42,
                            precision = 0,
                            nlnscg = True)

    ode = node.getObject('ode')

    # ode.stabilization = True
    
    # ground
    ground = pouf.tool.ground(scene)

    # robot
    robot = pouf.robot.Humanoid('robot')
    robot.insert( scene )

    # servo
    servo = pouf.control.PID(robot)

    pouf.pose.setup( servo )
    servo.set('pos', pouf.pose.stand() )
    
    # angular momentum constraint
    dofs = [ j.node.getObject('dofs') for j in robot.joints ]

    script = Script()
    pouf.script.insert( node, script )
    
    class AMConstraint(Compliant.tool.Constraint):

        def on_apply(self):

            com = robot.com()
            am = robot.am( com )
            
            # print self.jacobian[0]
            # print self.in_vel_stack()

            self.value[:] = np.zeros(3).reshape( (3, 1) )
            u = self.in_vel_stack()
            
            # update jacobian
            pouf.control.broyden(self.jacobian, u, am) 
            self.jacobian += 1e-14
            
            script.am = am
            script.com = com
            
    # TODO damping ? 
    constraint = AMConstraint('am_constraint', node, 3, input = dofs)
    
    # script

    script.robot = robot
    script.servo = servo
    script.ground = ground
    
    return 0
