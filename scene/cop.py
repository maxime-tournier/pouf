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
        self.H = None
        self.L = None
        pass

    
    def onBeginAnimationStep(self, dt):
        self.servo.pre_step(dt)

        # obtain contact infos
        self.active = pouf.contact.active( self.ground.node )
        self.polygon = pouf.contact.polygon( self.active )
        self.com = self.servo.robot.com()

        # broyden update for am jacobian
        self.am = self.servo.robot.am( self.com )
        self.dcom = self.servo.robot.dcom()

        if self.H == None: self.H = np.zeros( np.shape(self.constraint.matrix) )
        if self.L == None: self.L = np.zeros( np.shape(self.constraint.matrix) )
        
        self.constraint.enable( len(self.active) > 0 )

        if self.constraint.enabled():
            u = self.constraint.constrained_velocity()
            robot = self.servo.robot

            m = robot.mass

            pouf.control.broyden(self.H, u, self.am);
            pouf.control.broyden(self.L, u, m * self.dcom);
            # desired cop

            cop = pouf.contact.centroid( [ self.active[i][0] for i in self.polygon ] )

            s = self.com - cop
            current = self.am + np.cross(s, m * self.dcom)

            g = np.array([0, -9.81, 0])

            robot = self.servo.robot
            
            self.constraint.matrix = self.H + pouf.tool.hat(s).dot(self.L)
            self.constraint.value = current + dt * np.cross(s, m * g )
        
            self.constraint.update()
        
        return 0

    def onEndAnimationStep(self, dt):
        self.servo.post_step(dt)
        
        return 0

    def reset(self):
        self.servo.reset()
        self.polygon = None

        # self.constraint.matrix.fill(0)
        self.constraint.value.fill(0)

        
        return 0
    

    def draw(self):

        if self.polygon != None and len( self.polygon ) > 0:
            pouf.contact.draw(self.active,
                              self.polygon,
                              self.com)
            
        return 0



        

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

    pouf.pose.setup( servo )
    servo.set('pos', pouf.pose.stand() )
    
    # cop control
    dofs = [ j.node.getObject('dofs') for j in robot.joints ]
    constraint = pouf.control.Constraint('constraint', node, dofs, 3)
    constraint.compliance = 0.1

    # critical damping
    constraint.damping = 15
    
    # script
    script = Script()
    pouf.script.insert( node, script )

    script.robot = robot
    script.servo = servo
    script.ground = ground
    script.constraint = constraint;
    
    return 0
