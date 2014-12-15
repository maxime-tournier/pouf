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
        self.polygon = None

    
    def onBeginAnimationStep(self, dt):
        self.servo.pre_step(dt)

        # obtain contact infos
        self.active = pouf.contact.active( self.ground.node )
        self.polygon = pouf.contact.polygon( self.active )
        self.com = self.servo.robot.com()
        
        return 0

    def onEndAnimationStep(self, dt):
        self.servo.post_step(dt)
        
        return 0

    def reset(self):
        self.servo.reset()
        return 0
    

    def draw(self):

        if self.polygon != None and len( self.polygon ) > 0:
            pouf.contact.draw(self.active,
                              self.polygon,
                              self.com)
            
        return 0



        

def createScene(node):
    scene = pouf.tool.scene( node )
    
    num = node.createObject('pouf.pgs',
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
    
    # constraint on joint angles: zero velocities
    dofs = [ robot.lshoulder.node.getObject('dofs') ]
    constraint = pouf.control.Constraint('constraint', node, dofs, 3)
    
    # block dofs for all pids in joint lshoulder
    for i in range(constraint.dim):
        constraint.matrix[i, 3 + i] = 1
        constraint.value[i] = 0

    # note: you should update constraints whenever matrix/value
    # change
    constraint.update()


    # script
    script = Script()
    pouf.script.insert( node, script )

    script.robot = robot
    script.servo = servo
    script.ground = ground
    
    return 0
