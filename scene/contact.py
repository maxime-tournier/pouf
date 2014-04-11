import Sofa
import pouf

from pouf import robot
from pouf import rigid
from pouf import joint
from pouf import control
from pouf import script
from pouf import pose
from pouf import contact

import math
import numpy as np

from pouf import gl

# if you get errors on this import make sure pouf.gl is imported first
# (it disables error checking, which is the only way to get it working
# on my computer)
import OpenGL
from OpenGL.GL import *

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
    
# a simple scene demonstrating drawing, contact forces, support
# polygon, and center of pressure
def createScene(node):

    # default scene node
    scene = pouf.tool.scene( node )

    # numerical solver
    num = node.createObject('pouf.pgs',
                            name = 'num',
                            iterations = 30,
                            precision = 0)
    # ground 
    ground = pouf.tool.ground( scene )

    # robot
    robot = pouf.robot.Humanoid('robot')
    robot.insert( scene )

    # pid controllers
    servo = pouf.control.PID(robot)

    # setup pid gains with reasonable defaults
    pouf.pose.setup( servo )

    # set desired pose
    servo.set('pos', pouf.pose.stand( math.pi / 16 ) )
    
    # script
    script = Script()
    
    script.servo = servo
    script.ground = ground
    pouf.script.insert( node, script )

    return 0
