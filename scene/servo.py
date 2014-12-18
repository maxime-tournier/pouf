import Sofa
import pouf

from pouf import robot
from pouf import rigid
from pouf import joint
from pouf import control
from pouf import script
from pouf import pose

import math

path = pouf.path()

class Script:
    
    def onBeginAnimationStep(self, dt):
        self.servo.pre_step(dt)
        return 0

    def onEndAnimationStep(self, dt):
        self.servo.post_step(dt)
        return 0

    def reset(self):
        self.servo.reset()
        return 0
    
# a very simple scene demonstrating scripting and pid control of a
# humanoid robot posture.
def createScene(node):

    # default scene node
    scene = pouf.tool.scene( node )

    # numerical solver
    num = node.createObject('pouf.pgs',
                            iterations = 30,
                            precision = 0,
                            nlnscg = True)
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

    # change the kp values for ankles
    kp = {
        ('lankle', 0): 100,
        ('rankle', 0): 100,
        
    }

    
    servo.set('kp', kp )
    
    # script
    script = Script()
    script.servo = servo
    pouf.script.insert( node, script )

    return 0
