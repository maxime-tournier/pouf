import Sofa
import pouf

from pouf import robot
from pouf import rigid
from pouf import joint
from pouf import control
from pouf import script

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
    


def stand_pose( flex ):
    pose = { 'lknee': (0, 2 * flex),
             'rknee': (0, 2 * flex),
             
             'lhip': (0, -2 * flex),
             'rhip': (0, -2 * flex),
             
             'lankle': (0, -flex),
             'rankle': (0, -flex),
             
             'lshoulder': (2, flex),
             'rshoulder': (2, -flex),
             
             'lelbow': (0, -2 * flex),
             'relbow': (0, -2 * flex),
    }

    return pose

    
def createScene(node):
    scene = pouf.tool.scene( node )

    num = node.createObject('SequentialSolver',
                            iterations = 30,
                            precision = 0)

    ode = node.getObject('ode')
    
    ground = rigid.Body('ground')

    ground.visual = path + '/share/mesh/ground.obj'
    ground.collision = ground.visual
    ground.dofs.translation = [0, -1.1, 0]
    
    ground.insert( scene )
    ground.node.createObject('FixedConstraint', 
                             indices = '0' )
    
    robot = pouf.robot.Humanoid('robot')
    robot.insert( scene )

    servo = pouf.control.PID(robot)

    script = Script()
    pouf.script.insert( node, script )

    script.robot = robot
    script.servo = servo

    stiff = 1e4
    normal = 1e3
    soft = 1e2

    for p in servo.pid:
        p.kp = normal
        p.kd = 1
        p.ki = p.kp / 10


    kp = { 'lphal': stiff,
           'rphal': stiff,
           'lankle': stiff,
           'rankle': stiff,

           'lshoulder': soft,
           'rshoulder': soft,

           'lelbow': soft,
           'relbow': soft,
    }

    
    servo.set('kp', kp)
    servo.set('pos', stand_pose( math.pi / 14.0 ) )

    return 0
