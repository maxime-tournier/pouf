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
        self.fsm.step()
        self.servo.post_step(dt)
        
        return 0

    def reset(self):
        self.fsm.start()
        self.servo.reset()
        return 0
    


class StateGraph:

    def __init__(self, servo, root):

        self.states = ['start', 'high', 'low' ]
        self.transitions = [ ('wait', 'start', 'high'),
                             ('wait', 'high', 'low'),
                             ('wait', 'low', 'high') ]
        self.start = 'start'
        
        self.servo = servo
        self.root = root

        pouf.pose.setup( servo )

    # transitions
    def wait(self):
        return (self.root.getTime() - self.last) > 2

    # states
    def enter_start(self):
        self.servo.set('pos', pouf.pose.stand( math.pi / 14.0 ) )
        self.last = self.root.getTime()

    def enter_high(self):
        self.servo.set('pos', { ('lshoulder', 2): math.pi / 2,
                                ('rshoulder', 2): -math.pi / 2 } )
        self.last = self.root.getTime()

    def enter_low(self):
        self.servo.set('pos', { ('lshoulder', 2): math.pi / 14,
                                ('rshoulder', 2): -math.pi / 14 } )
        self.last = self.root.getTime()


    
def createScene(node):
    scene = pouf.tool.scene( node )

    num = node.createObject('SequentialSolver',
                            iterations = 30,
                            precision = 0)

    ode = node.getObject('ode')

    # ground
    ground = pouf.tool.ground(scene)
    
    # robot
    robot = pouf.robot.Humanoid('robot')
    robot.insert( scene )

    # servo
    servo = pouf.control.PID(robot)

    # state machine
    fsm = pouf.control.FSM( StateGraph(servo, node) )
    
    # script
    script = Script()
    pouf.script.insert( node, script )

    script.robot = robot
    script.servo = servo
    script.fsm = fsm
    
    return 0
