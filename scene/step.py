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
        self.balance = None
        
    
    def onBeginAnimationStep(self, dt):
        self.servo.pre_step(dt)

        self.balance.update(dt)
        self.fsm.step()
        
        return 0

    def onEndAnimationStep(self, dt):
        self.servo.post_step(dt)
        
        return 0

    def reset(self):
        self.servo.reset()
        self.fsm.start()

        return 0
    

    def draw(self):
        if self.balance != None:
            self.balance.draw()

        return 0




class StateMachine:

    def __init__(self, balance, cop, com):
        self.states = ['start', 'both', 'left', 'right']

        self.transitions = [
            ('always', 'start', 'both'),
            ('wait', 'both', 'left'),
            ('wait', 'both', 'right'),

            ('wait', 'left', 'both'),
            ('wait', 'right', 'both')
        ]

        self.start = 'start'

        self.balance = balance

        self.cop = cop
        self.com = com
        
        self.H = np.zeros( np.shape(self.cop.matrix) )
        self.L = np.zeros( np.shape(self.cop.matrix) )
        


    # transitions
    def always(self):
        return True

    def wait(self):
        return self.balance.robot.node.getTime() - self.time > 1


    # states
    def enter(self):
        self.time = self.balance.robot.node.getTime()
    
    def enter_start(self):
        self.enter()


    def update(self):
        u = self.cop.constrained_velocity()
        m = self.balance.robot.mass
    
        pouf.control.broyden(self.H, u, self.balance.am);
        pouf.control.broyden(self.L, u, m * self.balance.dcom);

    def enter_both(self):
        self.enter()

    def while_both(self):
        self.cop.enable( self.balance.centroid != None )

        if self.cop.enabled():
            self.update()
            
            m = self.balance.robot.mass

            # desired cop
            cop = self.balance.centroid
            
            s = self.balance.com - cop
            current = self.balance.am + np.cross(s, m * self.balance.dcom)

            g = self.balance.gravity

            lf = pouf.rigid.translation(self.balance.robot.lfoot.node)
            rf = pouf.rigid.translation(self.balance.robot.rfoot.node)

            mid = 0.5 * (lf + rf)

            cop = mid
            cop[1] = self.balance.centroid[1]

            spring = mid - self.balance.com
            spring[1] = 0

            
            f = m * g + spring * 2e3
            dt = self.balance.dt
            
            self.cop.matrix = self.H + pouf.tool.hat(s + dt * f).dot(self.L)
            self.cop.value = dt * (current + dt * np.cross(s, f))

            self.cop.update()
        
            
            target = np.array( [mid[0], mid[2]] )

            # self.com.matrix = self.L
            # self.cop.value 

    


        

def createScene(node):
    scene = pouf.tool.scene( node )
    
    num = node.createObject('pouf.pgs',
                            iterations = 50,
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

    cop = pouf.control.Constraint('constraint', node, dofs, 3)
    cop.compliance = 1e1
    cop.damping = 0

    # com = pouf.control.Constraint('constraint', node, dofs, 2)
    # com.compliance = 1e14
    # com.damping = 10

    
    # script
    script = Script()
    pouf.script.insert( node, script )

    # balance stuff
    balance = pouf.contact.Balance(robot, ground)

    for p in servo.pid:
        p.ki = 0
        
    # fsm
    fsm = pouf.control.FSM( StateMachine(balance, cop, None) )
    
    script.balance = balance
    script.robot = robot
    script.servo = servo
    script.ground = ground

    script.fsm = fsm
    
    return 0
