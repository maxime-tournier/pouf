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

        if len(self.active) > 0:
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

            self.constraint.matrix = self.H + pouf.tool.hat(s).dot(self.L)
            self.constraint.value = current + dt * np.cross(s, m * g )
        
            self.constraint.update()

    

    def onEndAnimationStep(self, dt):
        self.fsm.step()
        self.servo.post_step(dt)
        
        return 0

    def reset(self):
        self.fsm.start()
        self.servo.reset()

        self.polygon = None
        self.constraint.value.fill(0)

        return 0
    

    def draw(self):

        if self.polygon != None and len( self.polygon ) > 0:
            pouf.contact.draw(self.active,
                              self.polygon,
                              self.com)
            
        return 0




def side_stand_pose( side, flex ):

    other = { 'r' : 'l',
              'l' : 'r' }

    sign = {'l':-1,
            'r':1}
    
    pose = { (other[side] + 'knee', 0): 10 * flex,
             (side + 'knee', 0): 2 * flex,
             
             (other[side] + 'hip', 0): -2 * flex,
             (side + 'hip', 0): 0,

             ( other[side] + 'hip', 2): 0,
             ( side + 'hip', 2): 0,

             ( side + 'ankle', 0): -2 * flex,
             
             ( other[side] + 'ankle', 2): 0,
             ( side + 'ankle', 2): 0,

             ( other[side] + 'phal', 0): -flex,


             ('lshoulder', 2): flex,
             ('rshoulder', 2): -flex,
             
             ('lelbow', 0): 2 * flex,
             ('relbow', 0): 2 * flex,

             ('lshoulder', 0): 0,
             ('rshoulder', 0): 0,
    }

    return pose


def swing_pose( side, flex ):

    other = { 'r' : 'l',
              'l' : 'r' }

    sign = {'l':-1,
            'r':1}
    
    pose = { (side + 'hip', 0): 2 * flex,
             (other[side] + 'hip', 0): -2 * flex,

             (side + 'hip', 2): 0,
             (other[side] + 'hip', 2): 0,

             # 
             (other[side] + 'ankle', 0): 0,
             (side + 'ankle', 0): 2 * flex,

             
             (side + 'ankle', 2): 0,
             (other[side] + 'ankle', 2): 0,

             # 
             (side + 'phal', 0): -3 * flex,
             (other[side] + 'phal', 0): 0,

             ('lshoulder', 2): flex,
             ('rshoulder', 2): -flex,

             ('lknee', 0): 0,
             ('rknee', 0): 0,             

             (side + 'shoulder', 0): -flex,
             (other[side] +'shoulder', 0): flex,

             ('lelbow', 0): 2 * flex,
             ('relbow', 0): 2 * flex,

    }

    return pose




class StateGraph:

    def __init__(self, servo, root):

        self.states = ['start', 'stand',
                       'left_stand', 'left_swing',
                       'right_stand', 'right_swing'
        ]
        
        self.transitions = [ ('always', 'start', 'left_stand'),
                             ('left_unstable', 'left_stand', 'left_swing'),
                             ('right_extended', 'left_swing', 'right_stand'),
                             ('right_unstable', 'right_stand', 'right_swing'),
                             ('left_extended', 'right_swing', 'left_stand')
        ]
        
        self.start = 'start'
        
        self.servo = servo
        self.root = root
        self.flex = math.pi / 14
        

        self.speed = 0.02
        
    # helpers
    def push(self, state):
        print state
        self.last = (state, self.root.getTime())


    def interp(self, alpha):

        current = self.servo.get('pos', self.target )

        mix = { key : (1 - alpha) * current[ key ] + alpha * self.target[key]
                for key in self.target }

        self.servo.set('pos', mix)


    def unstable(self, side):
        robot = self.servo.robot
        com = robot.com()
        
        l = pouf.rigid.translation( robot.lfoot.node )
        r = pouf.rigid.translation( robot.rfoot.node )
        
        heading = robot.heading()

        foot = locals()[side]
        
        diff = foot - com
        diff[1] = 0

        past = (foot - com).dot( heading ) < 0

        dist = math.sqrt( diff.dot(diff) )

        return past and dist > 0.2


    def extended(self, side):
        robot = self.servo.robot
        com = robot.com()
        dcom = robot.dcom()

        l = pouf.rigid.translation( robot.lfoot.node )
        r = pouf.rigid.translation( robot.rfoot.node )
        
        dl = np.array(robot.lfoot.node.getObject('dofs').velocity[0][:3])
        dr = np.array(robot.rfoot.node.getObject('dofs').velocity[0][:3])
        
        heading = robot.heading()

        foot = locals()[side]
        dfoot = locals()['d' + side]

        past = (foot - com).dot( heading ) > 0
        backwards = (dfoot - dcom).dot( heading ) <= 1e-1

        return past and backwards


    

    def left_unstable(self):
        return self.unstable('l')

    def right_unstable(self):
        return self.unstable('r')


    def left_extended(self):
        return self.extended('l')

    def right_extended(self):
        return self.extended('r')

        
    # events
    def always(self):
        return True
        
    def wait(self):
        return (self.root.getTime() - self.last[1]) > 1

    def stable(self):
        return self.wait()

    
    # states
    def enter_start(self):
        for p in self.servo.pid:
            p.pos = 0        

    def enter_stand(self):
        self.target = pouf.pose.stand( self.flex )
        self.push('stand')

    def while_stand( self ):
        self.interp(self.speed)
        

    # left
    def enter_left_stand(self):
        self.push('left_stand')
        self.target = side_stand_pose('l', self.flex )
        
    def while_left_stand(self):
        self.interp(self.speed)

    def enter_left_swing(self):
        self.push('left_swing')
        self.target = swing_pose('l', self.flex )

    def while_left_swing(self):
        self.interp(self.speed)

    # right
    def enter_right_stand(self):
        self.push('right_stand')
        self.target = side_stand_pose('r', self.flex )
        
    def while_right_stand(self):
        self.interp(self.speed)

    def enter_right_swing(self):
        self.push('right_swing')
        self.target = swing_pose('r', self.flex )

    def while_right_swing(self):
        self.interp(self.speed)


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

    for p in servo.pid:
        p.kd = 1
        p.ki = 0


    fsm = pouf.control.FSM( StateGraph(servo, node) )

    # cop control
    dofs = [ j.node.getObject('dofs') for j in robot.joints ]
    constraint = pouf.control.Constraint('constraint', node, dofs, 3)
    constraint.compliance = 1e3
    constraint.damping = 5

    # script
    script = Script()
    pouf.script.insert( node, script )

    script.robot = robot
    script.servo = servo
    script.fsm = fsm
    script.ground = ground
    script.constraint = constraint;

    
    robot.body.node.createObject("FixedConstraint",
                                 indices = '0' )


    return 0
