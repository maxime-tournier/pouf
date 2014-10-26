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
from pouf import plot

import Compliant
from Compliant import Tools

from pouf.tool import concat

import math
import numpy as np

from matplotlib import pyplot as plt

from pouf import gui

from PySide import QtGui, QtCore
import gc

path = pouf.path()


class Gui(gui.Singleton):

    def __init__(self, n, parent = None):
        
        gui.Singleton.__init__(self, parent)
        
        lay = QtGui.QVBoxLayout()

        self.am = gui.Constraint()
        lay.addWidget(QtGui.QLabel("am"))
        lay.addWidget( self.am )

        self.com = gui.Constraint()
        lay.addWidget(QtGui.QLabel("com"))
        lay.addWidget( self.com )

        self.draw = gui.Flag('draw')
        lay.addWidget( self.draw )
        
        self.setLayout( lay )
        self.setWindowTitle('Config')
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)


class State( pouf.control.FSM ):

    def __init__(self):
        pouf.control.FSM.__init__(self)
        
        self.states = ['start', 'balance', 'panic' ]
        self.transitions = [ ('ready', 'start', 'balance'),
                             ('derp', 'balance', 'panic') ]
        
        self.initial = 'start'



        self.L = None
        self.H = None
        self.K = None

        self.LF = None
        self.RF = None
        self.BF = None

        self.target_cop = None
        self.target_com = None

        self.com_force = None

    def derp(self):
        return self.robot.com()[1] < 0.3

    def enter_panic(self):
        print 'panic !'
        self.robot.node.getRoot().reset()

    
    def enter_start(self):

        self.am.enable( False )
        self.com.enable( False ) 

        self.prev = None
        self.prev_com = None
        self.prev_delta = None

    def enter_balance(self):
        shape = self.am.matrix.shape
        
        if self.L == None:
            self.L = np.zeros( shape )
            
        if self.H == None:
            self.H = np.zeros(shape)

        if self.K == None:
            self.K = np.zeros( (2, self.am.cols) )

        if self.LF is None:
            self.LF = np.zeros( shape )

        if self.RF is None:
            self.RF = np.zeros( shape )

        if self.BF is None:
            self.BF = np.zeros( shape )


    def cop_ground(self, c, mg, f, z = 0):
        # taiste yo
        delta = f - mg
        
        if delta[1] != 0:
            alpha = (z - c[1]) / delta[1]
        else:
            alpha = 0

        res = c + alpha * delta

        # print 'cop', res, 'z', z
        return res

    
    def while_balance(self):
        # obtain contact infos
        dt = self.robot.node.getRoot().getDt()

        # we want 3 contact points or more
        ok = len(self.balance.active) > 2

        # update gains from gui
        self.gui.am.update(self.am)
        self.gui.com.update(self.com)

        # disable from gui
        self.com.enable( self.gui.com.enabled.isChecked() and ok )
        self.am.enable( self.gui.am.enabled.isChecked() and ok )

        pos = self.am.src_pos()
        vel = self.am.src_vel()

        def norm(x):
            return math.sqrt( x.dot(x) )
            
        am = self.balance.am
        dcom = self.balance.dcom
        com = self.balance.com
        centroid = self.balance.centroid

        mid = self.robot.mid_feet()

        if centroid is not None:
            mid[1] = centroid[1]
        

        m = self.robot.mass
        
        if self.am.enabled():

            # check
            u = self.am.constrained_velocity()

            if self.prev != None:
                u = (pos - self.prev) / dt

            # spring force on com
            com_target = mid
            delta = com_target - (com)
            delta[1] = 0

            ddelta = -dcom
            ddelta[1] = 0

            k = 1 / self.com.compliance
            b = self.com.damping
            
            f = k * delta + b * ddelta
            g = np.array([0, -9.81, 0])

            # desired cop
            cop = self.cop_ground(com, m * g, f, centroid[1])
            cop = pouf.contact.project(self.balance.hull(), cop )

            # cop = mid
            cop[1] = centroid[1]

            # cop = com_target
            cop = centroid
            
            self.target_cop = cop[:]
            self.target_com = com_target
            
            self.com_force = f

            s = com - cop
            
            current = am + np.cross(s, m * dcom)

            v = am
            error = norm(self.H.dot(u) - v) / norm(v)
            print 'am broyden prediction error:', error
            
            # broyden updates
            pouf.control.broyden(self.H, u, v);

            v = m * dcom
            if self.prev_com != None:
                v = m * (com - self.prev_com) / dt
            pouf.control.broyden(self.L, u, v)
            
            self.am.matrix = (self.H + pouf.tool.hat(s).dot(self.L))
            # self.am.matrix = self.H

            # AM change at desired cop should be due to gravity forces only
            value = (current + dt * np.cross(s, m * g))
            
            # value = np.zeros(3)

            self.am.value = dt * value

            factor = 1 / m
            self.am.matrix *= factor
            self.am.value *= factor

            # levmar
            # diag = np.diagonal(self.am.matrix.dot(self.am.matrix.transpose()))
            # self.am.compliance = self.gui.am.compliance.value() / diag
            
            # self.am.compliance = self.gui.am.compliance.value() * error;


            self.am.update()

            self.prev_com = com
            

            # taiste yo

            # x = centroid + f
            
            # s = self.com - x
            # self.constraint.matrix = self.H + pouf.tool.hat(s).dot(self.L)
            # value = np.zeros(3)
            # self.constraint.value = dt * value
            # self.constraint.update()


        if self.com.enabled():
            # spring force on com
            com_target = mid
            delta = com_target - (com)
            delta[1] = 0

            ddelta = -dcom
            ddelta[1] = 0

            u = self.com.constrained_velocity()

            if self.prev != None:
                u = (pos - self.prev) / dt
            
            v = ddelta[0:3:2]
            
            if self.prev_delta != None:
                v = (delta[0:3:2] - self.prev_delta[0:3:2]) / dt

            error = norm(self.K.dot(u) - v) / norm(v)
            print 'com/midfeet prediction error:', error
            pouf.control.broyden(self.K, u, v)
            
            self.com.matrix = self.K # horiz / m
            self.com.value = -delta[0:3:2]

            # levmar stuff
            # TODO this should be K Minv K^T diagonal
            
            # diag = np.diagonal(self.K.dot(self.K.transpose()))
            # self.com.compliance = self.gui.com.compliance.value()) / diag
            
            # print 'com compliance', self.com.compliance
            # print self.com.damping

            # self.com.compliance = self.gui.com.compliance.value() * error
            # self.com.damping = 2 / math.sqrt(self.com.compliance * self.robot.mass)
            
            self.com.update()
            self.prev_delta = delta

            # net = np.zeros(3)
            # for (p, f) in self.balance.active:
            #     net += f

            # print "check:", norm(net) , "ref:", norm(m* g)
        self.prev = pos

            
        
    def ready(self):
        lhit = self.robot.contact(self.robot.lfoot, self.ground.node)
        rhit = self.robot.contact(self.robot.rfoot, self.ground.node)

        return lhit and rhit

  
    
class Script(pouf.script.Base) :


    def reset(self):
        self.fsm.start()
        self.fsm.servo.reset()

        self.count = 0
        
        return 0 

    def onBeginAnimationStep(self, dt):
        self.fsm.servo.pre_step( dt )
        return 0
        
    def onEndAnimationStep(self, dt):
        self.fsm.servo.post_step( dt )
        self.fsm.balance.update( dt )
        self.fsm.step()

        
        if self.count % 10 == 0:
            gc.collect()
            
        self.count += 1
        return 0


    def draw(self):

        if self.fsm.gui.draw.enabled():
            self.fsm.balance.draw()

            if len(self.fsm.balance.active) > 0:
                pouf.gl.line_width(4)
                pouf.gl.lighting( False )

                if self.fsm.target_cop != None:
                    pouf.gl.color([1, 0, 1])
                    bob = np.zeros(3)

                    # target com vs com
                    bob[0:3:2] = self.fsm.balance.com[0:3:2]
                    bob[1] = self.fsm.target_cop[1]
                    pouf.gl.line( bob, self.fsm.target_com )

                    # target cop vs cop
                    pouf.gl.color([1, 1, 1])
                    pouf.gl.line(  self.fsm.balance.cop,
                                   self.fsm.target_cop )

            pouf.gl.lighting( True )
            pouf.gl.line_width( 1.0 )

        return 0



def set_params(gui, p):
    for i in p:
        x = getattr(gui, i)
        for j in p[i]:
            y = getattr(x, j)
            y.setValue(p[i][j])
    
def createScene(node):

    
    scene = pouf.tool.scene( node )

    num = node.createObject('pouf.jacobi',
                            # nlnscg = True,
                            anderson = 4,
                            iterations = 150,
                            threads = 4,
                            newmark = True,
                            precision = 0,
                            omega = 1 )

    ode = node.getObject('ode')
    
    ode.stabilization = True
    ode.stabilization_damping = 1e-3
    
    ode.warm_start = True
    ode.debug = 0

    # ground
    ground = pouf.tool.ground(scene)
    
    # robot
    robot = pouf.robot.Humanoid('robot' )
    robot.insert( scene )

    # world setup
    node.dt = 1e-2

    # dofs basis
    dofs = [j.node.getObject('dofs') for j in robot.joints]


    am = control.Constraint('am-control', node, dofs, 3)
    # am.ff.isCompliance = False

    com = control.Constraint('com-feet-proj-control', node, dofs, 2)
    # com.ff.isCompliance = False
    
    gui = Gui(robot.inner_dofs)
    gui.show()

    gui.am.enabled.setChecked( True )
    gui.com.enabled.setChecked( True )

    c = 2e-4
    p = {
        'am' : {
            'compliance': 4e-4,
            'damping': 0
        },
        'com' : {
            'compliance': c ,
            'damping': 2.0 * (2.0 / math.sqrt( c * robot.mass))
        }
    }


    gui.draw.box.setChecked( True )

    set_params(gui, p)

    servo = control.PID(robot)
    pose.setup( servo, 1e2 )
    servo.set('pos', pose.stand( math.pi / 14 ))
    
    fsm = State()
    
    fsm.robot = robot
    fsm.servo = servo
    fsm.am = am
    fsm.com = com
    
    fsm.balance = pouf.contact.Balance(robot, ground)
    fsm.gui = gui
    fsm.ground = ground
    
    # script
    script = Script(node)

    script.fsm = fsm
    script.gui = gui

    
    return 0
