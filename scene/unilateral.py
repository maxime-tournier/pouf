
import Sofa
import pouf

from pouf import rigid
from pouf import tool
from pouf import script
from pouf.tool import concat
from pouf import quat, gui

from PySide import QtGui, QtCore

import math

mesh_path = pouf.path() + '/share/mesh'

import pouf.plot


# from pysofa import core
from SofaPython import console


import numpy as np
import random


random.seed(14)

class Script:


    def reset( self ):
        self.count = 0
    
    def onEndAnimationStep(self, dt):
        every = int(0.045 / dt)

        if self.count % every == 0 and self.node.getTime() <= 30:
            radius = 0.02

            rho = 1 + random.randrange(1000) / 10
            mass = 4/3 * math.pi * (radius * radius * radius)
            height = 0.7

            vel = [0, -1, 0]

            ball(self.balls, [-0.0, height, -0.0], mass, radius, vel)
            # box(self.balls, [0, 0.6, 0], mass, radius)
            
        self.count += 1

        if not self.gui.plot.isChecked():
            return 0
        
        data = {}
        cv = [ x[0] for x in self.qp.convergence if x[0] > 0]

        # print self.qp.convergence
        
        if len(cv) > 0:
            last = int(cv[-1])
            
            dynamics = cv[-(last + 1):-1]
            correction = cv[0:-(last + 2)]
            
            data['dynamics'] = (range(len(dynamics)), dynamics)
            data['correction'] = (range(len(correction)), correction)

            pouf.plot.draw( data, None, True )
            


# def box(pos, mass = 1):
#     b = pouf.rigid.Body()
#     b.collision = mesh_path + '/cube.obj'
#     b.mass = mass
#     b.inertia = [mass, mass, mass]
#     b.dofs.translation = pos
#     return b

def ball(node, pos, mass, radius = 0.5, vel = None):

    res = node.createChild('ball')
    
    dofs = res.createObject('MechanicalObject', template = "Vec3d", position = concat(pos) )
    res.createObject('UniformMass', tempalte = 'Vec3d', mass = mass)
    res.createObject('SphereModel', radius = radius)

    if vel:
        dofs.velocity = concat(vel)
    
    
    return res

def box(node, pos, mass, radius = 0.5):
    
    res = rigid.Body('box-{0}'.format(random.randrange(1000)))
    res.collision = pouf.path() + '/share/mesh/cube.obj'
    res.dofs.translation = pos
    # res.mass_from_mesh( res.collision, 10 )

    res.scale = 2 * radius * np.ones(3)
    res.mass = mass
    res.inertia = res.scale * res.scale * mass
    
    res.insert( node )

    return res.node



def stack(node, n):
    for i in range(n):
        box([0, 1 + 1.5 * i, 0]).insert( node )


def bowl(pos):
    b = pouf.rigid.Body('bowl')
    b.collision = Sofa.src_dir() + '/share/mesh/SaladBowl.obj'
    b.mass_from_mesh( b.collision )
    b.dofs.translation = pos
    b.dofs.rotation = pouf.quat.exp( [-math.pi / 2, 0, 0] )
    b.scale = [1, 1, 1]
    return b


class Gui(pouf.gui.Singleton):
    def __init__(self, parent = None):
        pouf.gui.Singleton.__init__(self, parent)

        
        lay = QtGui.QVBoxLayout()
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(QtGui.QLabel('plot:'))
        self.plot = QtGui.QCheckBox()
        hbox.addWidget( self.plot )
        lay.addLayout(hbox)

        self.setLayout( lay )
        self.setWindowTitle('config')
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        



def createScene(node):
    scene = pouf.tool.scene( node )

    # node.createObject('RequiredPlugin',
    #                   pluginName = 'CompliantDev' )
    
    # num = node.createObject('pouf.qp',
    #                         iterations = 100,
    #                         precision = 1e-8,
    #                         schur = True,
    #                         filename = '/tmp/lcp')


    # num = node.createObject('pouf.jacobi',
    #                         iterations = 10,
    #                         precision = 1e-5,
    #                         homogenize = False,
    #                         nlnscg = 0,
    #                         anderson = 4,
    #                         log = False,
    #                         omega = 1,
    # )

    
    

    # node.createObject('LDLTResponse', regularize = 0)


    ode = node.getObject('ode')
    ode.debug = 0
    ode.stabilization = 1
    
    node.removeObject(ode)
    ode = node.createObject('CompliantImplicitSolver')

    num = node.createObject('ModulusSolver',
                            iterations = 30,
                            precision = 1e-8,
                            anderson = 2)

    num = node.createObject('SequentialSolver',
                            iterations = 30,
                            precision = 1e-8)



    # ode.stabilization = True
    # ode.stabilization_damping = 1e-3


    
    proximity = node.getObject('proximity')
    proximity.alarmDistance = 0.005
    proximity.contactDistance = 0.001
    
    manager = node.getObject('manager')
    
    manager.response = 'CompliantResponse'
    manager.responseParams = 'damping=0&compliance=0&restitution=0'
    
    ground = pouf.tool.ground(scene, position = [0, 1, 0], scale = [0.1, 0.1, 0.1] )


    
    # stack(scene, 20)
    
    script = Script()
    script.qp = num
    script.node = node

    script.gui = Gui()
    script.gui.show()
    pouf.script.insert( node, script )
    
    script.balls = scene.createChild('balls')
    bowl([0, 0.1, 0]).insert(scene)
    
    node.dt = 5e-3
    node.dt = 1e-2

    # lcp callback

    data = {}
    
    def process( info ):
        t = node.getTime()

        # correction
        if 'time' not in data or data['time'] != t:
            data['correction'] = info
            data['time'] = t
        else:
            data['dynamics'] = info


    def save( prefix ):

        def write(f, row ):
            f.write(' '.join( (str(x) for x in row) )) 
            f.write('\n')

        for what in ['correction', 'dynamics']:

            # lcp
            with open('{}.{}.lcp'.format(prefix, what), 'w') as f:
                (M, q, d) = data[what]
                n = q.size

                write(f, [n])

                for row in M: write(f, row)
                write( f, q )

            # ms
            with open('{}.{}.lcp.ms'.format(prefix, what), 'w') as f:
                write( f, d )
                
            
    import ctypes
    def array(obj, size, T = ctypes.c_double):
        return ctypes.cast(obj._ptr_, ctypes.POINTER(T))[0:size]
    
    def cb(n, MM, qq, dd):

        M = np.zeros( n * n )
        q = np.zeros( n )
        d = np.zeros( n )

        M[:] = array(MM, n * n)
        M = M.reshape( (n, n) )
        
        q[:] = array( qq, n )
        d[:] = array( dd, n )

        process( (M, q, d) )
        
    # script.cb = cb
    
    # num = core.sofa_object( num ).cast()
    # num = num.derived().cast()

    # num.log = True
    # num.set_cb( cb )
    
    c = console.Console( locals() )
    
