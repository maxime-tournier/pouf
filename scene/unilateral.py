
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


import numpy as np

import random

random.seed(14)

class Script:


    def reset( self ):
        self.count = 0
    
    def onEndAnimationStep(self, dt):
        every = int(0.10 / dt)

        if self.count % every == 0 and self.node.getTime() <= 15:
            radius = 0.02

            rho = 1 + random.randrange(1000) / 10
            mass = 4/3 * math.pi * (radius * radius * radius)
            height = 0.7
            ball(self.balls, [-0.0, height, -0.0], mass, radius)
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

def ball(node, pos, mass, radius = 0.5):

    res = node.createChild('ball')
    
    res.createObject('MechanicalObject', template = "Vec3d", position = concat(pos) )
    res.createObject('UniformMass', tempalte = 'Vec3d', mass = mass)
    res.createObject('SphereModel', radius = radius)

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
    b = pouf.rigid.Body()
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


    num = node.createObject('pouf.jacobi',
                            iterations = 50,
                            precision = 1e-8,
                            # nlnscg = True,
                            accel_alt = 4,
                            threads = 4,
                            # accel = 4,
                            #log = True
    )

    node.createObject('LDLTResponse', regularize = 0)
    
    ode = node.getObject('ode')
    ode.stabilization = True
    ode.stabilization_damping = 1e-3

    proximity = node.getObject('proximity')
    proximity.alarmDistance = 0.0012
    proximity.contactDistance = 0.001
    
    manager = node.getObject('manager')

    manager.response = 'CompliantResponse'
    manager.responseParams = 'damping=0&compliance=0&restitution=0'
    
    ground = pouf.tool.ground(scene, [0, 1, 0], [0.1, 0.1, 0.1] )


    
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
