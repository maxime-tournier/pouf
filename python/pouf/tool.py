from . import path

import re

import numpy as np

def concat(x):
    return ' '.join(map(str, x))



# insert ground mesh into a node
def ground(node, **kwargs):

    position = kwargs.get('position', None)
    scale = kwargs.get('scale', [1, 1, 1])
    mesh = kwargs.get('mesh', 'ground')

    res = rigid.Body('ground')
    res.visual = path() + '/share/mesh/{}.obj'.format(mesh)
    res.collision = res.visual
    res.dofs.center = [0, -1.1, 0]
    res.scale = scale
    
    if position is not None:
        res.dofs.center += position
        
    res.insert( node )
    res.node.createObject('FixedConstraint', 
                          indices = '0' )

    return res


def matrix_size(dofs):
    if type(dofs.velocity) == float: return 1
    return len(dofs.velocity) * len( dofs.velocity[0] )

def template(dofs):
    data = dofs.findData('velocity').getValueTypeString()
    res = re.match(r'.*<(.*)>.*', data).group(1)
    return res


def gravity(node):
    return np.array(node.gravity[0])






import rigid
# a reasonable standard scene
def scene(root):

    node = root
    node.createObject('RequiredPlugin', pluginName = "Compliant" )
    
    node.dt = 0.005
    node.gravity = '0 -9.81 0'


    node.createObject('DefaultPipeline', name = 'pipeline')

    node.createObject('BruteForceDetection', name = 'detection')

    proximity = node.createObject('NewProximityIntersection',
                                  name = 'proximity' )

    proximity.alarmDistance = 0.021
    proximity.contactDistance = 0.02

    manager = node.createObject('DefaultContactManager',
                                name = 'manager',
                                response = "FrictionCompliantContact",
                                responseParams = "mu=0.7" )
    
    style = node.createObject('VisualStyle', 
                              name = 'style',
                              displayFlags =
                              'hideBehaviorModels showCollisionModels hideMappings hideForceFields hideVisualModels')
    
    scene = node.createChild('scene')

    ode = node.createObject('CompliantImplicitSolver',
                            name = 'ode',
                            warm_start = False,
                            stabilization = False,
                            propagate_lambdas = True)

    # ode = node.createObject('CompliantImplicitSolver',
    #                         name = 'ode',
    #                         warm_start = False,
    #                         stabilization = 0,
    #                         propagate_lambda = False)
    
    
    
    return scene 



import ctypes
import platform

def dll(name = 'pouf'):
    system = platform.system()
    
    extension = '.so'

    if system == 'Windows':
        extension = '.dll'
    elif system == 'Darwin':
        extension = '.dylib'

    prefix = 'lib'
    full =  prefix + name + extension

    print 'loading', full

    return ctypes.CDLL( full )


