import rigid
from . import path

import re

import numpy as np

def concat(x):
    return ' '.join(map(str, x))

# a reasonable standard scene
def scene(root):

    node = root
    node.createObject('RequiredPlugin', pluginName = "Compliant" )
    node.createObject('RequiredPlugin', pluginName = "pouf" )
    
    node.dt = 0.01
    node.gravity = '0 -9.81 0'


    node.createObject('DefaultPipeline', name = 'pipeline', verbose = True)

    node.createObject('BruteForceDetection', name = 'detection')

    proximity = node.createObject('NewProximityIntersection',
                                  name = 'proximity' )

    proximity.alarmDistance = 0.05
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

    ode = node.createObject('pouf.solver',
                            name = 'ode',
                            warm_start = False,
                            stabilization = False,
                            aggregate_lambdas = True)
    
    
    return scene 


# insert ground mesh into a node
def ground(node, position = None):

    res = rigid.Body('ground')
    res.visual = path() + '/share/mesh/ground.obj'
    res.collision = res.visual
    res.dofs.translation = [0, -1.1, 0]

    if position != None:
        res.dofs.translation += position
        
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


def hat(x):
    return np.array( [[0, -x[2], x[1]],
                      [x[2], 0, -x[0]],
                      [-x[1], x[0], 0]])

