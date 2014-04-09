

def concat(x):
    return ' '.join(map(str, x))


# a reasonable standard scene
def scene(root):

    node = root
    node.createObject('RequiredPlugin', pluginName = "Compliant" )
    node.createObject('RequiredPlugin', pluginName = "pouf" )
    
    node.dt = 0.01
    node.gravity = '0 -9.81 0'

    node.createObject('DefaultPipeline', name = 'pipeline')
    node.createObject('BruteForceDetection', name = 'detection')
    
    proximity = node.createObject('NewProximityIntersection', name = 'proximity' )
    
    manager = node.createObject('DefaultContactManager',
                                name = 'manager',
                                response = "FrictionCompliantContact",
                                responseParams = "mu=1" )
    
    style = node.createObject('VisualStyle', 
                              name = 'style',
                              displayFlags =
                              'hideBehaviorModels hideCollisionModels hideMappings hideForceFields')
    
    scene = node.createChild('scene')

    ode = node.createObject('pouf_solver', name='ode' )
    
    group = node.createObject('CollisionGroup', 
                              name = 'group' )

    return scene 
