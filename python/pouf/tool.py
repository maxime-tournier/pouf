
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
                            stabilization = False)
    
    
    return scene 
