import Sofa
import pouf

from pouf import console


class Script(Sofa.PythonScriptController):

    def onBeginAnimationStep(self, dt):
        console.poll()

        if config.debug:
            print 'onBeginAnimationStep'


            
class Config:

    def __init__(self):
        self.debug = False


        

def createScene(node):

    node.createObject('PythonScriptController',
                      filename = __file__,
                      classname = 'Script' )

    global console
    global config

    config = Config()

    print 'usage: start the simulation, then type "config.debug = True/False" to enable/disable debugging.'
    console = pouf.console.Console( globals() )

    node.animate = True
