import Sofa
import pouf

from pouf import console


class Script(Sofa.PythonScriptController):

    def onBeginAnimationStep(self, dt):

        if config.debug:
            print 'onBeginAnimationStep'


            
class Config:

    def __init__(self):
        self.debug = False





from PySide import QtCore

from datetime import datetime



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

