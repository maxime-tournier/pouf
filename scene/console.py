import Sofa
from SofaPython import console

def createScene(node):
    print "usage: as always in python, call dir() to see what's available."
    c = console.Console( locals() )
    
    import atexit

    def handler():
        print 'handler'

    atexit.register(handler)
    



