
import Sofa

# this eases the process of adding a script and getting its handle
_event_name = 'RetrieveScriptHandle'


def _noop(*args):
    return 0

# we delegate on 'impl' attribute
class Proxy(Sofa.PythonScriptController):
    
    def onScriptEvent(self, sender, event, data):
        # print 'retrieving script handle'
        
        if event == _event_name:
            data.res = self;
            return 0
        else:
            return getattr(self.impl, 'onScriptEvent', _noop)(sender, event, data)


    def onBeginAnimationStep(self, dt):
        return getattr(self.impl, 'onBeginAnimationStep', _noop)(dt)

    def onEndAnimationStep(self, dt):
        return getattr(self.impl, 'onEndAnimationStep', _noop)(dt)

    def reset(self):
        return getattr(self.impl, 'reset', _noop)()

    def draw(self):
        return getattr(self.impl, 'draw', _noop)()

    def onKeyPressed(self, k):
        return getattr(self.impl, 'onKeyPressed', _noop)(k)

    def bwdInitGraph(self, node):
        return getattr(self.impl, 'bwdInitGraph', _noop)(node)

    # def onLoaded(self, node):
    #     return getattr(self.impl, 'onLoaded', _noop)(node)

    # TODO more ?
        


# adds a python script controller, send a custom event to get the
# python instance and return it
def __insert( node, **kwargs ):
    
    node.createObject('PythonScriptController',
                      **kwargs)
    # TODO: check that classname value derives from Base

    class Data:
        pass

    data = Data()

    node.sendScriptEvent(_event_name, data)

    # TODO clean error message if data.res was not found 

    return data.res


# insert a script into the scene using all the machinery above
def insert(node, script):
    res = __insert(node, filename = __file__, classname = 'Proxy' )
    res.impl = script

    


# convenience
class Base:
    def __init__(self, node):
        insert(node, self)


