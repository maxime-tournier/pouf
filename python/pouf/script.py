
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

    # can't call this 
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

        # callbacks
        self.pre_step = []
        self.post_step = []
        self.on_reset = []
        self.on_draw = []

    def onBeginAnimationStep(self, dt):
        for cb in self.pre_step:
            cb(dt)

    def onEndAnimationStep(self, dt):
        for cb in self.post_step:
            cb(dt)

    def reset(self):
        for cb in self.on_reset:
            cb()

def make(node, filename, **kwargs):

    class Generator(Sofa.PythonScriptController):

        def onLoaded(self, node):
            global shared
            self.data = shared
            del shared

        def _gen(self, what):
                return self.data[what]() if what in self.data else None
            
        def reset(self):
            self.simulation = self._gen('simulation')
            self.on_draw = self._gen('draw')

            if self.simulation: self.simulation.next()
            
        def onBeginAnimationStep(self, dt):
            if self.simulation: self.simulation.send( dt )

        def draw(self):
            if self.on_draw: self.on_draw.next()
            

    global shared
    shared = kwargs

    # monkey-patch calling module with controller class
    import os
    mod_name = os.path.splitext( os.path.basename(filename) )[0]
    module = __import__(mod_name)

    module.__controller = Generator

    ret = node.createObject('PythonScriptController',
                            filename = filename,
                            classname = '__controller')

    
    return ret
