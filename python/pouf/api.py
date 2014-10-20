import ctypes
import types

# base class for pointer wrappers.
class Class(ctypes.Structure):

    _fields_ = [('_object_', ctypes.c_void_p) ]

    # bind c functions as methods 
    def __getattribute__(self, attr):
        
        res =  object.__getattribute__(self, attr)

        if type(res) == dll.__func_type__:
            return types.MethodType( res, self)
        else:
            return res

    def __nonzero__(self):
        return bool(self._object_)
    

import platform

# dll loading
def load():

    name = 'pouf'
    
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

# classes
class Simulation(Class): pass
class Object(Class): pass


class Node(Class):

    # node/object based on relative path (downwards only)
    def find(self, path):
        split = path.split('/')

        res = self
        
        for i, rel in enumerate( split ):
            old = res

            res = res.child( rel )

            if i + 1 == len(split) and not res:
                res = old.object( rel )
                if not res:
                    error = '{} not found'.format( '/'.join( split[0:i+1] ) )
                    raise Exception, error

        return res


    def children(self):
        res = []
        def callback(x):
            res.append( x )

        self.each_child( callback )

        return res
    
# setup
dll = load()
dll.simulation.restype = Simulation
dll.__func_type__ = type(dll.simulation)


# helper
def func(f, **kwargs):

    for k in kwargs:
        setattr(f, k, kwargs[k])

    res = f

    # indirection to convert parameters if argtypes is given
    if 'argtypes' in kwargs:
        types = kwargs['argtypes']

        def res(*args):
            converted = [ (x(y) if type(y) != x else y ) for x, y in zip(types, args) ]
            return f(*converted)
        return res
    
    else:
        return f


# avanti
Simulation.root = func(dll.simulation_root, restype = Node)

Object.name = func(dll.object_name, restype = ctypes.c_char_p)

Node.object = func(dll.node_object, restype = Object)
Node.child = func(dll.node_child, restype = Node)
Node.name = func(dll.node_name, restype = ctypes.c_char_p)

Node.each_child = func(dll.node_each_child,
                       restype = None,
                       argtypes = [Node, ctypes.CFUNCTYPE(None, Node) ] )

