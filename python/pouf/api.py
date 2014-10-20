import ctypes
import types

# base class for pointer wrappers
class Class(ctypes.c_void_p):

    def __getattribute__(self, attr):
        c = object.__getattribute__(self, '__class__')
        d = c.__dict__

        # look for attribute in class dictionary, and bind method if
        # found
        if attr in d and type(d[attr]) == dll.__func_type__:
            return types.MethodType(d[attr], self)
        else:
            return object.__getattribute__(self, attr)
        

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
class Node(Class): pass

# setup
dll = load()
dll.simulation.restype = Simulation
dll.__func_type__ = type(dll.simulation)


# helper
def func(f, **kwargs):
    for k in kwargs:
        setattr(f, k, kwargs[k])
    return f


# avanti
Simulation.root = func(dll.simulation_root, restype = Node)

Object.name = func(dll.object_name, restype = ctypes.c_char_p)

Node.object = func(dll.node_object, restype = Object)
Node.child = func(dll.node_child, restype = Node)
Node.name = func(dll.node_name, restype = ctypes.c_char_p)
