import Sofa

import ctypes
import os

def createScene(node):


    pouf = ctypes.CDLL( '{0}/lib/libpouf.dylib'.format( Sofa.build_dir() ) )

    pouf.test_dependency('/tmp/prout')
    

    return node


