
import itertools

from itertools import chain as seq
from itertools import izip_longest as par

import bisect


memory = {}        

# iterates gen( memory[what] ) whenever available. you probably want
# memory[what] to be a mutable structure
def requires(what, gen):
    g = memory
    while True:
        if what in g:
            for x in gen(g[what]):
                yield x
                if what not in g:
                    break
            
        yield

# put non-None results from gen into memory[what]. you probably want
# to yield a mutable structure from gen
def results(what, gen):
    g = memory
    for x in gen:
        if x: g[what] = x
        elif what in g: del g[what]
        yield x


# convenience class for using requires/results
class Variable:

    def __rlshift__(self, gen):
        return requires(self, gen)


    def __rrshift__(self, gen):
        return results(self, gen)


za
    
    
# resource is shared between generators while Context is per generator
class Resource(object):

    def __init__(self):
        self.queue = []
        self.user = None

    class Context(object):
        def __init__(self, resource):
            self.resource = resource

        def __nonzero__(self):
            return not self.release

        def acquire(self, p):
            bisect.insort(self.resource.queue, p)
            self.priority = p
            
            on_top = self.resource.queue[-1] == p
            
            while not (self.resource.user is None and on_top):
                if on_top: self.resource.user.release = True
                yield

            self.release = False
            self.resource.user = self

        def __del__(self):
            self.resource.queue.remove( self.priority )
            
    def __enter__(self):
        return Resource.Context(self)

    def __exit__(self, *args):
        self.user = None




class FSM:
    '''simple finite state machine based on generators'''

    def __iter__(self):

        lookup = {}
        for cond, src, dest in self.transitions:
            lookup.setdefault(src, []).append( (cond, dest) )
        
        current = self.start
        gen = current()

        yield next(gen)

        while True:
            for (cond, dest) in lookup[current]:
                if cond():
                    current = dest
                    gen = current()

                    print('cond', cond.__name__)
                    print('state', current.__name__)

                    break
                
            yield next( gen )


