
import itertools

seq = itertools.chain
par = itertools.izip_longest


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



# concurrent access
locked = {}
priority = {}

import bisect

class Cleanup: pass
    

# wraps the generator f(*args) for concurrent access to 'resource'
# with priority 'p'
def lock(resource, p, f):

    def res(*args):
        self = f(*args)

        if resource not in priority: priority[resource] = []

        bisect.insort(priority[resource], p)

        try:
            while True:

                top = priority[resource][-1]

                if resource not in locked and top == p:
                    locked[resource] = self

                # i have the lock
                if resource in locked and locked[resource] == self:
                    if top == p:
                        yield self.next()
                    else:
                        # cleanup
                        self.throw( StopIteration )
                        yield
                        while self.next(): yield
                        del locked[resource]
                else:
                    yield
        finally:
            priority[resource].remove( p )
            if resource in locked: del locked[resource]
            
    return res
