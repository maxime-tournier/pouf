# helper functions for treating lists as quaternions

import numpy as np
import math

def zero():
    return np.zeros(4)

def id():
    res = zero()
    res[3] = 1
    return res

def conj(q):
    return np.array( [-q[0], -q[1], -q[2], q[3]] )

def inv(q):
    return 1.0 / np.dot(q, q) * conj(q)

def re(q):
    return q[3]

def im(q):
    return np.array(q[:3])

# TODO optimize
def prod(a, b):
    res = zero()

    res[:3] = re(a) * im(b) + (re(b) * im(a)) + np.cross(im(a), im(b))
    res[3] = re(a) * re(b) - np.dot( im(a), im(b) )

    return res

# TODO optimize
def rotate(q, x):
    # TODO assert q is unit
    tmp = zero()
    tmp[:3] = x
    return im( prod(q, prod( tmp, conj(q))) )


# exponential
def exp(v):
    theta = np.dot(v, v)
    s = math.sin(theta / 2)
    c = math.cos(theta / 2)

    return np.array([ v[0] / theta * s,
                      v[1] / theta * s,
                      v[2] / theta * s,
                      c ])
             
