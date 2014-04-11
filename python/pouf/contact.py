from scipy.spatial import ConvexHull

import numpy as np
from numpy import array as vec

# active point/force list, given a sofa node (created from pouf)
def active(ground):
    collision = ground.getChild('user').getChild('collision')
    
    res = []
    for cp in collision.getChildren():
        dofs = cp.getObject('dofs')
        res.extend( [(vec(p), -vec(f)) 
                     for p, f in zip( dofs.position, dofs.force )
                     if np.dot(f, f) > 0
                 ])
    return res

# support polygon, given the output from active. only for 2d,
# axis-aligned surface!

# hint: if you get empty polygon all the time, make sure
# aggregate_lambda is active in the pouf.solver

def polygon( info ):
    if len(info) == 0:
        return []

    # 2d points
    points = np.zeros( (len(info), 2) )
    for i, (p, f) in enumerate(info):
        points[i, :] = [p[0], p[2]]

    # convex hull
    if len(points) > 3:
        hull = ConvexHull( points )
        return hull.vertices
    else:
        return range(len(info))


def wrench( node ):
    return np.array( node.getObject('dofs').force[0] )

# center of pressure, see [Sardain & Bessonnet 2004]
def cop(wrench):

    # normal
    n = np.array([0, 1, 0])
    
    return np.cross( n, wrench[3:] ) / np.dot(n, wrench[:3] )


import gl
import OpenGL
from OpenGL.GL import *

def draw(active, polygon, com, scale = 1.5e-3 ):
    glLineWidth(2.0)
    glDisable(GL_LIGHTING)

    # (closed) polygon
    glColor([0, 1, 0] )
    hull = [ active[i][0] for i in polygon ]
    gl.line_strip( hull + [hull[0]] )

    # contact forces
    glColor([1, 1, 0])
    for i in range(len(active)):
        (p, f) = active[i]
        gl.line( p, p + scale * f )

    # a contact point on the ground
    origin = active[0][0]

    # contact wrench at origin
    w = np.zeros( 6 )
    for (p, f) in active:
        w[:3] += f
        w[3:] += np.cross(p - origin, f)

    # cop
    c = origin + cop( w )

    glColor([1, 0, 0])
    gl.line( c, c + scale * w[:3] )

    # com projection
    glPointSize(4.0)
    glColor([0, 0.5, 1])
    com_proj = np.copy(com)
    com_proj[1] = origin[1]
    
    glBegin(GL_POINTS)
    glVertex(com_proj)
    glEnd()
    
    glEnable(GL_LIGHTING)
    glLineWidth(1.0)
    glPointSize(1.0)

