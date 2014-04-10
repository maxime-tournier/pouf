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

