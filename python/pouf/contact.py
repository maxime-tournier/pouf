from scipy.spatial import ConvexHull

import numpy as np
from numpy import array as vec

import tool
import math


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
        return range(len(points))

    duplicates = False
    if duplicates: 
        # eliminate duplicates to keep QHull happy and store their
        # original index
        dup = { (x[0], x[1]): i for i, x in enumerate(points) }

        # update points without duplicates
        points = np.zeros( (len(dup), 2) )

        for i, (x, y) in enumerate(dup):
            points[i, :] = [x, y]

        # map indices from new to old
        indir = [ dup[k] for k in dup ]

        # convex hull
        if len(points) > 3:
            hull = ConvexHull( points )
            vertices =  hull.vertices 
        else:
            vertices =  range(len(points))
        
        # TODO output warning in case of duplicates ?
        if len(info) != len(indir):
            print 'warning: duplicate contact points'

        # map back to original points
        return [ indir[v] for v in vertices ]

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
    if len(active) == 0: return

    gl.line_width(2.0)
    gl.lighting( False )

    # (closed) polygon
    gl.color([0, 1, 0] )
    hull = [ active[i][0] for i in polygon ]
    gl.line_strip( hull + [hull[0]] )

    # contact forces
    gl.color([1, 1, 0])
    for (p, f) in active:
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
        
    gl.color([1, 0, 0])
    gl.line( c, c + scale * w[:3] )

    # com projection
    gl.point_size(4.0)
    gl.color([0, 0.5, 1])
    com_proj = np.copy(com)
    com_proj[1] = origin[1]
    
    gl.points( com_proj )
    
    gl.lighting( True )
    gl.line_width(1.0)
    gl.point_size(1.0)





# computes centroid of convex hull
def centroid( points ):

    # actually this is useless, we could use any vertex instead
    mean = np.mean(points, axis = 0)

    rows = len(points)
    mass = np.zeros( rows )
    
    m = 0                       # mean mass
    c = 0                       # total mass

    # split convex hull into triangles around mean
    for i in xrange(rows):
        n = i + 1 if i < rows - 1 else 0
        p2 = points[n] 
        p1 = points[i] 
    
        b =  (p2 - p1)

        norm2 = b.dot(b)

        # otherwise triangle is zero area, skip it
        if norm2 > 1e-10:
            h = p1 + b * b.dot(mean) / norm2 - mean

            area = np.sqrt(norm2 * h.dot(h)) / 2
            
            # spread mass across vertices
            mass[i] += area / 3
            mass[n] += area / 3
            
            m += area / 3
            c += area

    # compute weighted average
    p = m * mean
    
    for i in xrange(rows):
        p += mass[i] * points[i]

    # TODO line stuff instead
    if c == 0: return mean

    p /= c

    return p


def project( hull, p ):

    # so that we know where is interior
    c = np.mean(hull, axis = 0)

    if len(hull) == 1:
        return c

    for (start, end) in zip(hull, hull[1:] + [hull[0]]):

        line = end - start

        line_T = np.array([-line[2], line[1], line[0]])

        if line_T.dot(c - start) * line_T.dot(p - start) <= 0:
            # p is outside, project
            line2 = line.dot(line)
            # print 'line', line2

            if line2 < 1e-14:
                print 'degenerate hull !'
                return start

            coord = line.dot(p - start)
            if coord < 0: coord = 0
            if coord > line2: coord = line2
            
            p = start + line * coord / line2
            return p

    return p
        


# most of the stuff needed for balance
# TODO maybe move elsewhere ?
class Balance:
    
    def __init__(self, robot, ground):
        self.robot = robot
        self.ground = ground
        self.gravity = tool.gravity(robot.node)

        self.active = []
        self.am = None
        self.com = None
        
    def draw(self):
        scale = 1.5e-3
        
        
        gl.lighting( False )
        
        gl.color([0.6, 0.6, 0.6])
        gl.line_width(4)
        if self.am != None and self.com != None:
            gl.line( self.com, self.com + scale * 10 * self.am )
        
        gl.color([1., 1., 1.])
        gl.point_size(5.0)
        if self.com != None:
            gl.points( self.com )

        gl.lighting( True )


        if self.active:
            draw(self.active, self.polygon, self.com, scale)

        gl.line_width(1)
        gl.point_size(1)
        

        
    def update(self, dt):
        self.active = active( self.ground.node )
        self.polygon = polygon( self.active )

        self.com = self.robot.com()
        self.dcom = self.robot.dcom()
        self.am = self.robot.am( self.com )

        self.dt = dt
        
        if len(self.active) > 0:
            self.centroid = centroid( self.hull() )
        else:
            self.centroid = None
            return

        
        origin = self.active[0][0]

        # contact wrench at origin
        w = np.zeros( 6 )
        for (p, f) in self.active:
            w[:3] += f
            w[3:] += np.cross(p - origin, f)

        # cop
        self.cop = origin + cop( w )

    # list of vertices
    def hull(self):
        return [ self.active[i][0] for i in self.polygon ]

    def static_stable(self, threshold = 0.8):

        h = self.hull()
        c = self.centroid

        def norm(x): return math.sqrt( x.dot(x) )

        dist = []
        
        for start, end in zip(h[:-1], h[1:]):
            delta = (end - start)[0:3:2]
            local = (c - start)[0:3:2]

            proj = (delta / norm(delta)).dot(local)

            di = math.sqrt( local.dot(local) - proj * proj )
            
            dist.append(di)

        d = sorted( dist )[0]
            
        return len(h) >= 3 and norm( (c - self.com)[0:3:2] ) < threshold * d;
        

        
