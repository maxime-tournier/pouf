
import rigid
import joint
import quat

from . import path

from tool import concat

import numpy as np
from numpy import array as vec

_mesh_path = path() + '/share/mesh'

def _make_rigid(name, mesh, position = None, density = 1000 ):
    res = rigid.Body( name )
    
    res.visual = mesh
    res.collision = mesh
    res.mass_from_mesh( res.visual, density )

    res.inertia_forces = True

    if position != None:
        res.dofs.translation = position
        
    return res

class Humanoid:

    # TODO add some parameters yo
    def __init__(self, name):

        self.name = name

        scale = 0.15


        self.height = 0.3
        height = self.height

        self.lfoot = _make_rigid('lfoot', 
                                _mesh_path + '/foot.obj',
                                scale * vec([1, height + 0, -0.95]) )

        self.rfoot = _make_rigid('rfoot',
                                _mesh_path + '/foot.obj',
                                scale * vec( [-1, height + 0, -0.95] ))
        
        self.ltibia = _make_rigid('ltibia',
                                 _mesh_path + '/leg.obj',
                                 scale * vec( [1, height + 2.1, -1]))

        self.rtibia = _make_rigid( 'rtibia',
                                  _mesh_path + '/leg.obj',
                                  scale * vec( [-1, height + 2.1, -1] ))

        self.lfemur = _make_rigid( 'lfemur',
                                  _mesh_path + '/leg.obj',
                                  scale * vec( [1, height + 6, -1] ))
        
        self.rfemur = _make_rigid( 'rfemur',
                                  _mesh_path + '/leg.obj',
                                  scale * vec( [-1, height + 6, -1] ))
        
        self.body = _make_rigid( 'body',
                                _mesh_path + '/body.obj',
                                scale * vec( [0, height + 10, -1] ))

        arm_width = 1.7
        
        self.larm = _make_rigid( 'larm', 
                                _mesh_path + '/arm.obj',
                                scale * vec( [arm_width, height + 10.5, -1] ))

        self.lforearm = _make_rigid( 'lforearm', 
                                    _mesh_path + '/arm.obj',
                                    scale * vec( [arm_width, height + 7.5, -1] ))

        self.rarm = _make_rigid( 'rarm', 
                                _mesh_path + '/arm.obj',
                                scale * vec( [-arm_width, height + 10.5, -1] ))

        self.rforearm = _make_rigid( 'rforearm', 
                                    _mesh_path + '/arm.obj',
                                    scale * vec( [-arm_width, height + 7.5, -1] ))


        self.head = _make_rigid( 'head', 
                                _mesh_path + '/head.obj',
                                 scale * vec( [0, height + 13, -0.8] ))
        
        self.ltoe = _make_rigid('ltoe', 
                               _mesh_path + '/toe.obj',
                               scale * vec([1, height + 0, 0.4]) )

        self.rtoe = _make_rigid('rtoe', 
                               _mesh_path + '/toe.obj',
                               scale * vec([-1, height + 0, 0.4]) )

        
        self.segments = [ self.ltoe, self.lfoot, self.ltibia, self.lfemur,
                          self.rtoe, self.rfoot, self.rtibia, self.rfemur,
                          self.body,
                          self.larm, self.lforearm,
                          self.rarm, self.rforearm,
                          self.head ]

        # total mass
        self.mass = sum( [ s.mass for s in self.segments ] )
        
        # collision groups
        for i in self.arm_segments('l'):
            i.group = 1

        for i in self.arm_segments('r'):
            i.group = 2

        for i in [self.lfoot, self.ltoe]:
            i.group = 3

        for i in [self.rfoot, self.rtoe]:
            i.group = 4

        # for i in [self.head, self.body]:
        #     i.group = 5
        

        # for i in [self.lfoot, self.ltibia]:
        #     i.group = 3

        # for i in [self.rfoot, self.rtibia]:
        #     i.group = 4

        
        # joints
        self.lknee = joint.Revolute(0, name = 'lknee' )
        self.rknee = joint.Revolute(0, name = 'rknee' )

        self.lhip = joint.Spherical(name = 'lhip')
        self.rhip = joint.Spherical(name = 'rhip')
        
        self.lankle = joint.Spherical(name = 'lankle')
        self.rankle = joint.Spherical(name = 'rankle')
        
        self.lshoulder = joint.Spherical(name = 'lshoulder')
        self.rshoulder = joint.Spherical(name = 'rshoulder')

        self.lelbow = joint.Revolute(0, name = 'lelbow')
        self.relbow = joint.Revolute(0, name = 'relbow')
        
        self.neck = joint.Spherical(name = 'neck')

        self.lphal = joint.Revolute(0, name = 'lphal')
        self.rphal = joint.Revolute(0, name = 'rphal')

        self.joints = [ self.lankle, self.lknee, self.lhip,
                        self.rankle, self.rknee, self.rhip,
                        self.lshoulder, self.lelbow,
                        self.rshoulder, self.relbow,
                        self.neck,
                        self.lphal, self.rphal ]

        # inner dofs
        self.inner_dofs = 0
        
        for j in self.joints:
            self.inner_dofs += j.dim()

        # limits
        self.lknee.lower_limit = 0
        self.rknee.lower_limit = 0

        self.lelbow.upper_limit = 0
        self.relbow.upper_limit = 0

        # self.lankle.upper_limit = math.pi / 6
        # self.rankle.upper_limit = math.pi / 6


    # is the robot contacting the environment ?
    def contact(self, segment):
        collision = segment.node.getChild('collision')
        
        for c in collision.getChildren():
            if c.name == 'TTriangleModel contact points':
                return True
                
        return False


    def insert(self, scene):

        res = scene.createChild( self.name )

        frame = rigid.Frame()

        # insert segments
        for s in self.segments:
            s.insert( res )

        # joint definition
        scale = 0.15

        frame.translation = scale * np.array([1, self.height + 0.5, -1])
        self.lankle.absolute(frame, self.ltibia.node, self.lfoot.node)

        frame.translation = scale * np.array([1, self.height + 4, -1])
        self.lknee.absolute(frame, self.lfemur.node, self.ltibia.node)

        frame.translation = scale * np.array([-1, self.height + 0.5, -1])
        self.rankle.absolute(frame, self.rtibia.node, self.rfoot.node)
        
        frame.translation = scale * np.array([-1, self.height + 4, -1])
        self.rknee.absolute(frame, self.rfemur.node, self.rtibia.node)

        frame.translation = scale * np.array([1, self.height + 8, -1])
        self.lhip.absolute(frame, self.body.node, self.lfemur.node)

        frame.translation = scale * np.array([-1, self.height + 8, -1])
        self.rhip.absolute(frame, self.body.node, self.rfemur.node)

        # shoulders
        frame.translation = scale * np.array([self.larm.dofs.translation[0] / scale,
                                              self.height + 12, -1])
        self.lshoulder.absolute(frame, self.body.node, self.larm.node)

        frame.translation = scale * np.array([self.rarm.dofs.translation[0] / scale,
                                              self.height + 12, -1])
        
        self.rshoulder.absolute(frame, self.body.node, self.rarm.node)

        elbow = (self.larm.dofs.translation[1] + self.lforearm.dofs.translation[1])/ ( 2.0 * scale )

        frame.translation = scale * np.array([self.larm.dofs.translation[0] / scale,
                                              elbow, -1])
        self.lelbow.absolute(frame, self.larm.node, self.lforearm.node)

        frame.translation = scale * np.array([self.rarm.dofs.translation[0] / scale,
                                              elbow, -1])
        self.relbow.absolute(frame, self.rarm.node, self.rforearm.node)

        frame.translation = scale * np.array([0, self.height + 13, -1.3])
        self.neck.absolute(frame, self.body.node, self.head.node)

        phal = 0.15

        frame.translation = scale * np.array([1, self.height + 0, phal])
        self.lphal.absolute(frame, self.lfoot.node, self.ltoe.node)

        frame.translation = scale * np.array([-1, self.height + 0, phal])
        self.rphal.absolute(frame, self.rfoot.node, self.rtoe.node)


        # insert joints
        for j in self.joints:
            j.node = j.insert( res )

        
        # TODO the rest should go elsewhere
        self.node = res
        return res       
    

    # angular momentum about c (usually com)
    def am(self, c):
        res = vec( [0, 0, 0] )
        
        for s in self.segments:
            dofs = s.node.getObject('dofs')
            omega = dofs.velocity[0][-3:]
            v = dofs.velocity[0][:3]

            q = dofs.position[0][-4:]
            p = dofs.position[0][:3]
            q_bar = quat.conj(q)

            body = quat.rotate(q_bar, omega)

            tmp = [ mi * gi for mi, gi in zip(s.inertia, body) ]
            
            mu = quat.rotate(q, tmp)
        
            res += vec(mu) + np.cross(vec(p) - vec(c), s.mass * vec(v) )
            
        return res

    # 
    def mid_feet(self):
        return 0.5 * ( rigid.translation( self.lfoot.node ) + rigid.translation( self.rfoot.node) )

    # center of mass
    def com(self):
        res = vec( [0, 0, 0] )
        
        for s in self.segments:
            pos = vec(s.node.getObject('dofs').position[0][:3])
            res += s.mass * pos
            
        return res / self.mass

    # com derivative
    def dcom(self):
        res = vec( [0, 0, 0] )
        
        for s in self.segments:
            pos = vec(s.node.getObject('dofs').velocity[0][:3])
            res += s.mass * pos
            
        return res / self.mass

    # direction of body z
    def heading(self):
        return quat.rotate( rigid.rotation( self.body.node ),
                            np.array([0, 0, 1]) )
    

    # TODO sort these two out ?
    # cop 
    def good_cop(self, node):
        res = None
        total = 0.0
        
        collision = node.getChild('user').getChild('collision')
        
        children = collision.getChildren()

        for cp in children:
            dofs = cp.getObject('dofs')

            for p, f in zip(dofs.position, dofs.force):
                theta = vec(f).norm()
                total += theta
                
                if theta > 0:
                    if res == None:
                        res = theta * vec(p)
                    else:
                        res += theta * vec(p)
        # end loop
        if res == None: 
            return None
            
        return res / total


    def bad_cop(self, wrench):
        force = vec(wrench[:3])
        torque = vec(wrench[-3:])
        
        f = hat(force.data)
        Q = np.dot( f.transpose(), f)
        
        kkt = np.zeros( (4, 4) )
        kkt[0:3, 0:3] = Q
        
        A = np.array([0, 1, 0])

        kkt[0:3, 3] = A
        kkt[3, 0:3] = A.transpose()

        rhs = np.zeros( 4 )
        rhs[0:3] = force.cross(torque).data

        try:
            x = np.linalg.solve(kkt, rhs)
            return vec(x[0:3])
        except np.linalg.LinAlgError:
            return None


    # body parts
    def leg_joints(self, side):
        hip = getattr(self, side + 'hip')
        knee = getattr(self, side + 'knee')
        ankle = getattr(self, side + 'ankle')
        phal = getattr(self, side + 'phal')

        return [hip, knee, ankle, phal]


    def arm_joints(self, side):
        shoulder = getattr(self, side + 'shoulder')
        elbow = getattr(self, side + 'elbow')

        return [shoulder, elbow]

    def arm_segments(self, side):
        arm = getattr(self, side + 'arm')
        forearm = getattr(self, side + 'forearm')

        return [arm, forearm]


