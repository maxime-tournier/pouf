import Sofa
from Compliant import Tools

from Compliant.types import Quaternion, Rigid3, vec

import tool
import rigid
import joint
import quat

from . import path

from tool import concat

import numpy as np

_mesh_path = path() + '/share/mesh'

def _make_rigid(name, mesh, position = None, density = 1000 ):
    res = rigid.Body( name )
    
    res.visual = mesh
    res.collision = mesh
    res.mass_from_mesh( res.visual, density )

    res.inertia_forces = True

    if position is not None:
        res.dofs.center = position
        
    return res

class Humanoid:


    def add_group(self, segments):
        self.groups.append( segments )

        for i in segments:
            i.groups.append( len(self.groups))

    # TODO add some parameters yo
    def __init__(self, name = 'robot', **kwargs):

        self.name = name

        scale = 0.15
        self.groups = []
        
        self.height = 0.3
        height = self.height

        self.lfoot = _make_rigid('lfoot', 
                                _mesh_path + '/foot.obj',
                                scale * vec(1, height + 0, -0.95) )

        self.rfoot = _make_rigid('rfoot',
                                _mesh_path + '/foot.obj',
                                scale * vec( -1, height + 0, -0.95 ))
        
        self.ltibia = _make_rigid('ltibia',
                                 _mesh_path + '/leg.obj',
                                 scale * vec( 1, height + 2.1, -1))

        self.rtibia = _make_rigid( 'rtibia',
                                  _mesh_path + '/leg.obj',
                                  scale * vec( -1, height + 2.1, -1 ))

        self.lfemur = _make_rigid( 'lfemur',
                                  _mesh_path + '/leg.obj',
                                  scale * vec( 1, height + 6, -1 ))
        
        self.rfemur = _make_rigid( 'rfemur',
                                  _mesh_path + '/leg.obj',
                                  scale * vec( -1, height + 6, -1 ))
        
        self.body = _make_rigid( 'body',
                                _mesh_path + '/body.obj',
                                scale * vec( 0, height + 10, -1 ))

        arm_width = 1.7
        
        self.larm = _make_rigid( 'larm', 
                                _mesh_path + '/arm.obj',
                                scale * vec( arm_width, height + 10.5, -1 ))

        self.lforearm = _make_rigid( 'lforearm', 
                                    _mesh_path + '/arm.obj',
                                    scale * vec( arm_width, height + 7.5, -1 ))

        self.rarm = _make_rigid( 'rarm', 
                                _mesh_path + '/arm.obj',
                                scale * vec( -arm_width, height + 10.5, -1 ))

        self.rforearm = _make_rigid( 'rforearm', 
                                    _mesh_path + '/arm.obj',
                                    scale * vec( -arm_width, height + 7.5, -1 ))


        self.head = _make_rigid( 'head', 
                                _mesh_path + '/head.obj',
                                 scale * vec( 0, height + 13, -0.8 ))
        
        self.ltoe = _make_rigid('ltoe', 
                               _mesh_path + '/toe.obj',
                               scale * vec(1, height + 0, 0.4) )

        self.rtoe = _make_rigid('rtoe', 
                               _mesh_path + '/toe.obj',
                               scale * vec(-1, height + 0, 0.4) )

        
        self.segments = [ self.ltoe, self.lfoot, self.ltibia, self.lfemur,
                          self.rtoe, self.rfoot, self.rtibia, self.rfemur,
                          self.body,
                          self.larm, self.lforearm,
                          self.rarm, self.rforearm,
                          self.head ]

        # total mass
        self.mass = sum( [ s.mass for s in self.segments ] )
        
        # collision groups
        self.add_group( self.arm_segments('l') )
        self.add_group( self.arm_segments('r') )
        
        self.add_group( [self.lfoot, self.ltoe] )
        self.add_group( [self.rfoot, self.rtoe] )

        self.add_group( [self.lfemur, self.body] )
        self.add_group( [self.rfemur, self.body] )
            
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
        limits = kwargs.get('limits', True)
        
        if limits:
            self.lknee.lower_limit = 1e-1
            self.rknee.lower_limit = 1e-1

            self.lelbow.upper_limit = 1e-1
            self.relbow.upper_limit = 1e-1

        # self.lankle.upper_limit = math.pi / 6
        # self.rankle.upper_limit = math.pi / 6


    # is the robot contacting the environment ?
    def contact(self, segment, other = None):
        user = segment.node.getChild('user')
        collision = user.getChild('collision')
        
        for c in collision.getChildren():
            if 'contact points' in c.name:

                if other is None: return True

                # mapped by the two objects
                d = c.getChildren()[0]
                parents = d.getParents()

                for p in parents:
                    test = p.getParents()[0].getParents()[0].getParents()[0]
                    if test.name == other.name: return True
                
        return False


    def insert(self, scene):

        res = scene.createChild( self.name )

        frame = rigid.Rigid3()

        # insert segments
        for s in self.segments:
            s.insert( res )

        # joint definition
        scale = 0.15

        frame.center = scale * np.array([1, self.height + 0.5, -1])
        self.lankle.absolute(frame, self.ltibia.node, self.lfoot.node)

        frame.center = scale * np.array([1, self.height + 4, -1])
        self.lknee.absolute(frame, self.lfemur.node, self.ltibia.node)

        frame.center = scale * np.array([-1, self.height + 0.5, -1])
        self.rankle.absolute(frame, self.rtibia.node, self.rfoot.node)
        
        frame.center = scale * np.array([-1, self.height + 4, -1])
        self.rknee.absolute(frame, self.rfemur.node, self.rtibia.node)

        frame.center = scale * np.array([1, self.height + 8, -1])
        self.lhip.absolute(frame, self.body.node, self.lfemur.node)

        frame.center = scale * np.array([-1, self.height + 8, -1])
        self.rhip.absolute(frame, self.body.node, self.rfemur.node)

        # shoulders
        frame.center = scale * np.array([self.larm.dofs.center[0] / scale,
                                              self.height + 12, -1])
        self.lshoulder.absolute(frame, self.body.node, self.larm.node)

        frame.center = scale * np.array([self.rarm.dofs.center[0] / scale,
                                              self.height + 12, -1])
        
        self.rshoulder.absolute(frame, self.body.node, self.rarm.node)

        elbow = (self.larm.dofs.center[1] + self.lforearm.dofs.center[1])/ ( 2.0 * scale )

        frame.center = scale * np.array([self.larm.dofs.center[0] / scale,
                                              elbow, -1])
        self.lelbow.absolute(frame, self.larm.node, self.lforearm.node)

        frame.center = scale * np.array([self.rarm.dofs.center[0] / scale,
                                              elbow, -1])
        self.relbow.absolute(frame, self.rarm.node, self.rforearm.node)

        frame.center = scale * np.array([0, self.height + 12.5, -1.3])
        self.neck.absolute(frame, self.body.node, self.head.node)

        phal = 0.15

        frame.center = scale * np.array([1, self.height + 0, phal])
        self.lphal.absolute(frame, self.lfoot.node, self.ltoe.node)

        frame.center = scale * np.array([-1, self.height + 0, phal])
        self.rphal.absolute(frame, self.rfoot.node, self.rtoe.node)


        # insert joints
        for j in self.joints:
            j.node = j.insert( res )

        
        # TODO the rest should go elsewhere
        self.node = res
        return res       
    

    # angular momentum about c (usually com)
    def am(self, c):
        res = np.zeros(3)
        
        for s in self.segments:
            
            dofs = s.node.getObject('dofs')
            assert dofs

            omega = dofs.velocity[0][-3:]
            v = dofs.velocity[0][:3]

            q = dofs.position[0][-4:]
            p = dofs.position[0][:3]
            q_bar = quat.conj(q)

            body = quat.rotate(q_bar, omega)

            tmp = [ mi * gi for mi, gi in zip(s.inertia, body) ]
            
            mu = quat.rotate(q, tmp)
        
            res += vec(*mu) + np.cross(vec(*p) - vec(*c), s.mass * vec(*v) )
            
        return res

    #
    def mid_feet(self):

        data = [ (2., self.lfoot),
                 (2., self.rfoot),
                 (1., self.ltoe),
                 (1., self.rtoe) ]

        return sum([ x[0] * rigid.center(x[1].node) for x in data] ) / sum([x[0] for x in data])

    
    # center of mass
    def com(self):
        res = np.zeros(3)
        
        for s in self.segments:
            dofs = s.node.getObject('dofs')
            
            pos = vec(*dofs.position[0][:3])
            res += s.mass * pos
            
        return res / self.mass

    # com derivative
    def dcom(self):
        res = np.zeros(3)
        
        for s in self.segments:
            pos = vec(*s.node.getObject('dofs').velocity[0][:3])
            res += s.mass * pos
            
        return res / self.mass

    # direction of body z
    def heading(self):
        return quat.rotate( rigid.rotation( self.body.node ),
                            np.array([0., 0., 1.]) )
    

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
                theta = vec(*f).norm()
                total += theta
                
                if theta > 0:
                    if res == None:
                        res = theta * vec(*p)
                    else:
                        res += theta * vec(*p)
        # end loop
        if res is None: 
            return None
            
        return res / total


    def bad_cop(self, wrench):
        force = vec(*wrench[:3])
        torque = vec(*wrench[-3:])
        
        f = hat(force.data)
        Q = np.dot( f.transpose(), f)
        
        kkt = np.zeros( (4, 4) )
        kkt[0:3, 0:3] = Q
        
        A = np.array([0., 1., 0.])

        kkt[0:3, 3] = A
        kkt[3, 0:3] = A.transpose()

        rhs = np.zeros( 4 )
        rhs[0:3] = force.cross(torque).data

        try:
            x = np.linalg.solve(kkt, rhs)
            return vec(*x[0:3])
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


    def leg_segments(self, side):

        return map(lambda x: getattr(self, side + x),
                   ['femur', 'tibia', 'foot', 'toe'])








    def joint_space(self, node):
        res = node.createChild('joint-space')

        n = self.inner_dofs
    
        dofs = res.createObject('MechanicalObject',
                                name = 'dofs',
                                template = 'Vec1d',
                                position = concat( [0] * n ),
                                velocity = concat( [0] * n ) )
        
        
        input = []

        for j in self.joints:
            input.append( '@{0}/{1}'.format( Tools.node_path_rel(res, j.node ),
                                             'dofs' ) )

        m = 6 * len(self.joints)
        
        matrix = np.zeros( (n, m) )
        value = np.zeros(n)

        for i in range(n):
            col = 0

            for ji, j in enumerate(self.joints):
                for di, d in enumerate(j.dofs):
                    if d != 0:
                        val = (i == col)
                        matrix[i, ji * 6 + di] = val
                        col += 1

        map = res.createObject('AffineMultiMapping',
                               name = 'map',
                               template = 'Vec6d,Vec1d',
                               input = concat( input ),
                               output = '@dofs',
                               matrix = concat( matrix.reshape( matrix.size ).tolist() ),
                               value = concat( value ) )

        
        return res



    def named_joints(self):
        for j in self.joints:
            col = 0
            for i, enabled in enumerate(j.dofs):
                if enabled:
                    key = (j.name, col)
                    yield key
                    col += 1
