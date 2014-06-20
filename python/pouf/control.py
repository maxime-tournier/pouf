import pid

import tool
from tool import concat

import numpy as np


# TODO get rid of this one
from Compliant import Tools

# build pid joints
def _make_pid(joint, class_name = pid.Implicit):
    res = []
     
    for i, d in enumerate(joint.dofs):

        if d == 1:
            name = 'pid-' + joint.node.name + '-' + str(i) 

            p = class_name( joint.node.getObject('dofs'),
                            name = name )

            p.basis = [0] * 6
            p.basis[i] = 1
            res.append(p)

    return res


class PID:
    
    def __init__(self, robot, class_name = pid.Implicit):
        self.robot = robot
        self.pid = []
        self.index = {}

        off = 0
        for j in robot.joints:
            j.pid = _make_pid(j, class_name)
            self.pid.extend( j.pid )

            for i in range(len(j.pid)):
                self.index[ (j.name, i) ] = off
                off += 1

        # init values
        for pid in self.pid:
            pid.kp = 1
            pid.kd = 0

            

    def reset(self):
        for pid in self.pid:
            pid.reset()


    def pre_step(self, dt):
        for pid in self.pid:
            pid.pre_step(dt)

    def post_step(self, dt):
        for pid in self.pid:
            pid.post_step(dt)

    # data is a dictionay { (name, index) : value } to set attribute
    # 'what' of pid 'index' in joint 'name' to 'value'
    def set(self, what, data):
        for (name, index) in data:
            joint = getattr(self.robot, name)
            value = data[ (name, index) ]

            setattr(joint.pid[index], what, value)

    # 'who' is a set { (name, index) } (or dictionary but only keys
    # are used). returns a dictionary { (name, index): value } giving
    # attribute 'what' from pid 'index' in joint 'name'
    def get(self, what, who):
        res = {}
        for (name, index) in who:
            joint = getattr(self.robot, name)
            value = getattr(joint.pid[index], what)
            res[ (name, index) ] = value

        return res



# data.states = ['start', 'foo', 'bar']
# data.transitions = [('event1', 'start', foo'), ('event2', 'start', bar') ]

class FSM:

    def __init__(self, data):
        self.data = data
        self.current = None
        # TODO build an optimized structure for transitions lookup

    def start(self):
        self.enter_state( self.data.start )
        

    def enter_state(self, s):
        self.current = s
        if not (self.current in self.data.states):
            raise Exception('unknown state ' + s )

        cb = getattr(self.data, 'enter_' + self.current, None)
        if cb != None: cb()


    def step(self):
        if self.current == None:
            raise Exception('machine not started !')
        
        # candidate transitions
        candidates = [ x for x in self.data.transitions if x[1] == self.current ]

        old = self.current
        
        for (name, src, dst) in candidates:

            # call cb to see if it matches
            if getattr(self.data, name)():

                # TODO this should go to enter_state
                # exit current
                cb = getattr(self.data, 'exit_' + self.current, None)
                if cb != None: cb()

                # new state
                self.enter_state( dst ) 

        if self.current == old:
            cb = getattr(self.data, 'while_' + self.current, None)
            if cb != None: cb()



# perform broyden update on J so that J u = v
def broyden(J, u, v):
    norm2 = np.inner(u, u)

    if norm2 >= 1e-10:
        Ju = J.dot(u)
        lhs = (v - Ju) / norm2
        J += np.outer(lhs, u)
            


# a kinematic constraint. 'matrix' is the constraint jacobian, and
# 'value' should be understood as 'desired position - current
# position'. for a velocity constraint, simply set: value = dt * v
class Constraint:

    # note: dofs must all have the same dofs
    def __init__(self, name, parent, dofs, rows):

        self.parent = parent
        self.node = parent.createChild(name)

        self.compliance = 0
        self.damping = 0

        # TODO rename ?
        self._constrained_dofs = dofs
        
        input = []

        self.rows = rows
        self.cols = 0

        for n in dofs:
            input.append( '@{0}/{1}'.format( Tools.node_path_rel(self.node, n.getContext() ),
                                             n.name ) )
            self.cols += tool.matrix_size( n )

        self.matrix = np.zeros( (self.rows, self.cols) )
        self.value = np.zeros( self.rows )

        self.dofs = self.node.createObject('MechanicalObject',
                                           name = 'dofs',
                                           template = 'Vec1d',
                                           position = concat( [0] * self.rows ) )

        template = tool.template( dofs[0] )

        self.map = self.node.createObject('AffineMultiMapping',
                                          name = 'map',
                                          hard_positions = True,
                                          template = '{0}, Vec1d'.format( template ),
                                          input = concat( input ),
                                          output = '@dofs',
                                          matrix = concat( self.matrix.reshape( self.matrix.size ).tolist() ),
                                          value = concat( -self.value ) )

        self.ff = self.node.createObject('UniformCompliance',
                                         name = 'ff',
                                         template = 'Vec1d',
                                         compliance = self.compliance,
                                         damping = self.damping )
        
    def update(self):
        self.map.matrix = concat( self.matrix.reshape(self.matrix.size).tolist() )
        self.map.value = concat( -self.value )
        self.map.init()

        self.ff.compliance = self.compliance
        self.ff.damping = self.damping
        self.ff.init()


    def enable(self, value):
        if not value:
            self.node.detachFromGraph()
        else:
            self.parent.addChild( self.node )


    def enabled(self):
        return len(self.node.getParents()) > 0
        
    def constrained_velocity(self):
        res = np.zeros( self.cols )

        off = 0
        # TODO optimize !
        for d in self._constrained_dofs:
            dim = tool.matrix_size( d )
            s = d.findData('velocity').getValueString()
            res[off:off + dim] = map(float, s.split(' '))
            off += dim

        return res
