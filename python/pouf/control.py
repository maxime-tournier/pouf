import pid


# build pid joints
def _make_pid(joint):
    res = []
     
    for i, d in enumerate(joint.dofs):

        if d == 1:
            name = 'pid-' + joint.node.name + '-' + str(i) 

            p = pid.Implicit( joint.node.getObject('dofs'),
                              name = name )

            p.basis = [0] * 6
            p.basis[i] = 1
            res.append(p)

    return res


class PID:
    
    def __init__(self, robot):
        self.robot = robot
        self.pid = []
        self.index = {}

        off = 0
        for j in robot.joints:
            j.pid = _make_pid(j)
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

                # exit current
                cb = getattr(self.data, 'exit_' + self.current, None)
                if cb != None: cb()

                # new state
                self.enter_state( dst ) 

        if self.current == old:
            cb = getattr(self.data, 'while_' + self.current, None)
            if cb != None: cb()



# a kinematic constraint. 'matrix' is the constraint jacobian, and
# 'value' should be understood as 'desired position - current
# position'. for a velocity constraint, simply set: value = dt * v
class Constraint:

    # note: dofs must all have the same dofs
    def __init__(self, name, parent, dofs, dim):

        self.dim = dim
        self.node = parent.createChild(name)

        self.compliance = 0
        self.damping = 0
        
        input = []
        dofs_dim = 0

        for n in dofs:
            input.append( '@{0}/{1}'.format( Tools.node_path_rel(self.node, n.getContext() ),
                                             n.name ) )
            dofs_dim += pouf.tool.matrix_size( n )

        self.matrix = np.zeros( (dim, dofs_dim) )
        self.value = np.zeros( dim )


        self.dofs = self.node.createObject('MechanicalObject',
                                           name = 'dofs',
                                           template = 'Vec1d',
                                           position = tool.concat( [0] * dim ) )

        template = pouf.tool.template( dofs[0] )

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

