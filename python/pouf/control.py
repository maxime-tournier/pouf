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

        for j in robot.joints:
            j.pid = _make_pid(j)
            self.pid.extend( j.pid )
            
        for pid in self.pid:
            pid.kp = -1
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

    def set(self, what, data):
        for (name, index) in data:
            joint = getattr(self.robot, name)
            value = data[ (name, index) ]

            setattr(joint.pid[index], what, value)
            
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
