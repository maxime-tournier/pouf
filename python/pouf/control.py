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

