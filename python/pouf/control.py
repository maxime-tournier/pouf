import pid

import tool
from tool import concat

import numpy as np
import math

import pickle
import pprint

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

            # print 'setting ', what, 'for ', name, index, ' = ', value


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

    def joints(self):
        res = []
        for j in self.robot.joints:
            for i in range(len(j.pid)):
                res.append( (j.name, i) )
        return res


    # only for gains
    def save(self, filename, attr = ['kp', 'kd', 'ki', 'kt'] ):
        data = {}
        for what in attr:
            data[what] = self.get(what, self.joints())

        with open(filename, 'w') as f:
            pprint.pprint(data, f)
            
    def load(self, filename):
        with open(filename) as f:
            data = eval(f.read())

            for what in data:
                self.set(what, data[what])
                

# data.states = ['start', 'foo', 'bar']
# data.transitions = [('event1', 'start', foo'), ('event2', 'start', bar') ]

class FSM:

    # 
    def __init__(self):
        self.current = None
        self.debug = None
        
        self.states = None
        self.transitions = None
        self.initial = None
        
    def start(self):

        # TODO build an optimized structure for transitions lookup ?
        # TODO checks on states/transitions/initial ?
        self.enter_state( self.initial )


    def call(self, name):
        cb = getattr(self, name, None)
        if cb != None: return cb()
        

    def enter_state(self, s):
        self.current = s
        if not (self.current in self.states):
            raise Exception('unknown state ' + s )

        self.call('enter_' + self.current )

        if self.debug != None:
            print 'state:', s

    def while_state(self, s):
        self.call('while_' + s )

    def exit_state(self, s):
        self.call('exit_' + s)
        
    def step(self):
        if self.current == None:
            raise Exception('machine not started !')
        
        # candidate transitions
        candidates = [ x for x in self.transitions if x[1] == self.current ]

        old = self.current
        
        for (name, src, dst) in candidates:

            # call cb to see if it matches
            if self.call(name):
                
                if self.debug != None:
                    print 'transition:', name
                
                # exit current
                self.exit_state( self.current )
                
                # new state
                self.enter_state( dst ) 
                break
            
        if self.current == old:
            self.while_state(self.current)



# perform broyden update on J so that J u = v
def broyden(J, u, v, kahan = None):

    norm2 = u.dot(u)

    # norm2 = np.linalg.norm(J.dot(u) - v)
    norm = math.sqrt( norm2 )

    if norm >= 1e-5:
        Ju = J.dot(u)
        lhs = (v - Ju) / norm2

        if kahan == None:
            J += np.outer(lhs, u)
        else:
            y = np.outer(lhs, u) - kahan
            t = J + y
            kahan[:] = (t - J) - y
            J[:] = t
            


