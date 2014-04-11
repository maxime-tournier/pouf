import math


# setup controller with reasonable defaults
def setup(servo):
    
    # servo setup
    stiff = 1e4
    normal = 1e3
    soft = 1e2

    for p in servo.pid:
        p.kp = normal
        p.kd = 5
        p.ki = 10

        
    kp = {
        ('lphal', 0): stiff,
        ('rphal', 0): stiff,
        
        ('lankle', 0): stiff,
        ('lankle', 1): stiff,
        ('lankle', 2): stiff,

        ('rankle', 0): stiff,
        ('rankle', 1): stiff,
        ('rankle', 2): stiff,
        
        ('lshoulder', 0): soft,
        ('lshoulder', 1): soft,
        ('lshoulder', 2): soft,
        
        ('rshoulder', 0): soft,
        ('rshoulder', 1): soft,
        ('rshoulder', 2): soft,

        ('lelbow', 0): soft,
        ('relbow', 0): soft,
    }

    # TODO more
    overdamped = 10
        
    kd = {
        ('lankle', 0) : overdamped,
        ('lankle', 1) : overdamped,
        ('lankle', 2) : overdamped,
        
        ('rankle', 0) : overdamped,
        ('rankle', 1) : overdamped,
        ('rankle', 2) : overdamped,
    }

    servo.set('kp', kp)
    servo.set('kd', kd)

    
    
# posture collection
def stand( flex = math.pi / 14 ):
    pose = { ('lknee', 0): 2 * flex,
             ('rknee', 0): 2 * flex,
            
             ('lhip', 0): -2 * flex,
             ('rhip', 0): -2 * flex,

             ('lhip', 2): 0,
             ('rhip', 2): 0,


             ('lankle', 0): -flex,
             ('rankle', 0): -flex,

             ('lankle', 2): 0,
             ('rankle', 2): 0,


             ('lshoulder', 2): flex,
             ('rshoulder', 2): -flex,
             
             ('lelbow', 0): -2 * flex,
             ('relbow', 0): -2 * flex,
    }
    return pose
