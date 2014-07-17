import math


# setup controller with reasonable defaults
def setup(servo, base = 5e2):
    
    # servo setup
    stiff = 10 * base # 5e3
    normal = base
    soft = base / 2 # 1e2

    for p in servo.pid:
        p.kp = normal
        p.kd = 5
        p.ki = 10

    ankle = normal
    
    kp = {
        ('lphal', 0): ankle,
        ('rphal', 0): ankle,
        
        ('lankle', 0): ankle,
        ('lankle', 1): ankle,
        ('lankle', 2): ankle,

        ('rankle', 0): ankle,
        ('rankle', 1): ankle,
        ('rankle', 2): ankle,
        
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
    overdamped = 5
        
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


# posture collection
def rest( flex = math.pi / 14 ):
    pose = { ('lknee', 0): 2 * flex,
             ('rknee', 0): 2 * flex,
            
             ('lhip', 0): -2 * flex,
             ('rhip', 0): -2 * flex,

             ('lhip', 2): 0,
             ('rhip', 2): 0,


             # ('lankle', 0): flex / 2,
             # ('rankle', 0): flex / 2,

             ('lankle', 0): -flex/2,
             ('rankle', 0): -flex/2,


             ('lankle', 2): 0,
             ('rankle', 2): 0,


             ('lshoulder', 2): 1.5 * flex,
             ('rshoulder', 2): -1.5 * flex,
             
             ('lelbow', 0): -2 * flex,
             ('relbow', 0): -2 * flex,
    }
    return pose
