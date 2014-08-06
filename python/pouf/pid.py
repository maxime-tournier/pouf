
import tool
from tool import concat

# PID controllers, explicit and implicit


class Explicit:
    def __init__(self, dofs, **args):
        # gains
        self.kp = 1
        self.kd = 0
        self.ki = 0
          
        self.pos = 0
        self.vel = 0
        
        # actuation basis
        self.basis = [0, 0, 0, 0, 0, 0]
        
        if 'name' not in args:
            args['name'] = 'pid'

        name = args['name']

        # insertion starts here
        self.node = dofs.getContext().createChild( name )
        
        self.dofs = self.node.createObject('MechanicalObject', 
                                           name = 'dofs',
                                           template = 'Vec1d',
                                           position = '0')
        
        self.map = self.node.createObject('ProjectionMapping',
                                          set = '0 ' + concat(self.basis) )

        self.ff = self.node.createObject('ConstantForceField',
                                         template = 'Vec1d',
                                         forces = '0')
                                    
        self.reset()


    def reset(self):
        self.integral = 0


    def get_force(self):
        return float(self.ff.forces)

    def set_force(self, value):
        self.ff.forces = str(value)

    def add_force(self, value):
        self.set_force( self.get_force() + value )


    def pid(self, dt):
        p = self.pos - self.dofs.position
        d = self.vel - self.dofs.velocity
        i = self.integral + dt * p

        return p, i, d

    def pre_step(self, dt):
        self.set_force( 0.0 )

        # update mapping in case something changed
        self.map.set = concat([0] + self.basis)
        self.map.offset = str(self.pos)
        self.map.init()         # TODO does this trigger apply ?
        
        e, e_sum, e_dot = self.pid(dt)
        tau = self.kp * e  +  self.ki * e_sum  +  self.kd * e_dot
        self.integral = e_sum
        
        self.update(dt)

    def post_step(self, dt):
        pass






#
class Implicit:
    
    def __init__(self, dofs, **args):

        # gains
        self.kp = 1.0
        self.kd = 0.0
        self.ki = 0.0
          
        self.pos = 0.0
        
        # actuation basis
        self.basis = [0, 0, 0, 0, 0, 0]
        
        if 'name' not in args:
            args['name'] = 'pid'

        # insertion start here
        self.node = dofs.getContext().createChild( args['name'] )

        self.dofs = self.node.createObject('MechanicalObject', 
                                      name = 'dofs',
                                      template = 'Vec1d',
                                      position = '0')
        
        self.map = self.node.createObject('ProjectionMapping',
                                     set = '0 ' + concat(self.basis) )
        
        self.ff = self.node.createObject('UniformCompliance',
                                       template = 'Vec1d',
                                       compliance = '0' )

        self.reset()
        

    def reset(self):
        self.integral = 0.0
        
    def pre_step(self, dt):

        # update mapping just in case
        self.map.set = concat([0] + self.basis)
        self.map.offset = str(self.pos)
        self.map.init()         # TODO is apply triggered ?

        stiff = self.kp 
        damping = self.kd
        
        self.ff.compliance = 1.0 / stiff
        self.ff.damping = damping

        # trigger compliance matrix recomputation 
        self.ff.init()

        self.set_force( self.ki * self.integral )

        # pid acces
        self.p = self.dofs.position
        self.d = self.dofs.velocity
        self.dt = dt

    # get/set/add explicit force 
    def get_force(self):
        return self._force
    
    def set_force(self, value):
        self._force = value
        self.dofs.externalForce = str(value)

    def add_force(self, value):
        self.set_force( self.get_force() + value )
        
    # force applied at the end of time step
    def post_force(self):
        v =  self.dofs.velocity
        p = self.dofs.position
        
        # elastic =  - self.kp * self.p
        elastic = self.dofs.force
        damping = - self.kd * v
        explicit = self.get_force()

        res = elastic + damping + explicit

        # print 'check:', elastic, self.dofs.force
        
        # print self.dofs.force, elastic, implicit
        return res
        
    # call this during onEndAnimationStep
    def post_step(self, dt):
        # update integral with error on time step end
        self.integral -= dt * self.dofs.position

        self.p = self.dofs.position
        self.d = self.dofs.velocity

        # save torque
        self.tau = self.post_force()
        


