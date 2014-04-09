import Sofa

import subprocess

import quat
import numpy as np

from tool import concat

class Frame:
    # a rigid frame, group operations are available.
    
    # TODO kwargs
    def __init__(self, value = None):

        if value == None:
            self.__dict__['data'] = np.zeros(7)
        else:
            self.__dict__['data'] = value
            
        self.translation =  [0, 0, 0]
        self.rotation = [0, 0, 0, 1]


    def __getattr__(self, name):
        if name == 'rotation': return self.data[3:]
        elif name == 'translation': return self.data[:3]
        else: raise AttributeError('attribute "{0}" not found'.format(name))

    def __setattr__(self, name, value):
        if name == 'rotation': self.data[3:] = value
        elif name == 'translation': self.data[:3] = value
        else: raise AttributeError('attribute "{0}" not found'.format(name))

    def __str__(self):
        return concat(self.data)
            
    def copy(self):
        res = Frame( np.copy( self.data))
       
    def read(self, str):
        self.__dict__['data'] = map(float, str.split())
        return self

    def __mul__(self, other):
        res = Frame()
        res.translation =  self.translation + quat.rotate(self.rotation, other.translation)
        res.rotation = quat.prod( self.rotation, other.rotation) 
        
        return res

    def inv(self):
        res = Frame()
        res.rotation =  quat.conj( self.rotation )
        res.translation =  -quat.rotate(res.rotation, self.translation)
        return res


class MassInfo:
        pass

# front-end to sofa GenerateRigid tool. density unit is kg/m^3
def generate_rigid(filename, density = 1000.0, scale = [1, 1, 1]):
    cmd = Sofa.build_dir() + '/bin/GenerateRigid'

    tmp = '.tmp.rigid'
    args = [filename,
            tmp,
            density] + scale
    
    try:
        output = subprocess.call([cmd] + map(str, args))
        with open(tmp) as f:
            line = f.readlines()

    except OSError:
        # try the debug version
        cmd += 'd'
        
        try:
            output = subprocess.call([cmd] + map(str, args))
            with open(tmp) as f:
                line = f.readlines()
        except OSError:
            print 'error when calling GenerateRigid, do you have GenerateRigid built in SOFA ?'
            raise

    start = 1
        
    mass = float( line[start].split(' ')[1] )
    volm = float( line[start + 1].split(' ')[1] )
    inrt = map(float, line[start + 2].split(' ')[1:] )
    com = map(float, line[start + 3].split(' ')[1:] )
        
    # TODO extract principal axes basis if needed
    # or at least say that we screwd up
        
    res = MassInfo()

    # by default, GenerateRigid assumes 1000 kg/m^3 already
    res.mass = (density / 1000.0) * mass

    res.inertia = [mass * x for x in inrt]
    res.com = com

    return res



class Body:
    # generic rigid body
    def __init__(self, name = "unnamed"):
        self.name = name         # node name
        self.collision = None # collision mesh
        self.visual = None    # visual mesh
        self.dofs = Frame()   # initial dofs
        self.mass = 1         # mass 
        self.inertia = [1, 1, 1] # inertia tensor
        self.color = [1, 1, 1]   # not sure this is used 
        self.offset = None       # rigid offset for com/inertia axes
        self.inertia_forces = False # compute inertia forces flag
        self.group = None
        self.mu = 0           # friction coefficient
        self.scale = [1, 1, 1]

        # TODO more if needed (scale, color)
                
    def mass_from_mesh(self, name, density = 1000.0):
        info = generate_rigid(name, density, self.scale)
        
        self.mass = info.mass
        
        # TODO svd inertia tensor, extract rotation quaternion
        
        self.inertia = [info.inertia[0], 
                        info.inertia[3 + 1],
                        info.inertia[6 + 2]]
                
        self.offset = Frame()
        self.offset.translation = info.com

        # TODO handle principal axes ?


        # scaling ?
        if self.scale != [1, 1, 1]:
            print "handling scaling"
            s = np.array(self.scale)
            vol = s[0] * s[1] * s[2]
            
            mass = vol * self.mass
            com = vol * s * np.array(info.com)
            

    def insert(self, node):
        res = node.createChild( self.name )
        
        # mass offset, if any
        off = Frame()
        if self.offset != None:
            off = self.offset

        # kinematic dofs
        f = self.dofs * off

        dofs = res.createObject('MechanicalObject',
                                 template = 'Rigid',
                                 name = 'dofs',
                                 position = str(f) )
                                 
        # dofs are now located at the mass frame, good
        mass = res.createObject('RigidMass', 
                                template = 'Rigid',
                                name = 'mass', 
                                mass = self.mass, 
                                inertia = concat(self.inertia),
                                inertia_forces = self.inertia_forces )
                
        # user node i.e. the one the user provided
        user = res.createChild( self.name + '-user' )

        user.createObject('MechanicalObject',
                          template = 'Rigid',
                          position = str(off.inv()),
                          name = 'dofs')

        user.createObject('AssembledRigidRigidMapping',
                          template = 'Rigid,Rigid',
                        source = '0 ' + str(off.inv()) )
                
        # visual model
        if self.visual != None:
            visual_template = 'ExtVec3f'
            
            visual = user.createChild( 'visual' )
            ogl = visual.createObject('OglModel', 
                                      template = visual_template, 
                                      name = 'mesh', 
                                      fileMesh = self.visual, 
                                      color = concat(self.color), 
                                      scale3d = concat(self.scale))
                        
            visual_map = visual.createObject('RigidMapping', 
                                             template = 'Rigid' + ', ' + visual_template, 
                                             input = '@../')
        # collision model
        if self.collision != None:
            collision = user.createChild('collision')
                
            collision.createObject("MeshObjLoader", 
                                   name = 'loader', 
                                   filename = self.collision,
                                   scale3d = concat(self.scale) )
            
            collision.createObject('MeshTopology', 
                                   name = 'topology',
                                   triangles = '@loader.triangles')
            
            collision.createObject('MechanicalObject',
                                   name = 'dofs',
                                   position = '@loader.position')
                        
            model = collision.createObject('pouf.TriangleModel', 
                                           name = 'model',
                                           template = 'Vec3d',
                                           contactFriction = self.mu)
            if self.group != None:
                model.collisionGroupTags = str(self.group)
                
            collision.createObject('RigidMapping',
                                   template = 'Rigid,Vec3d',
                                   input = '@../',
                                   output = '@./')

        self.node = res
        self.user = user
        return res
