import Sofa

import SofaPython.mass

from Compliant import tool
from Compliant.types import Quaternion, Rigid3
import numpy as np

import subprocess
from .tool import concat

inertia_forces = False
    
class MassInfo:
        pass


class Body:
    # generic rigid body
    def __init__(self, name = "unnamed"):
        self.name = name         # node name
        self.collision = None # collision mesh
        self.visual = None    # visual mesh
        self.dofs = Rigid3()   # initial dofs
        self.mass = 1         # mass 
        self.inertia = [1, 1, 1] # inertia tensor
        self.color = [1, 1, 1]   # not sure this is used 
        self.offset = None       # rigid offset for com/inertia axes
        self.inertia_forces = inertia_forces # compute inertia forces flag
        self.groups = []
        self.mu = 0           # friction coefficient
        self.scale = [1, 1, 1]

        # TODO more if needed (scale, color)
                
    def mass_from_mesh(self, name, density = 1000.0):
        info = SofaPython.mass.RigidMassInfo()
        info.setFromMesh(name, density, self.scale)
        
        self.mass = info.mass
        
        # TODO svd inertia tensor, extract rotation quaternion
        
        self.inertia = info.diagonal_inertia
                
        self.offset = Rigid3()
        
        self.offset.center = info.com
        self.offset.orient = info.inertia_rotation
        
        # TODO handle principal axes ?
        
        # scaling ?
        if self.scale != [1, 1, 1]:
            print "handling scaling"
            s = np.array(self.scale)
            vol = s[0] * s[1] * s[2]
            
            self.mass = vol * self.mass
            self.com = vol * s * np.array(info.com)
            

    def insert(self, node):
        res = node.createChild( self.name )
        
        # mass offset, if any
        off = Rigid3()
        
        if self.offset is not None:
            off = self.offset

        # kinematic dofs
        f = self.dofs * off

        # print('test', concat(f) )
        dofs = res.createObject('MechanicalObject',
                                template = 'Rigid',
                                position = concat(f),
                                name = 'dofs' )
        
        # dofs are now located at the mass frame, good
        mass = res.createObject('RigidMass', 
                                template = 'Rigid',
                                name = 'mass', 
                                mass = self.mass, 
                                inertia = concat(self.inertia),
                                inertia_forces = self.inertia_forces,
                                draw = True )
                
        # user node i.e. the one the user provided
        user = res.createChild( 'user' )

        dofs = user.createObject('MechanicalObject',
                                 template = 'Rigid',
                                 position = concat(off.inv()),
                                 name = 'dofs')

        user.createObject('AssembledRigidRigidMapping',
                          template = 'Rigid,Rigid',
                          source = '0 ' + concat(off.inv()) )
                
        # visual model
        if self.visual is not None:
            visual_template = 'ExtVec3f'
            
            visual = user.createChild( 'visual' )
            ogl = visual.createObject('OglModel', 
                                      template = visual_template, 
                                      name = 'mesh', 
                                      fileMesh = self.visual, 
                                      color = concat(self.color), 
                                      scale3d = concat(self.scale))
                        
            visual_map = visual.createObject('RigidMapping', 
                                             template = 'Rigid3d' + ',' + visual_template, 
                                             input = '@../')
        # collision model
        if self.collision is not None:
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
                        
            model = collision.createObject('TriangleModel', 
                                           name = 'model',
                                           template = 'Vec3d',
                                           contactFriction = self.mu)
            if self.groups != []:
                model.group = concat(self.groups)
                
            collision.createObject('RigidMapping',
                                   template = 'Rigid,Vec3d',
                                   input = '@../',
                                   output = '@./')

        self.node = res
        self.user = user
        return res



