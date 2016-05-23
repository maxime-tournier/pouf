import Sofa

from Compliant import Tools
from Compliant.types import Quaternion, Rigid3, vec
import numpy as np

from tool import concat

import rigid

geometric_stiffness = 0

class Base:

    def __init__(self, name = 'joint'):

            self.dofs = [0] * 6
            self.body = []
            self.offset = []
            self.name = name

            # link constraints compliance
            self.compliance = 0
            self.stabilize = True

    def append(self, node, offset = None):
        self.body.append(node)
        self.offset.append(offset)
        # self.name = self.name + '-' + node.name
        
    # convenience: define joint using absolute frame and vararg nodes
    def absolute(self, frame, *nodes):
        for n in nodes:

            local = np.array( n.getObject('dofs').position ).view(Rigid3)
            self.append(n, local[0].inv() * frame)
        
    # joint dimension
    def dim(self):
        return sum( self.dofs )

        
    def insert(self, parent, **kwargs):
        
        node = parent.createChild(self.name)

        self.offset_node = []

        geometric = kwargs.get('geometric', geometric_stiffness)
        
        # build input data for multimapping
        input = []
        for b, o in zip(self.body, self.offset):
            if o is None:
                input.append( '@' + Tools.node_path_rel(node, b) + '/dofs' )
            else:
                joint = b.createChild( self.name + '-offset' )
                self.offset_node.append( joint )
                
                joint.createObject('MechanicalObject', 
                                   template = 'Rigid', 
                                   name = 'dofs' )
                
                joint.createObject('AssembledRigidRigidMapping', 
                                   template = "Rigid,Rigid",
                                   source = '0 ' + concat( o ),
                                   geometricStiffness = geometric)
                
                input.append( '@' + Tools.node_path_rel(node, b) + '/' + joint.name + '/dofs' )
                             
        assert len(input) > 0
   
        dofs = node.createObject('MechanicalObject', 
                                 template = 'Vec6d', 
                                 name = 'dofs', 
                                 position = '0 0 0 0 0 0' )

        mapping = node.createObject('RigidJointMultiMapping',
                                name = 'mapping', 
                                template = 'Rigid,Vec6d', 
                                input = concat(input),
                                output = '@dofs',
                                pairs = "0 0",
                                geometricStiffness = geometric
        )
                
        sub = node.createChild("constraints")
        
        sub.createObject('MechanicalObject', 
                         template = 'Vec1d', 
                         name = 'dofs')
		
        mask = [ (1 - d) for d in self.dofs ]
        
        mapping = sub.createObject('MaskMapping', 
                               name = 'mapping',
                               template = 'Vec6d,Vec1d',
                               input = '@../',
                               output = '@dofs',
                               dofs = concat(mask) )
		
        ff = sub.createObject('UniformCompliance',
                              name = 'compliance',
                              template = 'Vec1d',
                              compliance = self.compliance)

        if self.stabilize:
            stab = sub.createObject('Stabilization')
        
        self.ff = ff
        self.node = node
        
        return node


class Spherical(Base):
    def __init__(self, **args):
        Base.__init__(self)
        self.dofs = [0, 0, 0, 1, 1, 1]
        self.name = 'spherical'
        
        for k in args:
            setattr(self, k, args[k])

            
# this one has limits \o/
class Revolute(Base):

    # TODO make this 'x', 'y', 'z' instead
    def __init__(self, axis, **args):
        Base.__init__(self)
        self.dofs[3 + axis] = 1
        self.name = 'revolute'
        self.lower_limit = None
        self.upper_limit = None
        
        for k in args:
            setattr(self, k, args[k])


    def insert(self, parent):
        res = Base.insert(self, parent)

        if self.lower_limit == None and self.upper_limit == None:
            return res

        limit = res.createChild('limit')
        
        dofs = limit.createObject('MechanicalObject', template = 'Vec1d')
        map = limit.createObject('ProjectionMapping', template = 'Vec6d,Vec1d' )

        limit.createObject('UniformCompliance',
                           template = 'Vec1d',
                           compliance = '0' )
            
        limit.createObject('UnilateralConstraint');

        # don't stabilize as we need to detect violated
        # constraints first
        # limit.createObject('Stabilization');

        set = []
        position = []
        offset = []

        if self.lower_limit != None:
            set = set + [0] + self.dofs
            position.append(0)
            offset.append(self.lower_limit)

        if self.upper_limit != None:
            set = set + [0] + [ -x for x in self.dofs]
            position.append(0)
            offset.append(- self.upper_limit)
                
        map.set = concat(set)
        map.offset = concat(offset)
        dofs.position = concat(position)

        return res


class Cylindrical(Base):

    def __init__(self, axis ):
        Base.__init__(self)

        self.dofs[0 + axis] = 1
        self.dofs[3 + axis] = 1

        self.name = 'cylindrical'

class Prismatic(Base):

    def __init__(self, axis):
        Base.__init__(self)
        
        self.dofs[0 + axis] = 1
        self.name = 'prismatic'

class Planar(Base):
    
    def __init__(self, normal):
        Base.__init__(self)
        self.dofs = [ 
            int( (i != normal) if i < 3 else (i - 3 == normal) )
            for i in xrange(6)
        ]
        self.name = 'planar'

        
