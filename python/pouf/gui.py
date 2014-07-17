
from PySide import QtGui, QtCore

import numpy as np


class Singleton(QtGui.QWidget):

    __instances = {}
    
    def __init__(self, parent = None ):
        name = self.__class__.__name__
        if name in Singleton.__instances:
            Singleton.__instances[name].hide()

        QtGui.QWidget.__init__(self, parent)
        Singleton.__instances[name] = self



class Flag(QtGui.QWidget):

    def __init__(self, name, parent = None):
        QtGui.QWidget.__init__(self, parent)

        lay = QtGui.QHBoxLayout()
        lay.addWidget(QtGui.QLabel(name + ':'))

        self.box = QtGui.QCheckBox()
        lay.addWidget( self.box )

        self.setLayout( lay )

    def enabled(self):
        return self.box.isChecked()

    
class Constraint(QtGui.QWidget):

    def __init__(self, parent = None):
        QtGui.QWidget.__init__(self, parent)

        grid = QtGui.QGridLayout()

        # am control
        grid.addWidget(QtGui.QLabel('enabled:'), 0, 0)
        self.enabled = QtGui.QCheckBox()
        grid.addWidget(self.enabled, 0, 1)

        grid.addWidget(QtGui.QLabel('compliance:'), 0, 2)
        self.compliance = QtGui.QDoubleSpinBox()
        self.compliance.setValue(1.0)
        self.compliance.setDecimals(5)
        self.compliance.setRange(0, 1e5)
        grid.addWidget(self.compliance, 0, 3)

        grid.addWidget(QtGui.QLabel('damping:'), 0, 4)
        self.damping = QtGui.QDoubleSpinBox()
        self.damping.setValue(1.0)
        self.damping.setDecimals(5)
        self.damping.setRange(0, 1e5)
        grid.addWidget(self.damping, 0, 5)

        self.setLayout( grid )
        

    def update(self, constraint):
        constraint.enable( self.enabled.isChecked() );
        constraint.compliance = self.compliance.value()
        constraint.damping = self.damping.value()
        constraint.ff.init()



