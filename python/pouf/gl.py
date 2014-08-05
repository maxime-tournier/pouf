import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *


def line_strip( vertex_list, color_list = None ):
    glBegin(GL_LINE_STRIP)
    for i, v in enumerate(vertex_list):
        if color_list:
            glColor( color_list[i] )
        glVertex( v )
    glEnd()


def line( start, end ):
    glBegin(GL_LINES)
    glVertex( start )
    glVertex( end )
    glEnd()


def color( c ):
    glColor( c )

def points( *args ):
    glBegin(GL_POINTS)
    for v in args:
        glVertex( v )
    glEnd()


def point_size( x ):
    glPointSize(x)

def line_width( x ):
    glLineWidth(x)


def enable(what, value):
    if value: glEnable(what)
    else: glDisable(what)


def lighting( b ):
    enable(GL_LIGHTING, b)


def depth_test( b ):
    enable(GL_DEPTH_TEST, b)


def blend( b ):
    if b:
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable( GL_BLEND )
    else:
        glDisable( GL_BLEND )
        
