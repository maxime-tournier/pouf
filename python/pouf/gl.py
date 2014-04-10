import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *


def line_strip( vertex_list ):
    glBegin(GL_LINE_STRIP)
    for v in vertex_list:
        glVertex( v )
    glEnd()


def line( start, end ):
    glBegin(GL_LINES)
    glVertex( start )
    glVertex( end )
    glEnd()
