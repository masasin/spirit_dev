from contextlib import contextmanager

import numpy as np
from OpenGL import GL as gl
from OpenGL import GLU as glu

from helpers import rotation_matrix


@contextmanager
def gl_flag(gl_type):
    gl.glEnable(gl_type)
    try:
        yield
    finally:
        gl.glDisable(gl_type)


@contextmanager
def gl_ortho(width, height):
    with new_matrix(gl.GL_PROJECTION, gl.GL_MODELVIEW):
        gl.glLoadIdentity()
        glu.gluOrtho2D(-width / 2, width / 2, -height / 2, height / 2)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        with new_matrix(gl.GL_MODELVIEW, gl.GL_PROJECTION):
            gl.glLoadIdentity()
            yield


@contextmanager
def gl_primitive(gl_type):
    gl.glBegin(gl_type)
    try:
        yield
    finally:
        gl.glEnd()


@contextmanager
def new_matrix(mode_start=None, mode_end=None):
    if mode_start is not None:
        gl.glMatrixMode(mode_start)
    gl.glPushMatrix()
    yield
    gl.glPopMatrix()
    if mode_end is not None:
        gl.glMatrixMode(mode_end)


class Shape(object):
    def __init__(self, vertices, edges, surfaces, colors):
        self.vertices = vertices
        self.edges = edges
        self.surfaces = surfaces
        self.colors = colors

    def draw(self, quaternion=(0, 0, 0, 1)):
        vertices = np.dot(self.vertices, rotation_matrix(quaternion).T)

        with gl_primitive(gl.GL_QUADS):
            for surface in self.surfaces:
                for i, vertex in enumerate(surface):
                    gl.glColor3fv(self.colors[i % len(self.colors)])
                    gl.glVertex3fv(vertices[vertex])
        gl.glColor3fv((1, 1, 1))

        with gl_primitive(gl.GL_LINES):
            for edge in self.edges:
                for vertex in edge:
                    gl.glVertex3fv(vertices[vertex])


class Cube(Shape):
    def __init__(self, size=1):
        vertices = np.array([
            (1, -1, -1), (1, 1, -1),
            (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1),
            (-1, -1, 1), (-1, 1, 1),
        ]) * size

        edges = (
            (0, 1), (0, 3), (0, 4),
            (2, 1), (2, 3), (2, 7),
            (6, 3), (6, 4), (6, 7),
            (5, 1), (5, 4), (5, 7),
        )

        surfaces = (
            (0, 1, 2, 3),
            (3, 2, 7, 6),
            (6, 7, 5, 4),
            (4, 5, 1, 0),
            (1, 5, 7, 2),
            (4, 0, 3, 6),
        )

        colors = (
            (0.5, 0.5, 0.5),
        )

        super(Cube, self).__init__(vertices, edges, surfaces, colors)
