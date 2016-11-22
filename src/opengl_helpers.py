# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD
"""
Helper functions for use with OpenGL.

"""
from contextlib import contextmanager

import numpy as np
from OpenGL import GL as gl
from OpenGL import GLU as glu
from OpenGL import GLUT as glut

from helpers import Quat


def gl_font(name, height):
    """
    Return a handle to a bitmapped OpenGL font.

    Parameters
    ----------
    name : str
        The name of the font. The following fonts are available:

        - "fixed" for Fixed
        - "times" for Times Roman
        - "helvetica" for Helvetica

    height : int
        The height of the fonts, in points. The following heights are possible
        for each font:

        - "fixed": 13 or 15 points.
        - "times": 10 or 24 points.
        - "helvetica": 10, 12, or 18 points.

    Returns
    -------
    ctypes.c_void_p
        A handle to the requested font.

    Raises
    ------
    ValueError
        If a bad font/size combination is requested.

    """
    name = name.lower()

    if name not in ("fixed", "times", "helvetica"):
        raise ValueError("Unknown font requested.")

    if name == "fixed":
        if height == 13:
            return glut.GLUT_BITMAP_8_BY_13
        elif height == 15:
            return glut.GLUT_BITMAP_9_BY_15
        else:
            raise ValueError("The Fixed font can only be 13 or 15 points high.")

    elif name == "times":
        if height not in (10, 24):
            raise ValueError("The Times Roman font can only be 10 or 24 points "
                             "high.")
        return getattr(glut, "GLUT_BITMAP_TIMES_ROMAN_{}".format(height))

    elif name == "helvetica":
        if height not in (10, 12, 18):
            raise ValueError("The Helvetica font can only be 10, 12, or 18 "
                             "points high.")
        return getattr(glut, "GLUT_BITMAP_HELVETICA_{}".format(height))


@contextmanager
def gl_flag(gl_type):
    """
    Context manager for enabling a single flag.

    Parameters
    ----------
    gl_type : gl.GLenum
        The flag to be used.

    """
    gl.glEnable(gl_type)
    try:
        yield
    finally:
        gl.glDisable(gl_type)


@contextmanager
def gl_ortho(width, height):
    """
    Context manager for enabling 2D orthographic projection.

    Parameters
    ----------
    width : int
        The width of the viewing region.
    height : int
        The height of the viewing region.

    """
    with new_matrix(gl.GL_PROJECTION, gl.GL_MODELVIEW):
        gl.glLoadIdentity()
        glu.gluOrtho2D(-width / 2, width / 2, -height / 2, height / 2)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        with new_matrix(gl.GL_MODELVIEW, gl.GL_PROJECTION):
            gl.glLoadIdentity()
            yield


@contextmanager
def gl_primitive(gl_mode):
    """
    Context manager for creating primitives.

    Parameters
    ----------
    gl_mode : gl.GLenum
        The mode used for generating primitives.

    """
    gl.glBegin(gl_mode)
    try:
        yield
    finally:
        gl.glEnd()


@contextmanager
def new_state():
    """
    Context manager for temporarily changing parameters.

    """
    gl.glPushAttrib(gl.GL_CURRENT_BIT)
    yield
    gl.glPopAttrib(gl.GL_CURRENT_BIT)


@contextmanager
def new_matrix(mode_start=None, mode_end=None):
    """
    Context manager for temporarily working with a new matrix.

    Parameters
    ----------
    mode_start : gl.GLenum
        The matrix mode to use for the new matrix.
    mode_end : gl.GLenum
        The matrix mode to use after finishing with the new matrix.

    """
    if mode_start is not None:
        gl.glMatrixMode(mode_start)
    gl.glPushMatrix()
    yield
    gl.glPopMatrix()
    if mode_end is not None:
        gl.glMatrixMode(mode_end)


class Shape(object):
    """
    A drawable shape made out of polygons.

    Parameters
    ----------
    vertices : Sequence[Sequence[float]]
        A sequence of 3D coordinates representing the vertices on the shape.
    colours : Sequence[Sequence[float]]
        A sequence of RGB values between 0 and 1, assigned to vertices.
    edges : Sequence[Sequence[int]]
        A sequence of 2-tuple representing the indices of the vertices to be
        joined.
    surfaces : Sequence[Sequence[int]]
        A sequence of the list of indices of vertices forming a surface,
        in order.

    Attributes
    ----------
    vertices : Sequence[Sequence[float]]
        A sequence of 3D coordinates representing the vertices on the shape.
    colours : Sequence[Sequence[float]]
        A sequence of RGB values between 0 and 1, assigned to vertices.
    edges : Sequence[Sequence[int]]
        A sequence of 2-tuple representing the indices of the vertices to be
        joined.
    surfaces : Sequence[Sequence[int]]
        A sequence of the list of indices of vertices forming a surface,
        in order.

    """
    def __init__(self, vertices, colours, edges, surfaces):
        self.vertices = vertices
        self.colours = colours
        self.edges = edges
        self.surfaces = surfaces

    def draw(self, quaternion=(0, 0, 0, 1), edge_colour=(1, 1, 1)):
        """
        Draw the shape.

        Parameters
        ----------
        quaternion : Optional[Sequence[float]]
            The x, y, z, and w quaternion of the pose. Default is no rotation.
        edge_colour : Optional[Sequence[float]]
            The colour to draw the edges in, as RGB values between 0 and 1.
            Default is white.

        """
        with new_matrix():
            gl.glRotate(*Quat.to_axis(quaternion))
            self._draw_components(self.vertices, self.colours, self.edges,
                                  self.surfaces, edge_colour)

    @staticmethod
    def _draw_components(vertices, colours, edges, surfaces, edge_colour):
        # Draw the surfaces.
        with gl_primitive(gl.GL_QUADS):
            for surface in surfaces:
                for i, vertex in enumerate(surface):
                    gl.glColor3fv(colours[i % len(colours)])
                    gl.glVertex3fv(vertices[vertex])
        gl.glColor3fv(edge_colour)

        # Draw the edges.
        with gl_primitive(gl.GL_LINES):
            for edge in edges:
                for vertex in edge:
                    gl.glVertex3fv(vertices[vertex])


class Cube(Shape):
    """
    A cube which can be drawn.

    The cube is centred at the origin, with a size of one unit per side.

    Parameters
    ----------
    scale : Optional[float]
        The scale of the cube. Default is 1.

    Attributes
    ----------
    vertices
    colours
    edges
    surfaces

    """
    def __init__(self, scale=1):
        vertices = np.array([
            (1, -1, -1), (1, 1, -1),
            (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1),
            (-1, -1, 1), (-1, 1, 1),
        ]) * 0.5 * scale

        colours = (
            (0.5, 0.5, 0.5),
        )

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

        super(Cube, self).__init__(vertices, colours, edges, surfaces)
