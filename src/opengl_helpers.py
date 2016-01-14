from contextlib import contextmanager

import numpy as np
from OpenGL import GL as gl
from OpenGL import GLU as glu

from helpers import rotation_matrix


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
def new_matrix(mode_start=None, mode_end=None):
    """
    Context manager for temporarily working with a new matrix.

    Parameters
    ----------
    mode_start : gl.GLenum
        The matrix mode to use for the new matrix.
    mode_end : int
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
        A sequence of 3D points representing the vertices on the shape.
    edges : Sequence[Sequence[int]]
        A sequence of 2-tuple representing the indices of the vertices to be
        joined.
    surfaces : Sequence[Sequence[int]]
        A sequence of the list of indices of vertices forming a surface,
        in order.
    colours : Sequence[Sequence[float]]
        A sequence of RGB values between 0 and 1, assigned to vertices.

    Attributes
    ----------
    vertices : Sequence[Sequence[float]]
        A sequence of 3D points representing the vertices on the shape.
    edges : Sequence[Sequence[int]]
        A sequence of 2-tuple representing the indices of the vertices to be
        joined.
    surfaces : Sequence[Sequence[int]]
        A sequence of the list of indices of vertices forming a surface,
        in order.
    colours : Sequence[Sequence[float]]
        A sequence of RGB values between 0 and 1, assigned to vertices.

    """
    def __init__(self, vertices, edges, surfaces, colours):
        self.vertices = vertices
        self.edges = edges
        self.surfaces = surfaces
        self.colours = colours

    def draw(self, quaternion=(0, 0, 0, 1), edge_colour=(1, 1, 1)):
        """
        Draw the shape.

        Parameters
        ----------
        quaternion : Sequence[float]
            The x, y, z, and w quaternion of the pose.
        edge_colour : Optional[Sequence[float]]
            The colour to draw the edges in. Default is white.

        """
        # Rotate the shape.
        vertices = np.dot(self.vertices, rotation_matrix(quaternion).T)

        # Draw the surfaces.
        with gl_primitive(gl.GL_QUADS):
            for surface in self.surfaces:
                for i, vertex in enumerate(surface):
                    gl.glColor3fv(self.colours[i % len(self.colours)])
                    gl.glVertex3fv(vertices[vertex])
        gl.glColor3fv(edge_colour)

        # Draw the edges.
        with gl_primitive(gl.GL_LINES):
            for edge in self.edges:
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
    edges
    surfaces
    colours

    """
    def __init__(self, scale=1):
        vertices = np.array([
            (1, -1, -1), (1, 1, -1),
            (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1),
            (-1, -1, 1), (-1, 1, 1),
        ]) * scale

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

        colours = (
            (0.5, 0.5, 0.5),
        )

        super(Cube, self).__init__(vertices, edges, surfaces, colours)
