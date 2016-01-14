from __future__ import division, print_function

from contextlib import contextmanager

import numpy as np
from OpenGL import GL as gl
from OpenGL import GLU as glu
from OpenGL import GLUT as glut
import pygame as pg

from helpers import (get_pose_components, pose_from_components, quat2axis,
                     rotation_matrix)
from opengl_helpers import gl_flag, gl_ortho, gl_primitive, Shape


class Drone(Shape):
    """
    A shape representing the drone.

    This assumes that the drone is square-shaped, with a set height. An arrow is
    drawn at the top of the drone, and coloured with navigation lights. (i.e.
    there is a red light on the left, and a green light on the right.)

    Parameters
    ----------
    size : Optional[float]
        The side of each of the drone's square sides. Default is 50 cm.
    height : Optional[float]
        The height of the drone. Default is 15 cm.

    Attributes
    ----------
    vertices
    colours
    edges
    surfaces
    arrow_vertices : Sequence[Sequence[float]]
        A sequence of 3D coordinates representing the vertices on the arrow.
    arrow_colours : Sequence[Sequence[float]]
        A sequence of RGB values between 0 and 1, assigned to arrow vertices.
    arrow_edges : Sequence[Sequence[int]]
        A sequence of 2-tuple representing the indices of the arrow vertices to
        be joined.
    arrow_surfaces : Sequence[Sequence[int]]
        A sequence of the list of indices of vertices forming the arrow, in
        order.

    See Also
    --------
    Shape

    """
    def __init__(self, size=0.5, height=0.15):
        vertices = np.array([
            (1, -1, -1), (1, 1, -1),
            (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1),
            (-1, -1, 1), (-1, 1, 1),
        ]) * size
        vertices[:, 1] *= height

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

        super(Drone, self).__init__(vertices, colours, edges, surfaces)

        self.arrow_vertices = np.array([
            (-1, 1, 1), (0, 1, -1), (1, 1, 1), (0, 1, 0),
            (-1, -1, 1), (0, -1, -1), (1, -1, 1), (0, -1, 0),
        ]) * size
        self.arrow_vertices[:, 1] *= height
        self.arrow_colours = (
            (1, 0, 0),  # Red on left
            (1, 1, 1),  # White in front
            (0, 1, 0),  # Green on right
            (1, 0.5, 0)  # Orange in back
        )
        self.arrow_edges = (
            (0, 1), (1, 2), (2, 3), (0, 3),
        )
        self.arrow_surfaces = (
            (0, 1, 2, 3),
        )

    def draw(self, quaternion=(0, 0, 0, 1), edge_colour=(1, 1, 1)):
        """
        Draw the drone.

        Parameters
        ----------
        quaternion : Optional[Sequence[float]]
            The x, y, z, and w quaternion of the pose. Default is no rotation.
        edge_colour : Optional[Sequence[float]]
            The colour to draw the edges in. Default is white.

        """
        super(Drone, self).draw(quaternion, edge_colour)

        # Draw arrow
        vertices = np.dot(self.arrow_vertices, rotation_matrix(quaternion).T)

        with gl_primitive(gl.GL_QUADS):
            for surface in self.arrow_surfaces:
                for i, vertex in enumerate(surface):
                    gl.glColor3fv(self.arrow_colours[i %
                                                     len(self.arrow_colours)])
                    gl.glVertex3fv(vertices[vertex])
        gl.glColor3fv(edge_colour)

        with gl_primitive(gl.GL_LINES):
            for edge in self.arrow_edges:
                for vertex in edge:
                    gl.glVertex3fv(vertices[vertex])


class Screen(object):
    """
    Class for displaying and updating the screen.

    Parameters
    ----------
    size : Sequence[int]
        The width and height of the display, in pixels.
    fov_vertical : Optional[float]
        The vertical size of the field of view, in degrees.
    fov_diagonal : Optional[float]
        The diagonal size of the field of view, in degrees.

    Notes
    -----
    `fov_vertical` and `fov_diagonal` are mutually exclusive, but at least one
    must be provided.
    """
    pg.init()
    glut.glutInit()

    def __init__(self, size, fov_vertical=None, fov_diagonal=None):
        if fov_diagonal and fov_vertical:
            raise TypeError("Enter only one value for field of view size.")

        self.size = self.width, self.height = size
        pg.display.set_mode(size, pg.DOUBLEBUF | pg.OPENGL)

        if fov_vertical is not None:
            self.fov = fov_vertical
        elif fov_diagonal is not None:
            self.fov = self.fov_diagonal2vertical(fov_diagonal)
        else:
            self.fov = 45

        self.model = None
        self.textures = []
        self._old_rel_pos = np.array([0, 0, 0])
        self._old_rot_cam = (0, 0, 0, 0)

    def fov_diagonal2vertical(self, fov_diagonal):
        aspect_ratio = self.width / self.height
        ratio_diagonal = np.sqrt(1 + aspect_ratio**2)
        return 2 * np.rad2deg(np.arctan(np.tan(np.deg2rad(fov_diagonal) / 2) /
                                        ratio_diagonal))

    def set_model(self, model):
        self.model = model

    def add_textures(self, *filenames):
        n_files = len(filenames)
        textures = gl.glGenTextures(n_files)
        if n_files == 1:
            textures = [textures]
        self.textures.extend(textures)

        for i, filename in enumerate(filenames):
            img = pg.image.load(filename)
            texture_data = pg.image.tostring(img, "RGB", 1)
            width = img.get_width()
            height = img.get_height()

            gl.glBindTexture(gl.GL_TEXTURE_2D, textures[i])
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER,
                               gl.GL_LINEAR)
            gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, width, height, 0,
                            gl.GL_RGB, gl.GL_UNSIGNED_BYTE, texture_data)

    def select_texture(self, number):
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.textures[number])

    def set_perspective(self, fov=None, near=0.1, far=100):
        if fov is None:
            fov = self.fov
        glu.gluPerspective(fov, self.width / self.height, near, far)

    @staticmethod
    def clear():
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

    def draw_background(self):
        self.select_texture(0)
        with gl_flag(gl.GL_TEXTURE_2D):
            with gl_ortho(self.width, self.height):
                gl.glTranslatef(-self.width / 2, -self.height / 2, 0)
                with gl_primitive(gl.GL_QUADS):
                    for x, y in ((0, 0), (0, 1), (1, 1), (1, 0)):
                        gl.glTexCoord2f(x, y)
                        gl.glVertex3f(self.width * x, self.height * y, 0)

    def write_text(self, text, x, y, font=glut.GLUT_BITMAP_HELVETICA_18):
        with gl_ortho(self.width, self.height):
            gl.glRasterPos2f(x, y)
            glut.glutBitmapString(font, text)

    @staticmethod
    def _find_relative(pose_cam, pose_drone):
        coords_cam, rot_cam = get_pose_components(pose_cam)
        coords_drone, rot_drone = get_pose_components(pose_drone)
        rel_pos = coords_drone - coords_cam
        rel_pos[2] *= -1
        return rel_pos, rot_cam, rot_drone

    def render(self, pose_cam, pose_drone, background=True):
        rel_pos, rot_cam, rot_drone = self._find_relative(pose_cam, pose_drone)

        gl.glRotatef(*self._old_rot_cam)
        self._old_rot_cam = quat2axis(-rot_cam)

        rot_cam[:3] *= -1
        gl.glRotate(*quat2axis(rot_cam))

        gl.glTranslatef(*(rel_pos - self._old_rel_pos))
        self._old_rel_pos = rel_pos

        if background:
            self.draw_background()
        self.model.draw(rot_drone)

    @contextmanager
    def step(self, wait=10):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                quit()
        self.clear()
        yield
        pg.display.flip()
        pg.time.wait(wait)

    def __del__(self):
        pg.quit()


def main():
    screen = Screen((640, 360), fov_diagonal=92)
    screen.set_model(Drone())
    screen.add_textures("background.bmp", "bird.jpg")
    screen.select_texture(0)
    screen.set_perspective()

    pos_cam = [-1.5, 4, -4]
    rot_cam = [-0.1, 0, 0, 1]
    pos_drone = [-1.4, 3.9, -1]
    rot_drone = [-0.3, 0, 0, 1]
    pose_cam = pose_from_components(pos_cam, rot_cam)
    pose_drone = pose_from_components(pos_drone, rot_drone)

    while True:
        with screen.step():
            screen.render(pose_cam, pose_drone)


if __name__ == '__main__':
    main()
