from __future__ import division, print_function

from contextlib import contextmanager

import numpy as np
from OpenGL import GL as gl
from OpenGL import GLU as glu
from OpenGL import GLUT as glut
import pygame as pg

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from helpers import (get_pose_components, pose_from_components, quat2axis,
                     rotation_matrix)
from opengl_helpers import (gl_font, gl_flag, gl_ortho, gl_primitive,
                            new_state, Shape)


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
    model : Shape
        The model to be drawn.
    fov_vertical : Optional[float]
        The vertical size of the field of view, in degrees.
    fov_diagonal : Optional[float]
        The diagonal size of the field of view, in degrees.

    `fov_vertical` and `fov_diagonal` are mutually exclusive. If neither is
    specified, the default vertical field of view is set to 45 degrees.

    Attributes
    ----------
    size : Sequence[int]
        The width and height of the display, in pixels.
    width : int
        The width of the display, in pixels.
    height : int
        The height of the display, in pixels.
    fov_y : float
        The vertical field of view, in degrees.
    model : Shape
        The model to draw.
    textures : Sequence[gl.GLuint]
        A list of usable textures.

    Raises
    ------
    TypeError
        If both `fov_vertical` and `fov_diagonal` are provided.

    """
    pg.init()
    glut.glutInit()

    def __init__(self, size, model, fov_vertical=None, fov_diagonal=None):
        if fov_diagonal and fov_vertical:
            raise TypeError("Enter only one value for field of view size.")

        self.size = self.width, self.height = size
        pg.display.set_mode(size, pg.DOUBLEBUF | pg.OPENGL)

        if fov_vertical is not None:
            self.fov_y = fov_vertical
        elif fov_diagonal is not None:
            self.fov_y = self.fov_diagonal2vertical(fov_diagonal)
        else:
            self.fov_y = 45

        self.model = model
        self.textures = []
        self._old_rel_pos = np.array([0, 0, 0])
        self._old_rot_cam = (0, 0, 0, 0)

    def fov_diagonal2vertical(self, fov_diagonal):
        """
        Convert a diagonal field of view to vertical.

        Parameters
        ----------
        fov_diagonal : float
            The diagonal field of view.

        Returns
        -------
        float
            The vertical field of view.

        """
        aspect_ratio = self.width / self.height
        ratio_diagonal = np.sqrt(1 + aspect_ratio**2)
        return 2 * np.rad2deg(np.arctan(np.tan(np.deg2rad(fov_diagonal) / 2) /
                                        ratio_diagonal))

    def add_textures(self, *filenames):
        """
        Add images to the list of usable textures.

        Parameters
        ----------
        filenames : Sequence[str]
            A list of filenames of images to load.

        Raises
        ------
        pygame.error
            If the image cannot be loaded, or if the image format is not
            supported.

        """
        for filename in filenames:
            self.textures.append(gl.glGenTextures(1))
            img = pg.image.load(filename)
            texture_data = pg.image.tostring(img, "RGB", 1)
            width = img.get_width()
            height = img.get_height()
            self._init_texture(texture_data, width, height)

    def _init_texture(self, texture_data, width, height, texture_number=-1):
        self.select_texture(texture_number)
        gl.glTexParameter(target=gl.GL_TEXTURE_2D,
                          pname=gl.GL_TEXTURE_MIN_FILTER,
                          parameter=gl.GL_LINEAR)
        # Implementation does not accept kwargs. Order is target, level,
        # internalFormat, width, height, border, format, type, and pixels.
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, width, height, 0,
                        gl.GL_RGB, gl.GL_UNSIGNED_BYTE, texture_data)

    def select_texture(self, number=-1):
        """
        Bind a known texture for use.

        Parameters
        ----------
        number : Optional[int]
            The number of the texture, by the order it was added. Default is
            the latest texture.

        Raises
        ------
        IndexError
            If `number` is larger than the number of available textures.

        """
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.textures[number])

    def set_perspective(self, near=0.1, far=100):
        """
        Set up the perspective projection matrix.

        Parameters
        ----------
        near : Optional[float]
            The distance to the near clipping plane in the z-direction. Default
            is 10 cm.
        far : Optional[float]
            The distance to the far clipping plane in the z-direction. Default
            is 100 m.

        """
        glu.gluPerspective(self.fov_y, self.width / self.height, near, far)

    @staticmethod
    def clear():
        """
        Reset OpenGL buffers to preset values.

        """
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

    def draw_background(self, texture_number=0):
        """
        Draw the background image.

        Parameters
        ----------
        texture_number : Optional[int]
            The number of the texture, by the order it was added. Default is
            the first texture added.

        """
        self.select_texture(texture_number)
        with gl_flag(gl.GL_TEXTURE_2D):
            with gl_ortho(self.width, self.height):
                gl.glTranslatef(-self.width / 2, -self.height / 2, 0)
                with gl_primitive(gl.GL_QUADS):
                    for x, y in ((0, 0), (0, 1), (1, 1), (1, 0)):
                        gl.glTexCoord2f(x, y)
                        gl.glVertex3f(self.width * x, self.height * y, 0)

    def write_text(self, text, x=None, y=None, font=gl_font("fixed", 13),
                   colour=(0, 1, 0)):
        """
        Write text on the screen.

        Parameters
        ----------
        text : str
            The text to write.
        x : Optional[int]
            The horizontal position, in pixels, of the lower left pixel of the
            first line of the string. Default is 40% of the screen right of
            centre.
        y : Optional[int]
            The vertical position, in pixels, of the lower left pixel of the
            first line of the string. Default is 80% of the screen above centre.
        font : Optional[ctypes.c_void_p]
            The font to use. Default is 13-point Fixed.
        colour : Optional[Sequence[float]]
            The text colour, as RGB values between 0 and 1. Default is green.

        """
        if x is None:
            x = self.width * 0.2
        if y is None:
            y = self.height * 0.4
        with gl_ortho(self.width, self.height):
            with new_state():
                gl.glColor3fv(colour)
                gl.glRasterPos2f(x, y)
                glut.glutBitmapString(font, text)

    @staticmethod
    def _find_relative(pose_cam, pose_drone):
        """
        Find the relative positions and orientations of the camera and the
        drone.

        Parameters
        ----------
        pose_cam : PoseStamped
            The pose of the drone when the background image was taken.
        pose_drone : PoseStamped
            The current pose of the drone.

        Returns
        -------
        rel_pos : np.ndarray
            A 3-array with the x, y, and z positions of the relative positions
            of the drone, converted to the OpenGL coordinate system.
        rot_cam : np.ndarray
            A quaternion representing the orientation of the camera, in x, y, z,
            w format.
        rot_drone : np.ndarray
            A quaternion representing the orientation of the drone, in x, y, z,
            w format.

        """
        coords_cam, rot_cam = get_pose_components(pose_cam)
        coords_drone, rot_drone = get_pose_components(pose_drone)
        rel_pos = coords_drone - coords_cam
        return rel_pos, rot_cam, rot_drone

    def render(self, pose_cam, pose_drone, draw_background=True):
        """
        Render the scene.

        Parameters
        ----------
        pose_cam : PoseStamped
            The pose of the drone when the background image was taken.
        pose_drone : PoseStamped
            The current pose of the drone.
        draw_background : Optional[bool]
            Whether to draw the background. Default is True.

        """
        rel_pos, rot_cam, rot_drone = self._find_relative(pose_cam, pose_drone)

        # Set camera orientation.
        # Reset camera orientation.
        gl.glRotate(*self._old_rot_cam)
        self._old_rot_cam = quat2axis(-rot_cam)

        # Set new camera orientation.
        rot_cam[:3] *= -1  # z-axis is with respect to origin, not camera.
        gl.glRotate(*quat2axis(rot_cam))

        # Set camera position.
        # Convert position to OpenGL coordinate frame.
        rel_pos[1], rel_pos[2] = rel_pos[2], -rel_pos[1]

        # Move camera position.
        gl.glTranslate(*(rel_pos - self._old_rel_pos))
        self._old_rel_pos = rel_pos

        if draw_background:
            self.draw_background()
        self.model.draw(rot_drone)

    @contextmanager
    def step(self, wait=10):
        """
        Context manager for displaying a single frame.

        Parameters
        ----------
        wait : Optional[int]
            The time to wait before the next step, in milliseconds.

        """
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                quit()
        self.clear()
        yield
        pg.display.flip()
        pg.time.wait(wait)


class Visualizer(object):
    def __init__(self):


def test():
    screen = Screen((640, 360), model=Drone(), fov_diagonal=92)
    screen.add_textures("background.bmp", "bird.jpg")
    screen.select_texture(0)
    screen.set_perspective()

    pos_cam = [-1.5, -4, 4]
    rot_cam = [-0.1, 0, 0, 1]
    pos_drone = [-1.4, -1, 3.9]
    rot_drone = [-0.3, 0, 0, 1]
    pose_cam = pose_from_components(pos_cam, rot_cam)
    pose_drone = pose_from_components(pos_drone, rot_drone)

    while True:
        with screen.step():
            screen.render(pose_cam, pose_drone)


def main():
    rospy.init_node("visualizer", anonymous=True)
    Visualizer()
    rospy.loginfo("Started visualizer")
    rospy.spin()


if __name__ == '__main__':
    main()
