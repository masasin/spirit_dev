#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
Visualize the drone position using SPIRIT.

"""
from __future__ import division

from collections import deque
import os
import time
import threading

import numpy as np
from OpenGL import GL, GLU, GLUT
from PyQt5 import QtGui

import pygame as pg

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from helpers import Pose, Fov, Quat, d2r, unit_vector
from opengl_helpers import (gl_font, gl_flag, gl_ortho, gl_primitive,
                            new_matrix, new_state, Shape)


os.chdir(rospkg.RosPack().get_path("spirit"))

# Convenience
gl = GL
glu = GLU
glut = GLUT


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
    def __init__(self, size=0.3, height=0.13):
        offset = [0, 0, size]

        vertices = np.array([
            (1, -1, -1), (1, 1, -1),
            (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1),
            (-1, -1, 1), (-1, 1, 1),
        ]) * size
        vertices += offset
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
            # (0, 1, 2, 3),
            # (3, 2, 7, 6),
            # (6, 7, 5, 4),
            # (4, 5, 1, 0),
            # (1, 5, 7, 2),
            # (4, 0, 3, 6),
        )

        super(Drone, self).__init__(vertices, colours, edges, surfaces)

        self.arrow_vertices = np.array([
            (-1, 1, 1), (0, 1, -1), (1, 1, 1), (0, 1, 0),
            (-1, -1, 1), (0, -1, -1), (1, -1, 1), (0, -1, 0),
        ]) * size
        self.arrow_vertices += offset
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
        with new_matrix():
            gl.glRotate(*Quat.to_axis(quaternion))
            self._draw_components(self.arrow_vertices, self.arrow_colours,
                                  self.arrow_edges, self.arrow_surfaces,
                                  edge_colour)


class TexturesBase(object):
    """
    Implements methods which allow usage of textures.

    Attributes
    ----------
    textures : Sequence[gl.GLuint]
        A list of usable textures.

    """
    def setup_textures(self):
        """
        Set up texture variables.

        """
        self.textures = deque(maxlen=2)
        self._latest_texture = deque(maxlen=1)

    def add_textures(self, *images):
        """
        Add images to the list of usable textures.

        Parameters
        ----------
        images : Sequence[str | Image]
            A list of filenames or images to load.

        Raises
        ------
        pygame.error
            If the image cannot be loaded, or if the image format is not
            supported.
        TypeError
            If the input type is unsupported.

        """
        for texture_data, width, height in self.load_images(images):
            self._latest_texture.append((texture_data, width, height))

    def select_texture(self, texture_number=1):
        """
        Bind a known texture for use.

        Parameters
        ----------
        texture_number : Optional[int]
            The number of the texture, by the order it was added. Default is the
            latest texture.

        Raises
        ------
        IndexError
            If `texture_number` is larger than the number of available textures.

        """
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.textures[texture_number])

    def init_texture(self, texture_data, width, height, texture_number=1):
        """
        Initialize a texture for first use.

        Parameters
        ----------
        texture_data : Sequence
            The image data.
        width : int
            The width of the image.
        height : int
            The height of the image.
        texture_number : Optional[int]
            The number of the texture, by the order it was added. Default is the
            latest texture.

        Raises
        ------
        IndexError
            If `texture_number` is larger than the number of available textures.

        """
        self.textures.append(gl.glGenTextures(1))
        self.select_texture(texture_number)
        gl.glTexParameter(target=gl.GL_TEXTURE_2D,
                          pname=gl.GL_TEXTURE_MIN_FILTER,
                          parameter=gl.GL_LINEAR)
        # Implementation does not accept kwargs. Order is target, level,
        # internalFormat, width, height, border, format, type, and pixels.
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, width, height, 0,
                        gl.GL_RGB, gl.GL_UNSIGNED_BYTE, texture_data)

    def update_texture(self, texture_data, width, height, texture_number=1):
        """
        Update a known texture.

        Parameters
        ----------
        texture_data : Sequence
            The image data.
        width : int
            The width of the image.
        height : int
            The height of the image.
        texture_number : Optional[int]
            The number of the texture, by the order it was added. Default is the
            latest texture.

        Raises
        ------
        IndexError
            If `texture_number` is larger than the number of available textures.

        """
        self.select_texture(texture_number)
        # Implementation does not accept kwargs. Order is target, level,
        # xoffset, yoffset, width, height, format, type, and pixels.
        gl.glTexSubImage2D(gl.GL_TEXTURE_2D, 0, 0, 0, width, height,
                           gl.GL_RGB, gl.GL_UNSIGNED_BYTE, texture_data)

    def load_images(self, images):
        """
        Load images.

        Parameters
        ----------
        images : Sequence[str | Image]
            A list of filenames or images to load.

        Yields
        ------
        str
            The image data.
        int
            The width of the image.
        int
            The height of the image.

        Raises
        ------
        pygame.error
            If the image cannot be loaded, or if the image format is not
            supported.
        TypeError
            If the input type is unsupported.

        """
        for image in images:
            if isinstance(image, str):
                yield self._load_image_from_file(image)
            elif isinstance(image, Image):
                yield self._load_image_from_ros(image)
            else:
                raise TypeError("Cannot load image.")

    @staticmethod
    def _load_image_from_file(filename):
        """
        Load image from file.

        Parameters
        ----------
        filename : str
            The name of the file to be loaded.

        Returns
        -------
        str
            The image data.
        int
            The width of the image.
        int
            The height of the image.

        Raises
        ------
        pygame.error
            If the image cannot be loaded, or if the image format is not
            supported.

        """
        img = pg.image.load(filename)
        texture_data = pg.image.tostring(img, "RGB", True)
        return texture_data, img.get_width(), img.get_height()

    def _load_image_from_ros(self, image):
        """
        Load image from a ROS topic.

        Parameters
        ----------
        image : Image
            The Image to be loaded.

        Returns
        -------
        str
            The image data.
        int
            The width of the image.
        int
            The height of the image.

        """
        qt_image = QtGui.QPixmap.fromImage(
            QtGui.QImage(image.data, image.width, image.height,
	    QtGui.QImage.Format_RGB888))
        return qt_image, image.width, image.height


class RendererBase(TexturesBase):
    """
    Implements methods which enable rendering the scene.

    Attributes
    ----------
    textures
    size : np.ndarray[int]
        The width and height of the display, in pixels.
    width : int
        The width of the display, in pixels.
    height : int
        The height of the display, in pixels.
    model : Shape
        The model to draw.
    distance : float | None
        The distance at which to draw. If provided, the visualization can be
        zoomed in or out.
    fov_x : float
        The horizontal field of view, in degrees.
    fov_y : float
        The vertical field of view, in degrees.

    """
    def setup_renderer(self, size, model, distance,
                       fov_diagonal=None, fov_vertical=None):
        """
        Set up rendering parameters.

        `fov_vertical` and `fov_diagonal` are mutually exclusive. If neither is
        specified, the default vertical field of view is set to 45 degrees.

        Parameters
        ----------
        size : Sequence[int]
            The width and height of the display, in pixels.
        model : Shape
            The model to be drawn.
        distance : float | None
            The distance at which to draw. If provided, the visualization can be
            zoomed in or out.
        fov_vertical : Optional[float]
            The vertical size of the field of view, in degrees.
        fov_diagonal : Optional[float]
            The diagonal size of the field of view, in degrees.

        Raises
        ------
        TypeError
            If both `fov_vertical` and `fov_diagonal` are provided.

        """
        if fov_diagonal and fov_vertical:
            raise TypeError("Enter only one value for field of view size.")

        self.size = self.width, self.height = np.asarray(size)
        self.aspect_ratio = self.width / self.height
        self.model = model
        self.distance = distance

        self.text = {}

        self.pose_cam = self.pose_drone = None

        if fov_vertical is not None:
            self.fov_y = fov_vertical
        elif fov_diagonal is not None:
            self.fov_y = Fov.d2v(fov_diagonal, self.aspect_ratio)
        else:
            self.fov_y = 45
        self.fov_x = Fov.v2h(self.fov_y, self.aspect_ratio)
        self._image_distance = self.height / (2 * np.tan(d2r(self.fov_y) / 2))

    def render(self, pose_cam, pose_drone):
        """
        Render the scene.

        Parameters
        ----------
        pose_cam : Pose
            The pose of the drone when the background image was taken.
        pose_drone : Pose
            The current pose of the drone.

        """
        rel_pos, rot_cam, rot_drone = self._find_rel_pos(pose_cam, pose_drone)

        rot_cam[0] *= -1
        rot_cam[1], rot_cam[2] = rot_cam[2], rot_cam[1]

        rot_drone[0] *= -1
        rot_drone[1], rot_drone[2] = rot_drone[2], rot_drone[1]

        # Temporarily turn off zooming.
        if self.distance:
            scale = np.linalg.norm(rel_pos) / self.distance
            rel_pos = unit_vector(rel_pos) * self.distance
        else:
            scale = 1
        centre = self._find_drone_on_image(rel_pos)

        with new_matrix():
            # Set camera orientation.
            rot_cam[:3] *= -1  # z-axis is with respect to origin, not camera.
            gl.glRotate(*Quat.to_axis(rot_cam))

            # Set camera position.
            # Convert position to OpenGL coordinate frame first.
            rel_pos[1], rel_pos[2] = rel_pos[2], -rel_pos[1]
            gl.glTranslate(*rel_pos)

            self.draw_background(scale=scale, centre=centre)
            self.model.draw(rot_drone)

    # noinspection PyUnusedLocal
    def draw_background(self, texture_number=1, scale=1, centre=None,
                        rotation=0):
        """
        Draw the background image.

        Parameters
        ----------
        texture_number : Optional[int]
            The number of the texture, by the order it was added. Default is
            the latest texture added.
        scale : Optional[float]
            The amount of zoom applied to the image. Default is no zoom.
        centre : Optional[tuple[int]]
            The coordinates of the centre of zoom. Default is the centre of the
            image.
        rotation : Optional[float]
            The amount of clockwise rotation, in degrees. Default is no
            rotation.

        """
        # TODO: Fix method for zoom.
        def find_vertices(x, y):
            # centre_x, centre_y = centre

            # Temporarily turn off zooming
            # scale = 1
            centre_x, centre_y = self.size / 2

            # Real zooming code.
            vertex_x = self.width / 2 - scale * (centre_x - self.width * x)
            vertex_y = self.height / 2 - scale * (centre_y - self.height * y)
            return vertex_x, vertex_y

        # Clear background
        if texture_number != 0:
            self.draw_background(texture_number=0)

        try:
            self.select_texture(texture_number)
            if "no_texture" in self.text:
                del self.text["no_texture"]
        except IndexError:
            self.text["no_texture"] = ("No textures yet", None, None, (1, 0, 0))
            return

        if centre is None:
            centre = self.size / 2

        with gl_flag(gl.GL_TEXTURE_2D):
            with gl_ortho(self.width, self.height):
                gl.glRotate(rotation, 0, 0, 1)
                gl.glTranslate(-self.width / 2, -self.height / 2, 0)
                with gl_primitive(gl.GL_QUADS):
                    for x, y in ((0, 0), (0, 1), (1, 1), (1, 0)):
                        gl.glTexCoord2f(x, y)
                        tx, ty = find_vertices(x, y)
                        gl.glVertex(tx, ty, 0)

    def write_text(self, text, position=None, font=gl_font("fixed", 13),
                   colour=(0, 1, 0)):
        """
        Write text on the screen.

        Parameters
        ----------
        text : str
            The text to write.
        position : Optional[Sequence[int]]
            A sequence containing the horizontal and vertical positions, in
            pixels, of the lower left pixel of the first line of the text.
            Default is 40% of the screen right of centre, and 80% of the screen
            above centre.
        font : Optional[ctypes.c_void_p]
            The font to use. Default is 13-point Fixed.
        colour : Optional[Sequence[float]]
            The text colour, as RGB values between 0 and 1. Default is green.

        """
        if position is None:
            x = self.width * 0.2
            y = self.height * 0.4
        else:
            x, y = position
        with gl_ortho(self.width, self.height):
            with new_state():
                gl.glColor3fv(colour)
                gl.glRasterPos2f(x, y)
                glut.glutBitmapString(font, text)

    @staticmethod
    def _find_rel_pos(pose_cam, pose_drone):
        """
        Find the relative positions and orientations of the camera and the
        drone.

        Parameters
        ----------
        pose_cam : Pose
            The pose of the drone when the background image was taken.
        pose_drone : Pose
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
        rel_pos = pose_cam.position - pose_drone.position
        rel_pos[2] *= -1
        return rel_pos, pose_cam.orientation, pose_drone.orientation

    def _find_drone_on_image(self, rel_pos):
        """
        Find the location of the drone on the image.

        Parameters
        ----------
        rel_pos : np.ndarray
            A 3-array with the x, y, and z positions of the relative positions
            of the drone, without conversion.

        Returns
        -------
        centre_x : float
            The horizontal location of the drone, in pixels
        centre_y : float
            The vertical location of the drone, in pixels

        """
        # TODO: Consider rotation of the camera
        dx, dy, dz = rel_pos
        centre_x = self._image_distance * dx / dy + self.width / 2
        centre_y = self._image_distance * dz / dy + self.height / 2
        return centre_x, centre_y


class Screen(RendererBase):
    """
    Class for displaying and updating the screen.

    `fov_vertical` and `fov_diagonal` are mutually exclusive. If neither is
    specified, the default vertical field of view is set to 45 degrees.


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
    wait : Optional[int]
        The time to wait before the next step, in milliseconds. Default is 10.
    distance : Optional[float]
        The distance at which to draw. If provided, the visualization can be
        zoomed in or out. Default is to have no zoom.

    Attributes
    ----------
    textures
    size
    width
    height
    model
    distance
    fov_x
    fov_y
    wait : int
        The time to wait before the next step, in milliseconds.

    Raises
    ------
    TypeError
        If both `fov_vertical` and `fov_diagonal` are provided.

    """
    def __init__(self, size, model, fov_vertical=None, fov_diagonal=None,
                 wait=10, distance=None):
        # TODO: Make drone always horizontal?  NO
        # TODO: Keep drone in centre of image?
        # TODO: Allow rotation of background?
        # TODO: Keep image aligned with horizon?
        # TODO: Zoom only in, or both in and out?
        self.setup_textures()
        self.setup_renderer(size, model, distance, fov_diagonal, fov_vertical)

        self.wait = wait
        self.is_active = True
        self._bg_initialized = False

    def run(self):
        """
        Run the display.

        """
        pg.init()
        glut.glutInit()
        pg.display.set_caption("Past Image Viewer")
        pg.display.set_mode(self.size, pg.OPENGL)
        self.set_perspective()

        self.add_textures("media/blank.png")
        self.init_texture(*self._latest_texture.pop(), texture_number=0)

        while self.is_active:
            try:
                self.step()
            except pg.error:
                pg.quit()
                self.is_active = False
            pg.time.wait(self.wait)

    def step(self):
        """
        Show one frame.

        """
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                self.is_active = False
                return

        try:
            if self._bg_initialized:
                self.update_texture(*self._latest_texture.pop())
            else:
                self.init_texture(*self._latest_texture.pop())
                self._bg_initialized = True
        except IndexError:
            pass

        self.clear()
        try:
            self.render(self.pose_cam, self.pose_drone)
        except AttributeError:
            self.write_text("No data yet", colour=(1, 0, 0))
            return

        for text, position, font, colour in self.text.values():
            kwargs = {"text": text,
                      "position": position,
                      "font": font,
                      "colour": colour}
            self.write_text(**{k: v for k, v in kwargs.items()
                               if v is not None})
        pg.display.flip()

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
        glu.gluPerspective(self.fov_y, self.aspect_ratio, near, far)

    @staticmethod
    def clear():
        """
        Reset OpenGL buffers to preset values.

        """
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)


class VisualizerBase(object):
    def bg_callback(self, background):
        self.screen.add_textures(background)

    def pose_cam_callback(self, pose_cam):
        self.screen.pose_cam = Pose(pose_cam)

    def pose_drone_callback(self, pose_drone):
        self.screen.pose_drone = Pose(pose_drone)

    def tracked_callback(self, tracked):
        self.tracked = tracked.data
        if not self.tracked:
            self.screen.text["tracking"] = ("Tracking lost", None,
                                            gl_font("helvetica", 18), (1, 0, 0))
        elif "tracking" in self.screen.text:
            del self.screen.text["tracking"]

    def _start_screen(self, size):
        self.screen = Screen(size, model=Drone(), fov_diagonal=92)
        threading.Thread(target=self.screen.run).start()

    @property
    def is_active(self):
        return self.screen.is_active


class Visualizer(VisualizerBase):
    def __init__(self, size=(640, 480)):
        self._start_screen(size)
        rospy.Subscriber("/ardrone/past_image", Image, self.bg_callback,
                         queue_size=1)
        rospy.Subscriber("/ardrone/past_pose", PoseStamped,
                         self.pose_cam_callback, queue_size=1)
        rospy.Subscriber("/ardrone/pose", PoseStamped, self.pose_drone_callback,
                         queue_size=1)
        rospy.Subscriber("/ardrone/tracked", Bool, self.tracked_callback,
                         queue_size=1)


class TestVisualizer(VisualizerBase):
    def __init__(self, size=(640, 480)):
        rospy.Subscriber("/ardrone/image_raw", Image, self.bg_callback,
                         queue_size=1)
        self._start_screen(size)

        pos_cam = [-1.5, -4, 4]
        rot_cam = [-0, 0, 0, 1]
        pos_drone = [-1.5, -1, 4]
        rot_drone = [-0.3, 0, 0, 1]

        self.pose_cam_callback(Pose.generate_stamped(pos_cam, rot_cam))
        self.pose_drone_callback(Pose.generate_stamped(pos_drone, rot_drone))


def test_offline(size=(640, 480)):
    screen = Screen(size, model=Drone(), fov_diagonal=92)
    threading.Thread(target=screen.run).start()

    pos_cam = [1, 0, 0]
    rot_cam = [0, 0, 0, 1]
    pos_drone = [0, -3, 0]
    rot_drone = [0, 0, 0, 1]
    # pos_cam = [-0.5700, 0.08365, 0.0837]
    # rot_cam = [0.0006, 0.0042, 0.0166, 0.9999]
    # pos_drone = [-0.4767, 1.3597, 0.0770]
    # rot_drone = [0.0078, 0.0087, 0.0059, 0.9999]
    screen.pose_cam = Pose.generate_stamped(pos_cam, rot_cam)
    screen.pose_drone = Pose.generate_stamped(pos_drone, rot_drone)

    time.sleep(3)
    screen.add_textures("media/bird.jpg")


def shutdown_hook():
    pg.quit()


def main():
    rospy.init_node("visualizer", anonymous=True)
    rospy.on_shutdown(shutdown_hook)
    try:
        debug = rospy.get_param("~debug")
    except KeyError:
        rospy.logwarn("Running offline test.")
        debug = "offline"
    if debug == "offline":
        test_offline()
        return
    elif debug == "online":
        visualizer = TestVisualizer((640, 360))
    else:
        visualizer = Visualizer((640, 360))
    rospy.loginfo("Started visualizer")
    while visualizer.is_active:
        pass
    rospy.signal_shutdown("Done!")


if __name__ == '__main__':
    main()
